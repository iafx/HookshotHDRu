"""HD rumble for the Steam Controller (2026): host/hd_rumble_triton.h's reports and levels, and pad_triton.c's wiring.

The grips take mu-law audio (report 0x88, 31 samples a side, left from byte 2 and right from byte 33) after report
0x86 turns it on; the trackpads take tones (0x83) and a stop (0x82). The link picks the rate the controller keeps up
with: 8 kHz over the cable, 4 kHz through a dongle or Bluetooth (1 kHz Bluetooth off Windows). The mu-law values are
G.711's.

Run with: python -I -B tests/test_hd_rumble_triton.py
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PAD = ROOT / "host" / "pad_sdl.c"
TRITON = ROOT / "host" / "pad_triton.c"
HID = ROOT / "host" / "nn_hid.c"
SDL = ROOT / "host" / "hos_sdl.c"


def function(path, name):
    text = path.read_text(encoding="utf-8")
    found = re.search(re.escape(name) + r"\([^;{]*\)\s*\{", text)
    if not found:
        raise RuntimeError("no definition of " + name)
    at, start, depth = found.start(), found.end() - 1, 0
    for i in range(start, len(text)):
        depth += text[i] == "{"
        depth -= text[i] == "}"
        if depth == 0:
            return text[at:i + 1]
    raise RuntimeError("no end for " + name)


FIXTURE = r"""
#include <stdio.h>
#include "hd_rumble_triton.h"
static int fails;
#define CHECK(c) do { if (!(c)) { printf("line %d: %s\n", __LINE__, #c); fails++; } } while (0)
static int near(float a, float b, float e) { return a - b < e && b - a < e; }
int main(void) {
    /* links and formats */
    CHECK(triton_link(0x28DE, 0x1302) == TRITON_CABLE);
    CHECK(triton_link(0x28DE, 0x1303) == TRITON_BLUETOOTH);
    CHECK(triton_link(0x28DE, 0x1304) == TRITON_DONGLE && triton_link(0x28DE, 0x1305) == TRITON_DONGLE);
    CHECK(triton_link(0x28DE, 0x1205) == TRITON_NONE);   /* (the Steam Deck) */
    CHECK(triton_link(0x28DE, 0x11FF) == TRITON_NONE);   /* (Steam Input's virtual pad) */
    CHECK(triton_link(0x045E, 0x1302) == TRITON_NONE);
    int code, rate;
    triton_format(TRITON_CABLE, 1, &code, &rate); CHECK(code == 8 && rate == 8000);
    triton_format(TRITON_DONGLE, 1, &code, &rate); CHECK(code == 9 && rate == 4000);
    triton_format(TRITON_BLUETOOTH, 1, &code, &rate); CHECK(code == 9 && rate == 4000);
    triton_format(TRITON_BLUETOOTH, 0, &code, &rate); CHECK(code == 11 && rate == 1000);
    triton_format(TRITON_CABLE, 0, &code, &rate); CHECK(code == 8 && rate == 8000);
    /* the controller's input reports */
    const unsigned char in42[1] = {0x42}, in45[1] = {0x45}, in47[1] = {0x47}, other[1] = {0x01};
    CHECK(triton_is_input(in42, 1) && triton_is_input(in45, 1) && triton_is_input(in47, 1));
    CHECK(!triton_is_input(other, 1) && !triton_is_input(in42, 0));
    /* G.711 mu-law */
    CHECK(triton_mulaw(0) == 0xFF);
    CHECK(triton_mulaw(32767) == 0x80);
    CHECK(triton_mulaw(-32767) == 0x00);
    CHECK(triton_mulaw(-1) == 0x7F);
    CHECK(triton_mulaw(1000) == 0xCE);
    CHECK(triton_mulaw(-1000) == 0x4E);
    /* reports */
    uint8_t r[TRITON_REPORT_SIZE];
    triton_mode_report(1, 2, 8, r);
    CHECK(r[0] == 0x86 && r[1] == 2 && r[2] == 2 && r[3] == 8 && r[4] == 0);
    triton_mode_report(0, 5, 0, r);
    CHECK(r[0] == 0x86 && r[1] == 1 && r[2] == 5 && r[3] == 0);
    float left[TRITON_SAMPLES], right[TRITON_SAMPLES];
    for (int i = 0; i < TRITON_SAMPLES; i++) left[i] = 1.5f, right[i] = 0.0f;
    left[1] = -1.0f;
    triton_pcm_report(left, right, r);
    CHECK(r[0] == 0x88 && r[1] == 31);
    CHECK(r[2] == 0x80 && r[3] == 0x00);                   /* (full scale, clipped; full negative) */
    CHECK(r[32] == 0x80 && r[33] == 0xFF && r[63] == 0xFF); /* (left ends at byte 32, right starts at 33) */
    triton_tone_report(1, 0.5f, 300.0f, r);
    CHECK(r[0] == 0x83 && r[1] == 1 && (int8_t)r[2] == 0); /* (-6 dB + the trackpads' 6) */
    CHECK(r[3] == (300 & 0xFF) && r[4] == (300 >> 8) && r[5] == 60 && r[6] == 0 && r[7] == 0 && r[9] == 0);
    triton_tone_report(0, 0.001f, 300.0f, r);
    CHECK((int8_t)r[2] == 0 && r[0] == 0x82 && r[1] == 0 && r[3] == 0);   /* (too quiet: the stop) */
    triton_tone_report(0, 1.0f, 2000.0f, r);
    CHECK(r[0] == 0x83 && (int8_t)r[2] == 6 && (r[3] | r[4] << 8) == 1250);
    triton_tone_report(0, 0.03f, 10.0f, r);
    CHECK(r[0] == 0x83 && (int8_t)r[2] == -24 && (r[3] | r[4] << 8) == 40);
    /* the trackpads' resonances */
    CHECK(near(triton_avoid_rings(160.0f), 150.0f, 0.01f) && near(triton_avoid_rings(175.0f), 180.0f, 0.01f));
    CHECK(near(triton_avoid_rings(265.0f), 280.0f, 0.01f) && near(triton_avoid_rings(520.0f), 550.0f, 0.01f));
    CHECK(near(triton_avoid_rings(320.0f), 320.0f, 0.01f) && near(triton_avoid_rings(140.0f), 140.0f, 0.01f));
    /* the grips' gain */
    CHECK(triton_grip_gain(100.0f) == 1.0f && triton_grip_gain(500.0f) == 1.0f && triton_grip_gain(0.0f) == 1.0f);
    CHECK(near(triton_grip_gain(800.0f), 0.1778f, 0.001f) && near(triton_grip_gain(1200.0f), 0.1778f, 0.001f));
    CHECK(triton_grip_gain(600.0f) < 1.0f && triton_grip_gain(600.0f) > triton_grip_gain(700.0f));
    /* levels: the strength, then both bands to at most 0.9 together, the balance kept */
    float lv[2];
    triton_levels(0.2f, 0.1f, 100, lv); CHECK(near(lv[0], 0.2f, 1e-6f) && near(lv[1], 0.1f, 1e-6f));
    triton_levels(0.2f, 0.1f, 50, lv); CHECK(near(lv[0], 0.1f, 1e-6f) && near(lv[1], 0.05f, 1e-6f));
    triton_levels(0.8f, 0.4f, 100, lv); CHECK(near(lv[0] + lv[1], 0.9f, 1e-5f) && near(lv[0], 2.0f * lv[1], 1e-5f));
    triton_levels(0.5f, 0.5f, 0, lv); CHECK(lv[0] == 0.0f && lv[1] == 0.0f);
    /* tone cadence */
    CHECK(triton_tone_due(0.5f, 300.0f, 0, 0.0f, 0.0f, 0, 0));                         /* (a new tone: at once) */
    CHECK(!triton_tone_due(0.6f, 300.0f, 1, 0.5f, 300.0f, 5000000LL, 0));              /* (within the gap) */
    CHECK(triton_tone_due(0.6f, 300.0f, 1, 0.5f, 300.0f, 9000000LL, 0));               /* (a change, past the gap) */
    CHECK(!triton_tone_due(0.5f, 300.0f, 1, 0.5f, 300.0f, 20000000LL, 0));             /* (the same, not yet stale) */
    CHECK(triton_tone_due(0.5f, 300.0f, 1, 0.5f, 300.0f, 31000000LL, 0));              /* (refreshed) */
    CHECK(triton_tone_due(0.0f, 300.0f, 1, 0.5f, 300.0f, 1000000LL, 0));               /* (a stop: at once) */
    CHECK(!triton_tone_due(0.0f, 300.0f, 0, 0.0f, 300.0f, 1000000LL, 0));              /* (stopped: not again) */
    return fails ? 1 : 0;
}
"""


def build_and_run(test, source):
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "triton.c")
        with open(path, "w", encoding="utf-8") as f:
            f.write(source)
        exe = os.path.join(d, "triton")
        cc = os.environ.get("CC", "cc")
        made = subprocess.run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "host"), path, "-o", exe,
                               "-lm"], capture_output=True, text=True)
        test.assertEqual(made.returncode, 0, made.stderr)
        ran = subprocess.run([exe], capture_output=True, text=True)
        test.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)


class TritonRumbleTests(unittest.TestCase):
    def test_reports_and_levels(self):
        build_and_run(self, FIXTURE)

    def test_wired_in(self):
        self.assertIn("hos_triton_rumble(dev, v);", function(HID, "static void vibration_send"))
        self.assertIn("hos_triton_start();", function(PAD, "static int pads_start"))
        poll = function(PAD, "static void pads_poll")
        self.assertIn("pads[i].valve && hos_triton_playing()", poll)   # (no motor rumble to Valve's pads meanwhile)
        self.assertLess(poll.index("if (hd_poll(i, side, now)) continue;"), poll.index("hos_triton_playing()"))
        self.assertLess(poll.index("hos_triton_playing()"), poll.index("if (rumble_due("))
        self.assertIn("hos_sdl_pad_vendor(gc) == 0x28DE", poll)
        start = function(TRITON, "void hos_triton_start")
        self.assertIn('getenv("HOS_TRITON_RUMBLE")', start)
        self.assertIn("hos_sdl_hid_available()", start)
        play = function(TRITON, "static void play")
        self.assertIn('getenv("HOS_TRITON_TRACKPADS")', play)
        self.assertIn("hd_rumble_felt_amp(v[g][0])", play)   # (the game's small amplitudes, as the other pads feel them)
        self.assertIn("hos_sdl_rumble_percent()", play)      # (the menu's Rumble strength, as the other pads')
        self.assertIn("triton_avoid_rings(", play)

    def test_sdl_hid_layouts(self):
        text = SDL.read_text(encoding="utf-8")
        self.assertIn("struct hid_info3 { struct hid_info_head h; int bus_type; struct hid_info3 *next; };", text)
        self.assertIn("struct hid_info2 { struct hid_info_head h; struct hid_info2 *next; };", text)
        self.assertIn("(path, 0)", function(SDL, "void *hos_sdl_hid_open"))   # (SDL2's bExclusive)


if __name__ == "__main__":
    unittest.main()
