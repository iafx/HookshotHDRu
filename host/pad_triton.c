/* HD rumble on the Steam Controller (2026), written to its HID device through SDL's HID calls (hd_rumble_triton.h has
   the protocol and what was measured). SDL's own Valve drivers, and Steam Input's virtual pad, give it only the two
   grips' strengths; here each grip plays both bands of the game's vibration on its side as a wave, at the game's own
   frequencies, and each trackpad plays its side's high band as a tone.

   A thread of its own looks for the controller every 2 s (cable, Bluetooth or a Steam dongle, with or without Steam
   running), keeps the HID collection that sends its input, turns the grips' audio on and then writes one audio report
   every 31 samples at the link's rate (7.75 ms at 4 kHz, 3.9 ms at 8 kHz), with a trackpad tone between them when one
   is due. The game's values come from nn_hid.c's vibration_send (hos_triton_rumble), as they come for the Switch
   controllers' HD rumble; amplitudes go through the same felt curve (hd_rumble_felt_amp) and the menu's Rumble
   strength. While it plays, pad_sdl.c sends Valve's pads (the controller itself, or Steam's virtual pad for it) no
   motor rumble, so the two do not fight. If the controller goes away or its writes keep failing, the thread closes
   it and looks again; the motor rumble comes back meanwhile.

   HOS_TRITON_RUMBLE=0 leaves it out (the controller gets the motor rumble, as before); HOS_TRITON_TRACKPADS=0 keeps
   the grips' HD rumble and leaves the trackpads quiet. */
#include "hos.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "hos_sdl.h"
#include "hd_rumble.h"
#include "hd_rumble_triton.h"

void hos_host_thread_register_self(const char *name);   /* hos_prof.c */
int hos_sdl_rumble_percent(void);   /* pad_sdl.c: the menu's Rumble strength */

static SRWLOCK values_lock = SRWLOCK_INIT;
static float values[2][4] = {{0.0f, 160.0f, 0.0f, 320.0f}, {0.0f, 160.0f, 0.0f, 320.0f}};
static volatile LONG playing;   /* 1 while the grips' audio runs (pad_sdl.c: no motor rumble to Valve's pads) */
static void *device;            /* the open collection, while playing (the exit handler turns its audio off) */
static int device_format;

/* The game's vibration value for a side (0 left, 1 right): nn_hid.c's vibration_send */
void hos_triton_rumble(int side, const float value[4]) {
    AcquireSRWLockExclusive(&values_lock);
    memcpy(values[side & 1], value, sizeof values[0]);
    ReleaseSRWLockExclusive(&values_lock);
}
int hos_triton_playing(void) { return playing != 0; }

static int write_report(void *dev, const uint8_t report[TRITON_REPORT_SIZE]) {
    return hos_sdl_hid_write(dev, report, TRITON_REPORT_SIZE) > 0;
}
/* The grips' audio on or off (the controller takes it on channels 2 and 5, as LiveHaptics sends it) */
static void audio_mode(void *dev, int on, int format) {
    uint8_t r[TRITON_REPORT_SIZE];
    for (int channel = 2; channel <= 5; channel += 3) {
        triton_mode_report(on, channel, format, r);
        write_report(dev, r);
    }
}
static void at_exit(void) {   /* (the controller would keep its audio mode on after the game) */
    if (device) audio_mode(device, 0, device_format), audio_mode(device, 0, 0);
}

/* The controller's collection that sends input (a dongle has one per slot), open; its link in *link. NULL: none. */
static void *find_controller(int *link) {
    HosHidDevice found[16];
    const int n = hos_sdl_hid_list(TRITON_VENDOR, found, 16);
    for (int i = 0; i < n; i++) {
        const int l = triton_link(TRITON_VENDOR, found[i].product);
        if (l == TRITON_NONE) continue;
#ifdef _WIN32
        if (found[i].usage_page < 0xFF00) continue;   /* (Windows: the vendor collection, not its keyboard and mouse) */
#endif
        void *dev = hos_sdl_hid_open(found[i].path);
        if (!dev) continue;
        uint8_t r[TRITON_REPORT_SIZE];
        const long long until = hos_now_ns() + 300000000LL;
        int input = 0;
        while (!input && hos_now_ns() < until) {
            const int got = hos_sdl_hid_read(dev, r, sizeof r, 50);
            if (got < 0) break;
            input = triton_is_input(r, got);
        }
        if (input) {
            *link = l;
            return dev;
        }
        hos_sdl_hid_close(dev);
    }
    return NULL;
}

/* Plays until the controller goes away (or its writes keep failing) */
static void play(void *dev, int link) {
    int format, rate;
#ifdef _WIN32
    triton_format(link, 1, &format, &rate);
#else
    triton_format(link, 0, &format, &rate);
#endif
    const int trackpads = !(getenv("HOS_TRITON_TRACKPADS") && atoi(getenv("HOS_TRITON_TRACKPADS")) == 0);
    audio_mode(dev, 0, 0);
    Sleep(10);
    audio_mode(dev, 1, format);
    Sleep(200);   /* (the controller needs a moment between off and on) */
    device = dev, device_format = format;
    InterlockedExchange(&playing, 1);
    hos_log("hid: Steam Controller (2026) HD rumble on its grips (%s, %d samples/s)%s",
            link == TRITON_CABLE ? "cable" : link == TRITON_DONGLE ? "dongle" : "Bluetooth", rate,
            trackpads ? " and trackpads" : "");

    const long long period = 1000000000LL * TRITON_SAMPLES / rate;
    const float glide = 1.0f - expf(-1.0f / ((float)rate * 0.003f));   /* (3 ms: level changes do not click) */
    const float two_pi = 6.28318530718f;
    float amp[2][2] = {{0}}, phase[2][2] = {{0}};   /* [grip][band] */
    struct { int on; float amp, hz; long long ns; } tone[2] = {{0}};
    float left[TRITON_SAMPLES], right[TRITON_SAMPLES];
    uint8_t report[TRITON_REPORT_SIZE];
    long long next = hos_now_ns();
    int fails = 0;
    for (;;) {
        float v[2][4];
        AcquireSRWLockShared(&values_lock);
        memcpy(v, values, sizeof v);
        ReleaseSRWLockShared(&values_lock);
        const int percent = hos_sdl_rumble_percent();   /* (the menu's Rumble strength) */
        float level[2][2], hz[2][2];
        for (int g = 0; g < 2; g++) {
            triton_levels(hd_rumble_felt_amp(v[g][0]), hd_rumble_felt_amp(v[g][2]), percent, level[g]);
            hz[g][0] = v[g][1] > 20.0f && v[g][1] < 1300.0f ? v[g][1] : 160.0f;
            hz[g][1] = v[g][3] > 20.0f && v[g][3] < 1300.0f ? v[g][3] : 320.0f;
        }
        for (int i = 0; i < TRITON_SAMPLES; i++) {
            for (int g = 0; g < 2; g++) {
                float s = 0.0f;
                for (int b = 0; b < 2; b++) {
                    amp[g][b] += (level[g][b] * triton_grip_gain(hz[g][b]) - amp[g][b]) * glide;
                    phase[g][b] = fmodf(phase[g][b] + two_pi * hz[g][b] / (float)rate, two_pi);
                    s += amp[g][b] * sinf(phase[g][b]);
                }
                (g ? right : left)[i] = s;
            }
        }
        triton_pcm_report(left, right, report);
        fails = write_report(dev, report) ? 0 : fails + 1;
        if (fails >= 100) break;   /* (unplugged, out of range or switched off) */

        const long long now = hos_now_ns();
        for (int side = 0; trackpads && side < 2; side++) {
            const float a = level[side][1], f = triton_avoid_rings(hz[side][1]);
            if (!triton_tone_due(a, f, tone[side].on, tone[side].amp, tone[side].hz, now, tone[side].ns)) continue;
            triton_tone_report(side, a, f, report);
            if (write_report(dev, report)) tone[side].on = a >= 0.02f, tone[side].amp = a, tone[side].hz = f, tone[side].ns = now;
        }

        next += period;   /* (the controller plays samples as they arrive: the reports keep the link's rate) */
        while (next - hos_now_ns() > 1200000LL) Sleep(1);   /* (hos_main.c: 1 ms timer resolution) */
        while (hos_now_ns() < next) Sleep(0);
        if (hos_now_ns() - next > 100000000LL) next = hos_now_ns();   /* (stalled over 100 ms: no burst) */
    }
    InterlockedExchange(&playing, 0);
    device = NULL;
    hos_log("hid: Steam Controller (2026) HD rumble stopped (the controller went away): looking for it again");
}

static DWORD WINAPI triton_main(void *arg) {
    (void)arg;
    hos_host_thread_register_self("Steam Controller HD rumble");
    for (;;) {
        int link = TRITON_NONE;
        void *dev = find_controller(&link);
        if (dev) {
            play(dev, link);
            hos_sdl_hid_close(dev);
        }
        Sleep(2000);
    }
}

/* From pad_sdl.c once SDL's controllers are up */
void hos_triton_start(void) {
    static volatile LONG started;
    if (InterlockedExchange(&started, 1)) return;
    const char *off = getenv("HOS_TRITON_RUMBLE");
    if (off && atoi(off) == 0) return;
    if (!hos_sdl_hid_available()) {
        hos_log("hid: this SDL has no HID calls: a Steam Controller (2026) gets the motor rumble");
        return;
    }
    atexit(at_exit);
    CreateThread(NULL, 0, triton_main, NULL, 0, NULL);
}
