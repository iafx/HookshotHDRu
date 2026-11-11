/* HD rumble for the Steam Controller (2026, "Triton"), straight to its HID device (host/pad_triton.c).

   The controller has four LRAs: one under each trackpad and one in each grip. SDL's Valve drivers only give it the plain
   two-motor rumble (report 0x80, the grips' strengths), so the game's HD rumble (nn::hid VibrationValue: {amp_low,
   freq_low, amp_high, freq_high} per side) loses its frequencies. The controller can play them:
   - the grips take an audio stream: output report 0x86 [0x86, op (1 off, 2 on), channel, format] turns it on, then
     0x88 [0x88, count, left grip's samples from byte 2, right grip's from byte 33] carries the samples, which the grip
     plays as a wave, both bands of its side at their own frequency, as a Joy-Con's one actuator plays them. (The
     protocol LiveHaptics found; formats 8, 9 and 11 are mu-law at 8, 4 and 1 kHz, 31 samples a side per report.)
   - each trackpad takes a tone: 0x83 [0x83, side, gain in dB (signed), frequency u16, duration ms u16, LFO Hz u16, LFO
     depth %] plays it for its duration; 0x82 [0x82, side, 0, 0] stops it. The high band plays there too.

   Measured on one controller (Windows 11, its cable, its dongle and Bluetooth):
   - the controller plays the samples as they arrive, and each write waits for it to take the report. The cable took
     about 500 reports a second, the dongle about 250. Mu-law at 8 kHz needs 258 a second (the cable); 16-bit (15
     samples a report) would need 533 and played long and low. The dongle and Bluetooth get 4 kHz (129 a second);
     Bluetooth on Linux 1 kHz (what LiveHaptics found its stack keeps up with);
   - the grips are strongest from 40 to 500 Hz and fall off above: against a DualSense played the same level they felt
     about 15 dB weaker at 800 Hz than below 500, and above that they mostly buzz. A band above 500 Hz is played softer
     here (triton_grip_gain) so it is felt, not heard;
   - the trackpads ring against the shell at 160-170, 260-270 and 500-540 Hz: a tone there moves to the nearest edge
     (the grip still plays the game's own frequency);
   - the dongle shows a HID collection for each of its controller slots; only the one with a controller sends input
     (reports 0x42, 0x45, 0x47), and a report written to an empty one is lost.

   Pure (no state, no SDL): tests/test_hd_rumble_triton.py compiles it. */
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#define TRITON_VENDOR 0x28DE
#define TRITON_REPORT_SIZE 64
#define TRITON_REPORT_COMMAND 0x82
#define TRITON_REPORT_TONE 0x83
#define TRITON_REPORT_PCM_MODE 0x86
#define TRITON_REPORT_PCM_DATA 0x88
#define TRITON_SAMPLES 31                     /* per grip per report, mu-law */
#define TRITON_TONE_GAP_NS 8000000LL         /* the least time between two tones of a side */
#define TRITON_TONE_REFRESH_NS 30000000LL    /* a tone that goes on is sent again this often */
#define TRITON_TONE_MS 60                     /* how long each tone plays: it stops by itself if the game stalls */
#define TRITON_TRACKPAD_DB 6                  /* the trackpads' LRAs are weaker than the grips': +6 dB */

/* How the controller is connected, from its product id: 0 not a Steam Controller (2026) */
enum { TRITON_NONE, TRITON_CABLE, TRITON_BLUETOOTH, TRITON_DONGLE };
static inline int triton_link(uint16_t vendor, uint16_t product) {
    if (vendor != TRITON_VENDOR) return TRITON_NONE;
    if (product == 0x1302) return TRITON_CABLE;
    if (product == 0x1303) return TRITON_BLUETOOTH;
    if (product == 0x1304 || product == 0x1305) return TRITON_DONGLE;
    return TRITON_NONE;
}
/* The audio format for a link: the 0x86 format code and its sample rate */
static inline void triton_format(int link, int windows, int *code, int *rate) {
    if (link == TRITON_CABLE) *code = 8, *rate = 8000;
    else if (link == TRITON_BLUETOOTH && !windows) *code = 11, *rate = 1000;
    else *code = 9, *rate = 4000;
}
/* Whether an input report is the controller's (a dongle slot with a controller in it) */
static inline int triton_is_input(const uint8_t *report, int size) {
    return size > 0 && (report[0] == 0x42 || report[0] == 0x45 || report[0] == 0x47);
}

/* G.711 mu-law, as the controller takes it */
static inline uint8_t triton_mulaw(int sample) {
    const int sign = sample < 0 ? 0x80 : 0;
    int magnitude = (sample < 0 ? -sample : sample);
    if (magnitude > 32635) magnitude = 32635;
    magnitude += 0x84;
    int exponent = 7;
    for (int mask = 0x4000; exponent > 0 && !(magnitude & mask); mask >>= 1) exponent--;
    const int mantissa = (magnitude >> (exponent + 3)) & 0x0F;
    return (uint8_t)~(sign | (exponent << 4) | mantissa);
}

static inline void triton_mode_report(int on, int channel, int format, uint8_t out[TRITON_REPORT_SIZE]) {
    memset(out, 0, TRITON_REPORT_SIZE);
    out[0] = TRITON_REPORT_PCM_MODE, out[1] = (uint8_t)(on ? 2 : 1), out[2] = (uint8_t)channel, out[3] = (uint8_t)format;
}
/* One audio report: TRITON_SAMPLES samples of each grip, -1 to 1 */
static inline void triton_pcm_report(const float left[TRITON_SAMPLES], const float right[TRITON_SAMPLES],
                                     uint8_t out[TRITON_REPORT_SIZE]) {
    memset(out, 0, TRITON_REPORT_SIZE);
    out[0] = TRITON_REPORT_PCM_DATA, out[1] = TRITON_SAMPLES;
    for (int i = 0; i < TRITON_SAMPLES; i++) {
        const float l = left[i] < -1.0f ? -1.0f : left[i] > 1.0f ? 1.0f : left[i];
        const float r = right[i] < -1.0f ? -1.0f : right[i] > 1.0f ? 1.0f : right[i];
        out[2 + i] = triton_mulaw((int)lrintf(l * 32767.0f));
        out[33 + i] = triton_mulaw((int)lrintf(r * 32767.0f));
    }
}
/* A trackpad's tone (side 0 left, 1 right) at amplitude 0 to 1, or its stop when amplitude is under 0.02 */
static inline void triton_tone_report(int side, float amplitude, float hz, uint8_t out[TRITON_REPORT_SIZE]) {
    memset(out, 0, TRITON_REPORT_SIZE);
    if (!(amplitude >= 0.02f)) {   /* (also NaN) */
        out[0] = TRITON_REPORT_COMMAND, out[1] = (uint8_t)side;
        return;
    }
    long db = lrintf(20.0f * log10f(amplitude)) + TRITON_TRACKPAD_DB;
    db = db < -40 ? -40 : db > 6 ? 6 : db;
    const int f = hz < 40.0f ? 40 : hz > 1250.0f ? 1250 : (int)lrintf(hz);
    out[0] = TRITON_REPORT_TONE, out[1] = (uint8_t)side, out[2] = (uint8_t)(int8_t)db;
    out[3] = (uint8_t)(f & 0xFF), out[4] = (uint8_t)(f >> 8);
    out[5] = TRITON_TONE_MS & 0xFF, out[6] = TRITON_TONE_MS >> 8;   /* (no LFO: bytes 7-9 stay 0) */
}
/* A trackpad tone's frequency, moved out of the trackpads' resonances (see above) */
static inline float triton_avoid_rings(float hz) {
    static const float rings[3][2] = {{150.0f, 180.0f}, {250.0f, 280.0f}, {490.0f, 550.0f}};
    for (int i = 0; i < 3; i++)
        if (hz > rings[i][0] && hz < rings[i][1]) return hz - rings[i][0] < rings[i][1] - hz ? rings[i][0] : rings[i][1];
    return hz;
}
/* The grips' gain at a frequency: 1 up to 500 Hz, falling over log frequency to -15 dB at 800 Hz and staying there */
static inline float triton_grip_gain(float hz) {
    if (!(hz > 500.0f)) return 1.0f;
    const float k = hz >= 800.0f ? 1.0f : log2f(hz / 500.0f) / log2f(800.0f / 500.0f);
    return powf(10.0f, -15.0f * k / 20.0f);
}
/* A side's two bands' levels for the grip, from the game's amplitudes as they are felt (hd_rumble.h's
   hd_rumble_felt_amp) and the menu's Rumble strength in percent: both scaled together so they add up to at most 0.9
   (the wave does not clip, and the bands keep their balance) */
static inline void triton_levels(float low_felt, float high_felt, int percent, float out[2]) {
    const float scale = percent > 0 ? (float)percent / 100.0f : 0.0f;
    float l = low_felt * scale, h = high_felt * scale;
    if (!(l >= 0.0f)) l = 0.0f;   /* (also NaN) */
    if (!(h >= 0.0f)) h = 0.0f;
    if (l + h > 0.9f) {
        const float k = 0.9f / (l + h);
        l *= k, h *= k;
    }
    out[0] = l, out[1] = h;
}
/* Whether a trackpad's tone goes out now: amplitude the one wanted, on whether a tone is playing (sent_amp, sent_hz
   what it plays, sent_ns when it was sent). A change goes at once (after TRITON_TONE_GAP_NS); a tone that goes on is
   sent again every TRITON_TONE_REFRESH_NS; a stop is sent once. */
static inline int triton_tone_due(float amplitude, float hz, int on, float sent_amp, float sent_hz, long long now,
                                  long long sent_ns) {
    const int want = amplitude >= 0.02f;
    if (!want) return on;
    if (on && now - sent_ns < TRITON_TONE_GAP_NS) return 0;
    if (!on) return 1;
    const float da = amplitude - sent_amp, df = hz - sent_hz;
    if ((da < 0 ? -da : da) > 0.05f || (df < 0 ? -df : df) > sent_hz / 32.0f) return 1;
    return now - sent_ns >= TRITON_TONE_REFRESH_NS;
}
