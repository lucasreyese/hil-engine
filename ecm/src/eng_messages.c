/*
 * eng_messages.c -- pack/unpack mirroring docs/engine.dbc (little-endian).
 *
 * raw = (physical - offset) / scale, transmitted LSB-first. Encoders clamp to
 * the signal's documented range so an out-of-range command can never corrupt a
 * neighboring signal. These layouts are verified byte-for-byte against cantools
 * in tests/test_byte_compat.py.
 */
#include <math.h>
#include "eng_messages.h"

/* ---- little-endian field helpers ------------------------------------- */
static inline void put_u16(uint8_t *d, int off, uint16_t v) {
    d[off]     = (uint8_t)(v & 0xFFu);
    d[off + 1] = (uint8_t)((v >> 8) & 0xFFu);
}
static inline uint16_t get_u16(const uint8_t *d, int off) {
    return (uint16_t)(d[off] | ((uint16_t)d[off + 1] << 8));
}
static inline int16_t get_s16(const uint8_t *d, int off) {
    return (int16_t)get_u16(d, off);
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Scale + clamp a physical value to an unsigned raw of `bits` width. */
static uint16_t enc_u(float phys, float scale, float lo, float hi, int bits) {
    float r = lroundf(clampf(phys, lo, hi) / scale);
    float rmax = (float)((1u << bits) - 1u);
    if (r < 0.0f) r = 0.0f;
    if (r > rmax) r = rmax;
    return (uint16_t)r;
}
/* Scale + clamp to a signed 16-bit raw. */
static int16_t enc_s16(float phys, float scale, float lo, float hi) {
    float r = lroundf(clampf(phys, lo, hi) / scale);
    if (r > 32767.0f) r = 32767.0f;
    if (r < -32768.0f) r = -32768.0f;
    return (int16_t)r;
}

/* ---- decode: engine -> ECM ------------------------------------------- */
bool eng_msg_decode(uint32_t id, const uint8_t *d, uint8_t dlc, ecm_inputs_t *in) {
    switch (id) {
    case ID_ENG_SPEED:
        if (dlc < 6) return false;
        in->rpm             = get_u16(d, 0) * 0.25f;
        in->rpm_instant     = get_u16(d, 2) * 0.25f;
        in->crank_angle_deg = get_u16(d, 4) * 0.02f;
        return true;
    case ID_ENG_AIR:
        if (dlc < 6) return false;
        in->maf_g_s = get_u16(d, 0) * 0.01f;
        in->tps_pct = get_u16(d, 2) * 0.01f;
        in->iat_c   = get_s16(d, 4) * 0.1f;
        return true;
    case ID_ENG_THERMAL:
        if (dlc < 2) return false;
        in->ect_c = get_s16(d, 0) * 0.1f;
        return true;
    case ID_ENG_COMBUSTION:
        if (dlc < 4) return false;
        in->lambda = get_u16(d, 0) * 0.001f;
        in->knock  = get_u16(d, 2) * 0.0001f;
        return true;
    default:
        return false;
    }
}

/* ---- encode: ECM -> engine ------------------------------------------- */
uint8_t eng_msg_encode_actuators(const ecm_outputs_t *o, uint8_t d[8]) {
    put_u16(d, 0, (uint16_t)enc_s16(o->ignition_advance_deg, 0.1f, -30.0f, 50.0f));
    put_u16(d, 2, enc_u(o->injector_pw_ms, 0.001f, 0.0f, 65.535f, 16));
    put_u16(d, 4, (uint16_t)enc_s16(o->fuel_trim_pct, 0.01f, -25.0f, 25.0f));
    d[6] = (uint8_t)enc_u(o->cold_start_enrich_pct, 0.5f, 0.0f, 100.0f, 8);
    d[7] = (uint8_t)enc_u(o->idle_target_rpm, 10.0f, 0.0f, 2550.0f, 8);
    return 8;
}

uint8_t eng_msg_encode_status(const ecm_outputs_t *o, uint8_t d[8]) {
    d[0] = o->state;
    d[1] = o->flags;
    put_u16(d, 2, (uint16_t)enc_s16(o->knock_retard_deg, 0.1f, 0.0f, 25.0f));
    put_u16(d, 4, enc_u(o->target_lambda, 0.001f, 0.0f, 2.0f, 16));
    return 6;
}
