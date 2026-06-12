/*
 * ecm.c -- portable engine control strategy.
 *
 * One control step (ecm_step) runs, in order:
 *   - air estimate from MAF -> per-cylinder charge and a normalized load axis
 *   - run/crank/stall state machine
 *   - fueling: target lambda -> base fuel, cold-start enrichment, closed-loop
 *     lambda PI trim, overrun + rev-limit fuel cut -> injector pulse width
 *   - spark: base advance map, idle-speed governor (spark authority), knock
 *     retard feedback -> ignition advance
 *
 * No CAN, no OS, no float-formatting: just inputs -> outputs. That keeps it
 * identical on the Linux host and on an STM32.
 */
#include "ecm.h"

/* ---- small clamped interpolators (mirrors simulator/engine/tables.py) ---- */
static float interp1(float x, const float *xs, const float *ys, int n) {
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    int i = 0;
    while (x >= xs[i + 1]) i++;
    float t = (x - xs[i]) / (xs[i + 1] - xs[i]);
    return ys[i] + t * (ys[i + 1] - ys[i]);
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* ---- base spark map: advance (deg BTDC) vs rpm x load -------------------
 * Authored near MBT, biased a degree or two hot at high load so the engine
 * runs at the borderline-knock edge there -- exactly where the knock-retard
 * feedback below is meant to earn its keep. */
#define SPARK_NR 7
#define SPARK_NL 5
static const float SPARK_RPM[SPARK_NR] = {800, 1500, 2500, 3500, 4500, 5500, 6500};
static const float SPARK_LOAD[SPARK_NL] = {0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
static const float SPARK_ADV[SPARK_NR][SPARK_NL] = {
    /* load:  0.2   0.4   0.6   0.8   1.0 */
    {24, 21, 19, 18, 18},  /* 800  */
    {29, 26, 23, 22, 21},  /* 1500 */
    {34, 30, 27, 25, 23},  /* 2500 */
    {38, 34, 31, 28, 26},  /* 3500 */
    {40, 36, 33, 30, 28},  /* 4500 */
    {41, 37, 35, 32, 31},  /* 5500 */
    {42, 38, 36, 33, 32},  /* 6500 */
};

static float spark_base_advance(float rpm, float load) {
    /* bilinear: interp across load within the two bracketing rpm rows */
    float row[SPARK_NR];
    for (int i = 0; i < SPARK_NR; i++) {
        row[i] = interp1(load, SPARK_LOAD, SPARK_ADV[i], SPARK_NL);
    }
    return interp1(rpm, SPARK_RPM, row, SPARK_NR);
}

/* ---- configuration ------------------------------------------------------ */
void ecm_default_config(ecm_config_t *c, float dt_s) {
    c->dt_s = dt_s;

    c->inj_flow_g_per_ms = 320.0f * 0.745f / 60000.0f; /* matches the sim's injector */
    c->inj_deadtime_ms = 1.0f;
    c->afr_stoich = 14.7f;
    c->air_per_cyl_max_g = 0.55f;
    c->n_cyl = 4;

    c->run_rpm = 400.0f;
    c->stall_rpm = 200.0f;
    c->redline_rpm = 6500.0f;
    c->rev_resume_rpm = 6300.0f;
    c->overrun_rpm = 1500.0f;
    c->overrun_tps_pct = 2.0f;

    c->crank_lambda = 0.75f;
    c->power_lambda = 0.88f;
    c->power_enrich_load = 0.80f;

    c->cold_enrich_max_pct = 45.0f;
    c->cold_enrich_full_ect_c = -20.0f;
    c->cold_enrich_zero_ect_c = 60.0f;

    c->cl_lightoff_ect_c = 40.0f;
    c->cl_kp_pct = 60.0f;
    c->cl_ki_pct_s = 90.0f;
    c->cl_trim_limit_pct = 20.0f;

    c->idle_target_warm_rpm = 820.0f;
    c->idle_target_cold_rpm = 1150.0f;
    c->idle_tps_pct = 2.5f;
    c->idle_band_rpm = 260.0f;
    c->idle_base_advance_deg = 10.0f;  /* retarded; governor advances toward MBT */
    c->idle_kp_deg = 0.020f;
    c->idle_ki_deg_s = 0.060f;
    c->idle_authority_deg = 16.0f;

    c->knock_threshold = 0.12f;
    c->knock_retard_step_deg = 0.40f;
    c->knock_recover_deg_s = 2.0f;
    c->knock_retard_max_deg = 12.0f;

    c->advance_min_deg = -10.0f;
    c->advance_max_deg = 45.0f;
}

void ecm_init(ecm_t *e, const ecm_config_t *cfg) {
    e->cfg = *cfg;
    e->state = ECM_STATE_INIT;
    e->has_run = false;
    e->fuel_cut = false;
    e->lambda_trim_pct = 0.0f;
    e->idle_trim_deg = 0.0f;
    e->knock_retard_deg = 0.0f;
}

/* Air mass inducted by one cylinder per engine cycle, from the MAF reading. */
static float air_per_cyl_g(const ecm_config_t *c, float maf_g_s, float rpm) {
    float rpm_eff = rpm < 120.0f ? 120.0f : rpm;  /* guard the 1/rpm */
    /* whole engine inducts displacement once per 2 rev: cycle = 120/rpm s */
    return maf_g_s * (120.0f / rpm_eff) / (float)c->n_cyl;
}

void ecm_step(ecm_t *e, const ecm_inputs_t *in, ecm_outputs_t *out) {
    const ecm_config_t *c = &e->cfg;
    const float rpm = in->rpm;

    float air_cyl = air_per_cyl_g(c, in->maf_g_s, rpm);
    float load = clampf(air_cyl / c->air_per_cyl_max_g, 0.0f, 1.0f);

    /* --- state machine --------------------------------------------------- */
    bool running = rpm >= c->run_rpm;
    if (running) {
        e->has_run = true;
    }
    bool warm = in->ect_c >= c->cl_lightoff_ect_c;

    /* rev-limit fuel cut with hysteresis */
    if (rpm > c->redline_rpm) e->fuel_cut = true;
    else if (rpm < c->rev_resume_rpm) e->fuel_cut = false;

    bool closed_loop = warm && running && load < c->power_enrich_load && !e->fuel_cut;

    if (running) {
        e->state = closed_loop ? ECM_STATE_RUNNING : ECM_STATE_WARMUP;
    } else if (e->has_run && rpm < c->stall_rpm) {
        e->state = ECM_STATE_STALL;
    } else if (rpm > 50.0f) {
        e->state = ECM_STATE_CRANKING;
    } else {
        e->state = e->has_run ? ECM_STATE_STALL : ECM_STATE_INIT;
    }

    /* --- idle speed target (ECT scheduled) ------------------------------- */
    float ect_bp[2] = {20.0f, 75.0f};
    float idle_tgt_bp[2] = {c->idle_target_cold_rpm, c->idle_target_warm_rpm};
    float idle_target = interp1(in->ect_c, ect_bp, idle_tgt_bp, 2);

    /* --- fueling --------------------------------------------------------- */
    float target_lambda;
    if (e->state == ECM_STATE_CRANKING || e->state == ECM_STATE_INIT) {
        target_lambda = c->crank_lambda;
    } else if (load >= c->power_enrich_load) {
        /* blend stoich -> power_lambda as load rises past the threshold */
        float f = clampf((load - c->power_enrich_load) / (1.0f - c->power_enrich_load),
                         0.0f, 1.0f);
        target_lambda = 1.0f + f * (c->power_lambda - 1.0f);
    } else {
        target_lambda = 1.0f;
    }

    /* cold-start enrichment from coolant temp */
    float cold_frac = clampf(
        (c->cold_enrich_zero_ect_c - in->ect_c) /
            (c->cold_enrich_zero_ect_c - c->cold_enrich_full_ect_c),
        0.0f, 1.0f);
    float cold_enrich_pct = cold_frac * c->cold_enrich_max_pct;

    /* closed-loop lambda PI trim (only when authorized) */
    if (closed_loop) {
        float err = in->lambda - target_lambda;  /* >0 = lean -> add fuel */
        e->lambda_trim_pct += c->cl_ki_pct_s * err * c->dt_s;
        e->lambda_trim_pct = clampf(e->lambda_trim_pct,
                                    -c->cl_trim_limit_pct, c->cl_trim_limit_pct);
    } else {
        /* open loop: bleed the integrator back toward zero */
        e->lambda_trim_pct -= e->lambda_trim_pct * clampf(c->dt_s / 0.5f, 0.0f, 1.0f);
    }
    float trim_pct = e->lambda_trim_pct;
    if (closed_loop) {
        trim_pct += c->cl_kp_pct * (in->lambda - target_lambda);
        trim_pct = clampf(trim_pct, -c->cl_trim_limit_pct, c->cl_trim_limit_pct);
    }

    float fuel_g = air_cyl / (c->afr_stoich * target_lambda);
    fuel_g *= (1.0f + cold_enrich_pct / 100.0f);
    fuel_g *= (1.0f + trim_pct / 100.0f);

    float pw = fuel_g / c->inj_flow_g_per_ms + c->inj_deadtime_ms;

    /* fuel cut: rev limit (latched) or decel/overrun (condition) */
    bool overrun = warm && rpm > c->overrun_rpm && in->tps_pct < c->overrun_tps_pct;
    bool cut = e->fuel_cut || overrun;
    if (e->state == ECM_STATE_CRANKING) {
        cut = false;                 /* never cut while trying to start */
        if (pw < 2.0f) pw = 2.0f;    /* ensure a prime pulse */
    }
    if (cut) pw = 0.0f;

    /* --- spark ----------------------------------------------------------- */
    float advance;
    if (e->state == ECM_STATE_CRANKING || e->state == ECM_STATE_INIT) {
        advance = 8.0f;  /* fixed, gentle, for starting */
    } else {
        advance = spark_base_advance(rpm, load);
    }

    /* Idle-speed governor: at closed throttle near idle, replace the base map
     * with a deliberately retarded idle base and let a PI loop advance toward
     * MBT to make torque (and retard to shed it). Spark is a fast idle actuator;
     * base idle air comes from the throttle's mechanical minimum on the plant. */
    bool idle_active = running && !cut && in->tps_pct < c->idle_tps_pct &&
                       rpm < idle_target + c->idle_band_rpm;
    if (idle_active) {
        float err = idle_target - rpm;  /* >0 = too slow -> advance for torque */
        e->idle_trim_deg += c->idle_ki_deg_s * err * c->dt_s;
        e->idle_trim_deg = clampf(e->idle_trim_deg,
                                  -c->idle_authority_deg, c->idle_authority_deg);
        float gov = clampf(e->idle_trim_deg + c->idle_kp_deg * err,
                           -c->idle_authority_deg, c->idle_authority_deg);
        advance = c->idle_base_advance_deg + gov;
    } else {
        e->idle_trim_deg -= e->idle_trim_deg * clampf(c->dt_s / 0.5f, 0.0f, 1.0f);
    }

    /* knock retard feedback: pull timing on knock, restore slowly when clear */
    if (in->knock > c->knock_threshold) {
        e->knock_retard_deg += c->knock_retard_step_deg;
    } else {
        e->knock_retard_deg -= c->knock_recover_deg_s * c->dt_s;
    }
    e->knock_retard_deg = clampf(e->knock_retard_deg, 0.0f, c->knock_retard_max_deg);
    advance -= e->knock_retard_deg;
    advance = clampf(advance, c->advance_min_deg, c->advance_max_deg);

    /* --- publish --------------------------------------------------------- */
    out->ignition_advance_deg = advance;
    out->injector_pw_ms = pw;
    out->fuel_trim_pct = closed_loop ? trim_pct : 0.0f;
    out->cold_start_enrich_pct = cold_enrich_pct;
    out->idle_target_rpm = idle_target;
    out->knock_retard_deg = e->knock_retard_deg;
    out->target_lambda = target_lambda;

    out->state = (uint8_t)e->state;
    uint8_t flags = 0;
    if (closed_loop)                       flags |= ECM_FLAG_CLOSED_LOOP;
    if (e->knock_retard_deg > 0.1f)        flags |= ECM_FLAG_KNOCK_RETARD;
    if (cold_enrich_pct > 1.0f)            flags |= ECM_FLAG_COLD_START;
    if (cut)                               flags |= ECM_FLAG_FUEL_CUT;
    if (idle_active)                       flags |= ECM_FLAG_IDLE_CONTROL;
    out->flags = flags;
}

const char *ecm_state_name(ecm_state_t s) {
    switch (s) {
    case ECM_STATE_INIT:     return "INIT";
    case ECM_STATE_CRANKING: return "CRANKING";
    case ECM_STATE_WARMUP:   return "WARMUP";
    case ECM_STATE_RUNNING:  return "RUNNING";
    case ECM_STATE_STALL:    return "STALL";
    default:                 return "?";
    }
}
