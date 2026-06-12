/*
 * ecm.h -- Engine Control Module: portable control core.
 *
 * This is the hardware-independent strategy. It knows nothing about CAN, about
 * SocketCAN vs STM32 bxCAN, or about the OS. You hand it a snapshot of the
 * sensor inputs and a loop period; it returns the actuator commands. The same
 * object file links into the Linux host build and the STM32 firmware.
 *
 * Mirrors docs/can_spec.md: inputs are the decoded ENG_* signals, outputs are
 * the ECM_ACTUATORS + ECM_STATUS signals.
 */
#ifndef ECM_H
#define ECM_H

#include <stdint.h>
#include <stdbool.h>

/* ECM_STATUS.EcmState enumeration (see docs/can_spec.md). */
typedef enum {
    ECM_STATE_INIT     = 0,
    ECM_STATE_CRANKING = 1,
    ECM_STATE_WARMUP   = 2,  /* running, open-loop fuel */
    ECM_STATE_RUNNING  = 3,  /* running, closed-loop lambda */
    ECM_STATE_STALL    = 4
} ecm_state_t;

/* ECM_STATUS.StatusFlags bitfield. */
#define ECM_FLAG_CLOSED_LOOP   0x01u
#define ECM_FLAG_KNOCK_RETARD  0x02u
#define ECM_FLAG_COLD_START    0x04u
#define ECM_FLAG_FUEL_CUT      0x08u
#define ECM_FLAG_IDLE_CONTROL  0x10u

/* Decoded sensor snapshot (from the ENG_* frames). */
typedef struct {
    float rpm;             /* EngSpeedFiltered, rev/min */
    float rpm_instant;     /* EngSpeedInstant */
    float crank_angle_deg; /* CrankAngle */
    float maf_g_s;         /* MassAirFlow */
    float tps_pct;         /* ThrottlePosition */
    float iat_c;           /* IntakeAirTemp */
    float ect_c;           /* CoolantTemp */
    float lambda;          /* Lambda (wideband O2) */
    float knock;           /* KnockIntensity, 0..1 */
} ecm_inputs_t;

/* Actuator commands + telemetry (the ECM_ACTUATORS and ECM_STATUS frames). */
typedef struct {
    float    ignition_advance_deg;  /* deg BTDC */
    float    injector_pw_ms;        /* ms (the only physical fuel actuator) */
    float    fuel_trim_pct;         /* closed-loop trim, telemetry */
    float    cold_start_enrich_pct; /* telemetry */
    float    idle_target_rpm;
    uint8_t  state;                 /* ecm_state_t */
    uint8_t  flags;                 /* ECM_FLAG_* */
    float    knock_retard_deg;
    float    target_lambda;
} ecm_outputs_t;

/* Tunables. ecm_default_config() fills sensible values for the modeled 2.0L. */
typedef struct {
    float dt_s;                  /* control loop period */

    /* Injectors */
    float inj_flow_g_per_ms;
    float inj_deadtime_ms;
    float afr_stoich;
    float air_per_cyl_max_g;     /* normalizer for the load axis */
    int   n_cyl;

    /* Speed thresholds */
    float run_rpm;               /* above => running */
    float stall_rpm;             /* below (after running) => stalled */
    float redline_rpm;
    float rev_resume_rpm;        /* fuel cut clears below this */
    float overrun_rpm;           /* decel fuel cut above this ... */
    float overrun_tps_pct;       /* ... with throttle closed */

    /* Fuel targets */
    float crank_lambda;          /* rich while cranking */
    float power_lambda;          /* enrichment at high load */
    float power_enrich_load;     /* load fraction where power enrich begins */

    /* Cold-start enrichment (ECT based) */
    float cold_enrich_max_pct;
    float cold_enrich_full_ect_c; /* at/below => full enrichment */
    float cold_enrich_zero_ect_c; /* at/above => none */

    /* Closed-loop lambda PI */
    float cl_lightoff_ect_c;     /* coolant temp to allow closed loop */
    float cl_kp_pct;             /* %% trim per unit lambda error */
    float cl_ki_pct_s;           /* %% trim per unit error per second */
    float cl_trim_limit_pct;

    /* Idle speed governor (spark authority) */
    float idle_target_warm_rpm;
    float idle_target_cold_rpm;
    float idle_tps_pct;          /* below => idle control engaged */
    float idle_band_rpm;         /* governor active within this of target */
    float idle_base_advance_deg; /* retarded base so the governor can add torque */
    float idle_kp_deg;           /* deg spark per rpm error */
    float idle_ki_deg_s;
    float idle_authority_deg;    /* +/- spark limit for the governor */

    /* Knock control */
    float knock_threshold;
    float knock_retard_step_deg; /* applied per loop while knocking */
    float knock_recover_deg_s;   /* advance restored per second when clear */
    float knock_retard_max_deg;

    /* Spark output clamp */
    float advance_min_deg;
    float advance_max_deg;
} ecm_config_t;

/* Persistent controller state. */
typedef struct {
    ecm_config_t cfg;
    ecm_state_t  state;
    bool         has_run;        /* engine has caught at least once */
    bool         fuel_cut;       /* rev-limit fuel cut latched (hysteresis) */
    float        lambda_trim_pct;/* closed-loop integrator */
    float        idle_trim_deg;  /* idle governor integrator */
    float        knock_retard_deg;
} ecm_t;

void ecm_default_config(ecm_config_t *cfg, float dt_s);
void ecm_init(ecm_t *ecm, const ecm_config_t *cfg);

/* Run one control step: read inputs, produce outputs. */
void ecm_step(ecm_t *ecm, const ecm_inputs_t *in, ecm_outputs_t *out);

/* Human-readable state name (for logging). */
const char *ecm_state_name(ecm_state_t s);

#endif /* ECM_H */
