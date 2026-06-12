/*
 * main_linux.c -- run the ECM as a Linux process over SocketCAN.
 *
 * This is the host harness around the portable core in ecm.c. It does the three
 * things the STM32 firmware's main loop will also do: receive the ENG_* sensor
 * frames, run ecm_step() on a fixed 10 ms tick, and transmit ECM_ACTUATORS
 * (every tick) + ECM_STATUS (every 100 ms). On the STM32 the tick comes from a
 * hardware timer instead of clock_nanosleep, and CAN I/O goes through the STM32
 * can_iface backend -- the ecm_step() call in the middle is unchanged.
 *
 * Usage:  ecm_host [channel]      (default channel: vcan0)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>

#include "ecm.h"
#include "can_iface.h"
#include "eng_messages.h"

#define TICK_NS       (10 * 1000 * 1000L)  /* 10 ms control period */
#define STATUS_DECIM  10                   /* ECM_STATUS every 100 ms */
#define PRINT_DECIM   50                   /* telemetry every 500 ms */

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static void timespec_add_ns(struct timespec *t, long ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) {
        t->tv_nsec -= 1000000000L;
        t->tv_sec += 1;
    }
}

static void flags_str(uint8_t f, char out[6]) {
    out[0] = (f & ECM_FLAG_CLOSED_LOOP)  ? 'C' : '-';
    out[1] = (f & ECM_FLAG_KNOCK_RETARD) ? 'K' : '-';
    out[2] = (f & ECM_FLAG_COLD_START)   ? 'c' : '-';
    out[3] = (f & ECM_FLAG_FUEL_CUT)     ? 'X' : '-';
    out[4] = (f & ECM_FLAG_IDLE_CONTROL) ? 'I' : '-';
    out[5] = '\0';
}

int main(int argc, char **argv) {
    const char *channel = (argc > 1) ? argv[1] : "vcan0";

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    can_iface_t can;
    if (!can_open(&can, channel, 500000)) {
        fprintf(stderr, "ecm: could not open CAN on '%s'.\n"
                        "  vcan:  sudo ip link add dev %s type vcan && sudo ip link set up %s\n"
                        "  CANable: sudo ip link set %s up type can bitrate 500000\n",
                channel, channel, channel, channel);
        return 1;
    }

    ecm_t ecm;
    ecm_config_t cfg;
    ecm_default_config(&cfg, (float)TICK_NS / 1e9f);
    ecm_init(&ecm, &cfg);

    ecm_inputs_t in;
    memset(&in, 0, sizeof in);
    in.lambda = 1.0f;
    ecm_outputs_t out;

    printf("[ecm] DUT on %s @ 500k | %d-cyl, redline %.0f, control %.0f Hz\n",
           channel, cfg.n_cyl, cfg.redline_rpm, 1e9 / TICK_NS);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    unsigned long tick = 0;
    can_frame_t f;

    while (!g_stop) {
        /* 1. drain all pending sensor frames into the input snapshot */
        while (can_recv(&can, &f)) {
            eng_msg_decode(f.id, f.data, f.dlc, &in);
        }

        /* 2. run the controller */
        ecm_step(&ecm, &in, &out);

        /* 3. transmit actuator command every tick */
        can_frame_t tx = {.id = ID_ECM_ACTUATORS};
        tx.dlc = eng_msg_encode_actuators(&out, tx.data);
        if (!can_send(&can, &tx)) {
            fprintf(stderr, "[ecm] TX actuators failed\n");
        }

        /* 4. transmit status periodically */
        if (tick % STATUS_DECIM == 0) {
            can_frame_t st = {.id = ID_ECM_STATUS};
            st.dlc = eng_msg_encode_status(&out, st.data);
            can_send(&can, &st);
        }

        /* 5. telemetry */
        if (tick % PRINT_DECIM == 0) {
            char fl[6];
            flags_str(out.flags, fl);
            printf("[ecm] %-8s rpm=%6.0f load.lam=%4.2f tgt=%4.2f | "
                   "pw=%5.2fms adv=%5.1f ret=%4.1f trim=%+5.1f%% [%s]\n",
                   ecm_state_name((ecm_state_t)out.state), in.rpm, in.lambda,
                   out.target_lambda, out.injector_pw_ms, out.ignition_advance_deg,
                   out.knock_retard_deg, out.fuel_trim_pct, fl);
            fflush(stdout);
        }

        /* 6. wait for the next 10 ms boundary */
        timespec_add_ns(&next, TICK_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        tick++;
    }

    can_close(&can);
    printf("\n[ecm] stopped\n");
    return 0;
}
