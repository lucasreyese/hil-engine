#!/usr/bin/env python3
"""
Engine plant simulator -- the HIL rig's "engine".

Runs the mean-value engine model in real time, publishes sensor frames over CAN,
and applies the actuator commands it receives back from the ECM (the DUT). With
no ECM on the bus it falls back to safe default commands, so a missing or
crashed controller is obvious.

Examples
--------
  # Hardware-free loop on a virtual bus (see tools/vcan_up.sh):
  python3 main.py --transport vcan --channel vcan0 --profile drive

  # Real CANable (candleLight/gs_usb) brought up as can0 at 500k:
  python3 main.py --transport socketcan --channel can0

  # CANable in slcan mode:
  python3 main.py --transport slcan --channel /dev/ttyACM0
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import can  # noqa: E402  (python-can)

from canbus import DEFAULT_BITRATE, open_bus  # noqa: E402
from codec import DEFAULT_DBC, Codec  # noqa: E402
from driver import Driver  # noqa: E402
from engine import constants as k  # noqa: E402
from engine.physics import EngineModel  # noqa: E402
from engine.state import ControlInputs  # noqa: E402

PHYS_DT = 0.001            # s, physics integration step (1 kHz)
TX_PERIOD = {              # s, per-message transmit cycle (matches can_spec.md)
    "ENG_SPEED": 0.010,
    "ENG_AIR": 0.020,
    "ENG_THERMAL": 0.100,
    "ENG_COMBUSTION": 0.010,
}
ACTUATOR_TIMEOUT = 0.200   # s, revert to safe defaults if ECM goes silent
CRANK_TIMEOUT = 5.0        # s, give up cranking after this if it never catches
PRINT_PERIOD = 0.2         # s, telemetry print cadence


def parse_args():
    p = argparse.ArgumentParser(description="HIL engine plant simulator")
    p.add_argument("--transport", default="vcan",
                   choices=["socketcan", "vcan", "slcan", "virtual"],
                   help="CAN transport (default: vcan)")
    p.add_argument("--channel", default=None,
                   help="bus channel (default: can0/vcan0/ttyACM0 per transport)")
    p.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                   help="bitrate for slcan (SocketCAN is set via ip link)")
    p.add_argument("--dbc", default=DEFAULT_DBC, help="path to engine.dbc")
    p.add_argument("--profile", default="drive",
                   choices=["idle", "drive", "wot", "blip"],
                   help="throttle/load test profile")
    p.add_argument("--duration", type=float, default=None,
                   help="stop after N seconds (default: profile-dependent, 0=forever)")
    p.add_argument("--ambient", type=float, default=k.T_AMB_C,
                   help="ambient/start temperature in degC")
    p.add_argument("--seed", type=int, default=1, help="RNG seed for sensor noise")
    p.add_argument("--no-realtime", action="store_true",
                   help="run as fast as possible instead of wall-clock paced")
    p.add_argument("--quiet", action="store_true", help="suppress telemetry print")
    return p.parse_args()


class StatusView:
    """Latest decoded ECM_STATUS, for telemetry only."""
    def __init__(self):
        self.state = "?"
        self.flags = 0

    def update(self, decoded: dict):
        self.state = str(decoded.get("EcmState", "?"))
        self.flags = int(decoded.get("StatusFlags", 0))


def main():
    args = parse_args()
    codec = Codec(args.dbc)
    model = EngineModel(ambient_c=args.ambient, seed=args.seed)
    driver = Driver(args.profile)
    ctrl = ControlInputs()
    status = StatusView()

    if args.duration is None:
        duration = driver.duration_hint if args.profile != "idle" else 0.0
    else:
        duration = args.duration

    bus = open_bus(args.transport, args.channel, args.bitrate)
    chan = args.channel or ("vcan0" if args.transport == "vcan" else "can0")
    print(f"[sim] {args.transport}:{chan}  profile={args.profile}  "
          f"dbc={os.path.basename(args.dbc)}  "
          f"duration={'forever' if duration == 0 else f'{duration:.0f}s'}")
    print(f"[sim] engine: {k.N_CYL}-cyl {k.DISPLACEMENT_L:.1f}L NA, "
          f"redline {k.REDLINE_RPM:.0f}, idle ~{k.IDLE_RPM_NOMINAL:.0f} rpm")

    last_tx = {name: 0.0 for name in TX_PERIOD}
    last_actuator_rx = -ACTUATOR_TIMEOUT
    last_print = 0.0
    has_started = False
    sim_t = 0.0
    wall_start = time.perf_counter()

    try:
        while True:
            # --- starter logic: crank until it catches, then release ----
            rpm = model.rpm
            starter_on = (not has_started) and rpm < k.RUN_RPM_THRESHOLD and sim_t < CRANK_TIMEOUT
            if rpm > k.RUN_RPM_THRESHOLD:
                has_started = True

            # --- advance the plant --------------------------------------
            throttle = driver.throttle_pct(sim_t)
            load = driver.load_nm(rpm)
            state = model.step(throttle, ctrl, PHYS_DT, starter_on=starter_on, load_nm=load)

            # --- failsafe on a silent ECM -------------------------------
            if sim_t - last_actuator_rx > ACTUATOR_TIMEOUT:
                ctrl = ControlInputs()  # safe defaults

            # --- transmit scheduled sensor frames -----------------------
            for name, period in TX_PERIOD.items():
                if sim_t - last_tx[name] >= period:
                    last_tx[name] = sim_t
                    arb_id, data = codec.encode_sensor(name, state)
                    try:
                        bus.send(can.Message(arbitration_id=arb_id, data=data,
                                             is_extended_id=False))
                    except can.CanError as e:
                        print(f"[sim] TX error on {name}: {e}", file=sys.stderr)

            # --- drain received actuator/status frames ------------------
            while True:
                rx = bus.recv(0.0)
                if rx is None:
                    break
                if codec.is_actuator_frame(rx.arbitration_id):
                    ctrl = codec.decode_actuators(rx.data)
                    last_actuator_rx = sim_t
                elif rx.arbitration_id == codec.status_id:
                    status.update(codec.decode_status(rx.data))

            # --- telemetry ----------------------------------------------
            if not args.quiet and sim_t - last_print >= PRINT_PERIOD:
                last_print = sim_t
                print(
                    f"t={sim_t:6.2f} rpm={state.rpm_filtered:6.0f} "
                    f"thr={state.tps_pct:5.1f}% MAP={model.map_pa/1000:5.1f}kPa "
                    f"MAF={state.maf_g_s:6.2f} lam={state.lambda_:5.3f} "
                    f"ect={state.ect_c:4.1f} knock={state.knock_intensity:4.2f} | "
                    f"ECM[{status.state}] adv={ctrl.ignition_advance_deg:5.1f} "
                    f"pw={ctrl.injector_pw_ms:4.2f} trim={ctrl.fuel_trim_pct:+5.1f}%"
                )

            # --- advance time / pace to wall clock ----------------------
            sim_t += PHYS_DT
            if duration and sim_t >= duration:
                break
            if not args.no_realtime:
                ahead = (wall_start + sim_t) - time.perf_counter()
                if ahead > 0:
                    time.sleep(ahead)
    except KeyboardInterrupt:
        print("\n[sim] stopped")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
