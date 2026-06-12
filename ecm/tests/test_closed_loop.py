"""
Closed-loop SIL co-simulation: the real compiled C ECM (build/libecm.so) driving
the Python engine plant, with every sensor and actuator value passing through the
actual CAN encode/decode on both sides. No CAN bus or hardware involved -- this
verifies the controller's behavior, not just that it compiles.

Run:  python3 ecm/tests/test_closed_loop.py   (after `make lib`)
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", ".deps"))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "simulator"))
sys.path.insert(0, _HERE)

from codec import Codec
from engine import constants as k
from engine.physics import EngineModel
from engine.state import ControlInputs
from hil_ffi import ECM, Inputs

PHYS_DT = 0.001
ECM_DT = 0.010
_codec = Codec()

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    flag = "ok  " if cond else "FAIL"
    if cond:
        PASS += 1
    else:
        FAIL += 1
    print(f"  [{flag}] {name}" + (f"  ({detail})" if detail and not cond else ""))


def cosim(throttle_fn, load_fn, secs, ambient=25.0, seed=2):
    """Run the plant + C ECM through the full wire format. Returns sample list."""
    model = EngineModel(ambient_c=ambient, seed=seed)
    ecm = ECM(dt_s=ECM_DT)
    ctrl = ControlInputs()
    inp = Inputs()
    inp.lambda_ = 1.0
    out = None
    has_started = False
    decim = int(round(ECM_DT / PHYS_DT))
    samples = []

    n = int(secs / PHYS_DT)
    for i in range(n):
        t = i * PHYS_DT
        rpm = model.rpm
        starter = (not has_started) and rpm < k.RUN_RPM_THRESHOLD and t < 5.0
        if rpm > k.RUN_RPM_THRESHOLD:
            has_started = True

        s = model.step(throttle_fn(t), ctrl, PHYS_DT,
                       starter_on=starter, load_nm=load_fn(t, rpm))

        if i % decim == 0:
            # engine -> ECM, through CAN frames + C decode
            for name in _codec.sensor_message_names():
                arb, data = _codec.encode_sensor(name, s)
                ecm.decode(arb, bytes(data), into=inp)
            out = ecm.step(inp)
            # ECM -> engine, through C encode + CAN decode
            ctrl = _codec.decode_actuators(ecm.encode_actuators(out))

            if (i // decim) % 20 == 0:  # ~5 Hz sampling
                samples.append({
                    "t": t, "rpm": s.rpm_filtered, "tps": s.tps_pct,
                    "lambda": s.lambda_, "ect": s.ect_c, "knock": s.knock_intensity,
                    "state": out.state, "flags": out.flags,
                    "pw": out.injector_pw_ms, "adv": out.ignition_advance_deg,
                    "retard": out.knock_retard_deg, "trim": out.fuel_trim_pct,
                })
    return samples


def at(samples, t):
    return min(samples, key=lambda x: abs(x["t"] - t))


# ---------------------------------------------------------------------------
def scenario_warm_idle_cruise():
    print("\n[A] warm start: crank -> idle -> 30% cruise -> idle")

    def thr(t):
        if t < 6:   return 0.0
        if t < 12:  return 30.0
        return 0.0

    def load(t, rpm):
        return 3.0 * (rpm / 1000.0) ** 2

    s = cosim(thr, load, 18.0, ambient=85.0)
    started_by = next((x["t"] for x in s if x["rpm"] > k.RUN_RPM_THRESHOLD), None)
    idle = at(s, 5.5)["rpm"]
    cruise = at(s, 11.5)
    back = at(s, 17.5)["rpm"]

    check("engine starts within 3 s", started_by is not None and started_by < 3.0,
          f"started at {started_by}")
    check("idle settles 720-950 rpm", 720 <= idle <= 950, f"idle={idle:.0f}")
    check("closed-loop active at cruise", (cruise["flags"] & 0x01) != 0)
    check("cruise lambda within +/-3%", abs(cruise["lambda"] - 1.0) <= 0.03,
          f"lambda={cruise['lambda']:.3f}")
    check("throttle raises speed", cruise["rpm"] > idle + 200,
          f"cruise={cruise['rpm']:.0f} idle={idle:.0f}")
    check("returns toward idle", back <= idle + 250, f"back={back:.0f}")
    print(f"      idle={idle:.0f} cruise={cruise['rpm']:.0f}rpm/{cruise['lambda']:.3f}lam "
          f"back={back:.0f}")


def scenario_cold_start():
    print("\n[B] cold start at -5 C: enrichment + catches")

    s = cosim(lambda t: 0.0, lambda t, rpm: 3.0 * (rpm / 1000.0) ** 2, 12.0, ambient=-5.0)
    started = next((x["t"] for x in s if x["rpm"] > k.RUN_RPM_THRESHOLD), None)
    cold_flag_early = any((x["flags"] & 0x04) for x in s if x["t"] < 6)
    ect_rise = s[-1]["ect"] - s[0]["ect"]

    check("cold engine starts", started is not None, f"started={started}")
    check("cold-start enrichment flagged", cold_flag_early)
    check("coolant warms up", ect_rise > 2.0, f"dECT={ect_rise:.1f}")
    print(f"      started at {started}s, ECT {s[0]['ect']:.1f}->{s[-1]['ect']:.1f} C")


def scenario_rev_limiter():
    print("\n[C] WOT free-rev: rev limiter fuel-cut")

    s = cosim(lambda t: 0.0 if t < 2 else 100.0, lambda t, rpm: 0.0, 8.0, ambient=85.0)
    peak = max(x["rpm"] for x in s)
    cut_seen = any((x["flags"] & 0x08) for x in s)

    check("rev limiter fuel-cut engages", cut_seen)
    check("speed bounded near redline", peak < k.REDLINE_RPM + 400, f"peak={peak:.0f}")
    print(f"      peak rpm={peak:.0f} (redline {k.REDLINE_RPM:.0f})")


def scenario_knock_control():
    print("\n[D] sustained high load: knock detection + spark retard")

    # A strong quadratic brake load + WOT pins the engine at high load / low-mid
    # rpm (it still pulls away from idle, since quad load is small down low), right
    # where the base spark map sits at the borderline-knock edge.
    s = cosim(lambda t: 0.0 if t < 2 else 100.0,
              lambda t, rpm: 35.0 * (rpm / 1000.0) ** 2, 12.0, ambient=90.0)
    running = [x for x in s if x["t"] > 4]
    max_retard = max(x["retard"] for x in running)
    max_knock = max(x["knock"] for x in running)
    min_rpm = min(x["rpm"] for x in running)

    check("knock retard engages", max_retard > 0.5, f"max_retard={max_retard:.1f}")
    check("knock kept bounded", max_knock < 0.95, f"max_knock={max_knock:.2f}")
    check("engine keeps running under load", min_rpm > 800, f"min_rpm={min_rpm:.0f}")
    print(f"      max knock={max_knock:.2f}, max retard={max_retard:.1f} deg, "
          f"min rpm={min_rpm:.0f}")


if __name__ == "__main__":
    scenario_warm_idle_cruise()
    scenario_cold_start()
    scenario_rev_limiter()
    scenario_knock_control()
    print(f"\n{PASS} checks passed, {FAIL} failed")
    sys.exit(1 if FAIL else 0)
