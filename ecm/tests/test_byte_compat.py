"""
Byte-for-byte cross-check: the C message layer (eng_messages.c) vs the DBC as
interpreted by cantools (what the Python simulator uses). This is what keeps the
two independent implementations of the wire format from drifting apart.

Run:  python3 ecm/tests/test_byte_compat.py   (after `make lib`)
"""

import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", ".deps"))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "simulator"))
sys.path.insert(0, _HERE)

import cantools

from hil_ffi import ECM, Outputs

DBC = os.path.join(_HERE, "..", "..", "docs", "engine.dbc")
db = cantools.database.load_file(DBC)
ecm = ECM()

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
    else:
        FAIL += 1
        print(f"  FAIL {name}: {detail}")


def test_decode_matches_cantools():
    """C eng_msg_decode of a cantools-encoded sensor frame -> same values."""
    rng = random.Random(7)
    fields = {
        "ENG_SPEED": [("EngSpeedFiltered", "rpm", 0, 16000),
                      ("EngSpeedInstant", "rpm_instant", 0, 16000),
                      ("CrankAngle", "crank_angle_deg", 0, 720)],
        "ENG_AIR": [("MassAirFlow", "maf_g_s", 0, 600),
                    ("ThrottlePosition", "tps_pct", 0, 100),
                    ("IntakeAirTemp", "iat_c", -40, 150)],
        "ENG_THERMAL": [("CoolantTemp", "ect_c", -40, 150)],
        "ENG_COMBUSTION": [("Lambda", "lambda_", 0, 2),
                           ("KnockIntensity", "knock", 0, 1)],
    }
    for _ in range(2000):
        for msg_name, sigs in fields.items():
            msg = db.get_message_by_name(msg_name)
            values, expect = {}, {}
            for sig, attr, lo, hi in sigs:
                v = round(rng.uniform(lo, hi), 2)
                values[sig] = v
                expect[attr] = v
            data = msg.encode(values)
            inp = ecm.decode(msg.frame_id, data)
            for sig, attr, lo, hi in sigs:
                got = getattr(inp, attr)
                # tolerance = one scale step (rounding to the signal resolution)
                step = db.get_message_by_name(msg_name).get_signal_by_name(sig).scale
                check(f"decode {msg_name}.{sig}", abs(got - expect[attr]) <= step + 1e-4,
                      f"got {got} want {expect[attr]}")


def test_encode_matches_cantools():
    """C eng_msg_encode_* bytes == cantools encode of the same values."""
    rng = random.Random(11)
    for _ in range(2000):
        adv = round(rng.uniform(-30, 50), 1)
        pw = round(rng.uniform(0, 60), 3)
        trim = round(rng.uniform(-25, 25), 2)
        cold = round(rng.uniform(0, 100), 1)
        idle = float(rng.randrange(0, 2550, 10))
        out = Outputs(ignition_advance_deg=adv, injector_pw_ms=pw, fuel_trim_pct=trim,
                      cold_start_enrich_pct=cold, idle_target_rpm=idle)
        c_bytes = ecm.encode_actuators(out)
        ref = db.get_message_by_name("ECM_ACTUATORS").encode({
            "IgnitionAdvance": adv, "InjectorPulseWidth": pw, "FuelTrim": trim,
            "ColdStartEnrich": cold, "IdleTargetRpm": idle})
        check("encode ECM_ACTUATORS", c_bytes == ref, f"C={c_bytes.hex()} ref={ref.hex()}")

        # status frame
        knock_ret = round(rng.uniform(0, 25), 1)
        tgt = round(rng.uniform(0.7, 1.2), 3)
        state = rng.randrange(0, 5)
        flags = rng.randrange(0, 32)
        sout = Outputs(state=state, flags=flags, knock_retard_deg=knock_ret, target_lambda=tgt)
        c_st = ecm.encode_status(sout)
        ref_st = db.get_message_by_name("ECM_STATUS").encode({
            "EcmState": state, "StatusFlags": flags, "KnockRetard": knock_ret,
            "TargetLambda": tgt})
        check("encode ECM_STATUS", c_st == ref_st, f"C={c_st.hex()} ref={ref_st.hex()}")


if __name__ == "__main__":
    print("byte-compat: C eng_messages.c  <->  cantools/DBC")
    test_decode_matches_cantools()
    test_encode_matches_cantools()
    print(f"\n{PASS} checks passed, {FAIL} failed")
    sys.exit(1 if FAIL else 0)
