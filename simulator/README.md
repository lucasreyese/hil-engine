# Engine Simulator (the plant)

The simulator is the **engine** half of the HIL rig. It runs a real-time
mean-value engine model, publishes the sensor frames an ECM expects over CAN,
and applies the actuator commands it receives back. It is the *plant*; the ECM
is the *device under test*.

```
 throttle/load  ┌─────────────────────────┐   ENG_* sensor frames   ┌─────────┐
 (driver/dyno)─▶│   engine model (physics) │ ───────────────────────▶│   ECM   │
                │  air·fuel·spark·crank    │◀─────────────────────── │  (DUT)  │
                └─────────────────────────┘   ECM_ACTUATORS frame    └─────────┘
```

## The model

A mean-value, crank-angle-aware model (`engine/physics.py`), integrated at 1 kHz:

| Subsystem | What it does |
|-----------|--------------|
| **Air** | Throttle modeled as a compressible orifice filling an intake-manifold plenum; cylinders empty it via volumetric efficiency. Gives realistic MAP/MAF and tip-in lag. |
| **Fuel** | Injector pulse width → fuel, with a wall-film (X-τ) lag, so transient λ excursions are real. |
| **Combustion** | Indicated torque from burned fuel, scaled by spark-vs-MBT efficiency and an air/fuel-ratio efficiency; λ reported through a transport-delayed, lagged wideband O2 sensor. |
| **Rotation** | Net torque (indicated − friction − pumping − accessories − external load + starter) integrated through crank inertia. |
| **Thermal** | First-order coolant warmup and intake-air soak. |
| **Knock** | Intensity grows when spark exceeds a load/rpm/temperature borderline limit; suppressed by rich mixture. |
| **Crank** | `crank_signal.py` adds torsional speed ripple at the firing order (2nd-order for an inline-4), so instantaneous rpm is not a clean mean. |

Defaults model a **2.0 L NA inline-4** (`engine/constants.py`); the calibration
tables (VE, friction, MBT, knock limit) live in `engine/tables.py`. Change those
to model a different engine — nothing downstream hard-codes them.

`engine/state.py` defines `EngineState` (the sensor snapshot) and `ControlInputs`
(the ECM commands), and is the original spec the rest of the model fills in.

## Files

| File | Role |
|------|------|
| `engine/constants.py` | Geometry, thermo, injector, inertia, firing constants |
| `engine/tables.py` | VE / friction / MBT / knock-limit lookup tables + interpolation |
| `engine/physics.py` | `EngineModel.step()` — the integrator |
| `engine/crank_signal.py` | Crank angle + torsional ripple |
| `engine/state.py` | `EngineState`, `ControlInputs` dataclasses |
| `codec.py` | DBC-driven encode/decode (cantools) — see [`docs/engine.dbc`](../docs/engine.dbc) |
| `canbus.py` | Pluggable transport: socketcan / vcan / slcan / virtual |
| `driver.py` | Throttle + road-load test profiles |
| `main.py` | Real-time loop: step physics, TX sensors, RX actuators, telemetry |

## Setup

```bash
pip install -r requirements.txt      # python-can + cantools
# (fresh Debian/Ubuntu may need: sudo apt install python3-pip)
```

## Running

```bash
# Hardware-free, against a virtual bus (bring it up first: tools/vcan_up.sh)
python3 main.py --transport vcan --channel vcan0 --profile drive

# Real CANable in candleLight/gs_usb mode (shows up as can0):
python3 main.py --transport socketcan --channel can0

# CANable in slcan mode:
python3 main.py --transport slcan --channel /dev/ttyACM0
```

Key flags: `--profile {idle,drive,wot,blip}`, `--duration SECONDS`,
`--ambient DEGC` (try a cold start with `--ambient -10`), `--no-realtime`
(run flat-out for batch experiments), `--seed`. Run `--help` for the rest.

Without an ECM on the bus the engine just cranks and then runs on the safe
default commands — which are too lean to fire, so it sits at cranking speed.
That is expected: starting and running it is the ECM's job.

Telemetry line:

```
t=  9.40 rpm=  2470 thr= 30.0% MAP= 49.1kPa MAF= 18.31 lam=1.001 ect=88.6 knock=0.03 | ECM[RUNNING] adv= 28.4 pw=3.05 trim= +1.2%
```

See [`../docs/can_spec.md`](../docs/can_spec.md) for the bus contract.
