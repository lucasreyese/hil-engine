# CAN Specification

This document describes the CAN bus contract between the **engine simulator** (the
plant, Python) and the **ECM** (the device under test, C). The machine-readable
source of truth is [`engine.dbc`](engine.dbc); this page is the human-readable
companion. **If the two ever disagree, the DBC wins** — the simulator loads the DBC
directly, and the C ECM hand-packs frames to match it (see
[`ecm/src/can/eng_messages.c`](../ecm/src/can/eng_messages.c)).

## Bus parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Protocol | CAN 2.0A (classic) | Original CANable / bxCAN is classic-only |
| Bitrate | 500 kbit/s | Matches `simulator/can_demo.py` |
| ID type | 11-bit standard | |
| Max payload | 8 bytes | Drives the multi-frame split below |
| Byte order | little-endian (Intel) | All signals |

> **CAN-FD:** not used. It would require a CANable 2.0 (FDCAN/STM32G4). If you move to
> FD later, the larger payload lets `ENG_*` collapse into fewer frames — bump the DBC
> and both backends together.

## Nodes

| Node | Role | Implementation |
|------|------|----------------|
| `ENGINE_SIM` | Plant — produces sensor data, consumes actuator commands | `simulator/` (Python) |
| `ECM` | DUT — consumes sensors, produces actuator commands | `ecm/` (C) |

## Message summary

| ID | Name | Dir | DLC | Cycle | Purpose |
|------|----------------|-----------------|-----|--------|---------|
| 0x100 | `ENG_SPEED` | SIM → ECM | 6 | 10 ms | Crank speed + position |
| 0x110 | `ENG_AIR` | SIM → ECM | 6 | 20 ms | MAF, throttle, intake air temp |
| 0x120 | `ENG_THERMAL` | SIM → ECM | 2 | 100 ms | Coolant temp |
| 0x130 | `ENG_COMBUSTION`| SIM → ECM | 4 | 10 ms | Wideband O2 (lambda) + knock |
| 0x200 | `ECM_ACTUATORS` | ECM → SIM | 8 | 10 ms | Spark, injector PW, trims, idle target |
| 0x210 | `ECM_STATUS` | ECM → SIM | 6 | 100 ms | ECM state machine + flags (telemetry) |

Lower IDs win arbitration, so the 10 ms control-critical frames (`ENG_SPEED`,
`ENG_COMBUSTION`, `ECM_ACTUATORS`) sit below the slower diagnostic frames.

## Signal definitions

Encoding for each signal is `raw = (physical - offset) / scale`, transmitted
little-endian. `start` is the bit position of the LSB.

### 0x100 `ENG_SPEED` (SIM → ECM, 10 ms)
| Signal | Start | Bits | Type | Scale | Offset | Range | Unit |
|--------|-------|------|------|-------|--------|-------|------|
| `EngSpeedFiltered` | 0 | 16 | u | 0.25 | 0 | 0…16383.75 | rpm |
| `EngSpeedInstant` | 16 | 16 | u | 0.25 | 0 | 0…16383.75 | rpm |
| `CrankAngle` | 32 | 16 | u | 0.02 | 0 | 0…720 | deg |

`EngSpeedInstant` carries torsional ripple; it (and `CrankAngle`) alias above idle on
a 10 ms frame and are intended for low-speed/idle diagnostics. Closed-loop control
should use `EngSpeedFiltered`.

### 0x110 `ENG_AIR` (SIM → ECM, 20 ms)
| Signal | Start | Bits | Type | Scale | Offset | Range | Unit |
|--------|-------|------|------|-------|--------|-------|------|
| `MassAirFlow` | 0 | 16 | u | 0.01 | 0 | 0…655.35 | g/s |
| `ThrottlePosition` | 16 | 16 | u | 0.01 | 0 | 0…100 | % |
| `IntakeAirTemp` | 32 | 16 | s | 0.1 | 0 | -40…150 | °C |

### 0x120 `ENG_THERMAL` (SIM → ECM, 100 ms)
| Signal | Start | Bits | Type | Scale | Offset | Range | Unit |
|--------|-------|------|------|-------|--------|-------|------|
| `CoolantTemp` | 0 | 16 | s | 0.1 | 0 | -40…150 | °C |

### 0x130 `ENG_COMBUSTION` (SIM → ECM, 10 ms)
| Signal | Start | Bits | Type | Scale | Offset | Range | Unit |
|--------|-------|------|------|-------|--------|-------|------|
| `Lambda` | 0 | 16 | u | 0.001 | 0 | 0…2 | λ |
| `KnockIntensity` | 16 | 16 | u | 0.0001 | 0 | 0…1 | ratio |

`Lambda` = 1.0 is stoichiometric. `KnockIntensity` is a normalized 0…1 knock-sensor
energy estimate.

### 0x200 `ECM_ACTUATORS` (ECM → SIM, 10 ms)
| Signal | Start | Bits | Type | Scale | Offset | Range | Unit |
|--------|-------|------|------|-------|--------|-------|------|
| `IgnitionAdvance` | 0 | 16 | s | 0.1 | 0 | -30…50 | °BTDC |
| `InjectorPulseWidth` | 16 | 16 | u | 0.001 | 0 | 0…65.535 | ms |
| `FuelTrim` | 32 | 16 | s | 0.01 | 0 | -25…25 | % |
| `ColdStartEnrich` | 48 | 8 | u | 0.5 | 0 | 0…100 | % |
| `IdleTargetRpm` | 56 | 8 | u | 10 | 0 | 0…2550 | rpm |

`InjectorPulseWidth` is the **only physical fueling actuator** — the simulator derives
delivered fuel mass from it alone. `FuelTrim` and `ColdStartEnrich` are ECM telemetry
that have **already been folded into** the pulse width; the sim does not re-apply them
(doing so would double-count). They are on the bus so a HIL operator can see what the
controller is doing.

### 0x210 `ECM_STATUS` (ECM → SIM, 100 ms)
| Signal | Start | Bits | Type | Scale | Offset | Range | Unit |
|--------|-------|------|------|-------|--------|-------|------|
| `EcmState` | 0 | 8 | enum | 1 | 0 | 0…4 | — |
| `StatusFlags` | 8 | 8 | bitfield | 1 | 0 | 0…255 | — |
| `KnockRetard` | 16 | 16 | s | 0.1 | 0 | 0…25 | deg |
| `TargetLambda` | 32 | 16 | u | 0.001 | 0 | 0…2 | λ |

`EcmState`:

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `INIT` | Powered, no sync yet |
| 1 | `CRANKING` | Below run threshold, starter assumed engaged |
| 2 | `WARMUP` | Running, **open-loop** fuel (O2/coolant cold) |
| 3 | `RUNNING` | Running, **closed-loop** lambda |
| 4 | `STALL` | Was running, speed collapsed below stall threshold |

`StatusFlags` (bitfield):

| Bit | Mask | Name | Meaning |
|-----|------|------|---------|
| 0 | 0x01 | `CLOSED_LOOP` | Lambda PI trim active |
| 1 | 0x02 | `KNOCK_RETARD` | Spark being pulled for knock |
| 2 | 0x04 | `COLD_START` | Cold-start enrichment active |
| 3 | 0x08 | `FUEL_CUT` | Overrun or rev-limit fuel cut |
| 4 | 0x10 | `IDLE_CONTROL` | Idle-speed spark authority active |

## Timing & failsafes

- The ECM caches the latest value of each sensor frame and runs its control loop on a
  fixed **10 ms** tick; it does not block waiting on a specific frame.
- The simulator applies the last received `ECM_ACTUATORS` until a new one arrives. If
  no actuator frame has been seen for **200 ms**, the sim reverts to the safe defaults
  in `ControlInputs` (see `simulator/engine/state.py`) so a dead/restarting ECM is
  obvious rather than latching stale commands.
