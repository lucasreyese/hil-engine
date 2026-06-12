# ECM (the device under test)

The Engine Control Module is the **DUT**. It consumes the engine's sensor frames
and commands fuel and spark to keep the engine running, clean, and safe across a
variable throttle. It is written in portable C so the *same control code* runs:

- as a **Linux process** over SocketCAN (`build/ecm_host`) — for HIL/SIL testing
  on a desktop, against the simulator, with or without a CANable; and
- as **STM32 firmware** — the production target from the project README.

The split that makes this work is `include/can_iface.h`: the control core and
message layer only ever touch that transport-agnostic API. The backend is a
link-time choice (`can_socketcan.c` vs `can_stm32.c`). This is the same
application / driver separation AUTOSAR draws, and it is what lets the control
logic be unit-tested on a host.

## Layout

```
include/
  ecm.h            control types, config, ecm_init/ecm_step  (no CAN, no OS)
  can_iface.h      transport-agnostic CAN API (the porting seam)
  eng_messages.h   frame IDs + (de)serialization decls
src/
  ecm.c            the control strategy (fuel, spark, idle, knock, state machine)
  eng_messages.c   pack/unpack mirroring docs/engine.dbc (little-endian)
  can_socketcan.c  Linux backend
  can_stm32.c      STM32 FDCAN/bxCAN backend (compiled only with -DECM_TARGET_STM32)
  main_linux.c     host harness: 10 ms loop, RX sensors, TX actuators/status
tests/             ctypes co-sim + byte-compat (see "Verification" below)
```

## Build & run (host)

```bash
make                       # -> build/ecm_host
./build/ecm_host vcan0     # run against a CAN interface (default vcan0)
```

Easiest end-to-end: `../tools/vcan_up.sh` then `../tools/run_hil.sh vcan0 drive`
launches both the ECM and the simulator on a virtual bus.

```
[ecm] RUNNING  rpm=  2470 load.lam=1.00 tgt=1.00 | pw= 3.05ms adv= 28.4 ret= 0.0 trim= +1.2% [C---I]
```

The flag field is `[C K c X I]` = Closed-loop, Knock-retard, cold-start, fuel-cut,
Idle-control.

## Control strategy (`ecm.c`)

- **State machine**: INIT → CRANKING → WARMUP (open loop) → RUNNING (closed loop),
  plus STALL. Mirrors `ECM_STATUS.EcmState`.
- **Air estimate**: per-cylinder charge from MAF and rpm; a normalized load axis.
- **Fueling**: target λ (rich crank, stoich run, power-enrichment at high load) →
  base fuel → injector pulse width. **Cold-start enrichment** scheduled on coolant
  temp. **Closed-loop λ** PI trim once the O2 sensor is warm and load is moderate.
- **Spark**: base advance map (near MBT, biased to the borderline-knock edge at
  high load). **Knock retard** feedback pulls timing on knock and restores it
  slowly. **Idle-speed governor** uses spark as a fast torque actuator to hold the
  (coolant-scheduled) idle target.
- **Protection**: overrun (decel) fuel cut and a hysteretic **rev-limiter** fuel cut.

All tunables are in `ecm_config_t` (`ecm_default_config()`), so the same code
recalibrates without edits.

## Porting to STM32

1. Create a CubeMX project for your part. Let it generate the clock tree and CAN
   peripheral init — this is where the generator genuinely helps (bit timing for
   500 kbit/s, an 80% sample point). Use **FDCAN in classic frame mode** on
   G0/G4/H7 (e.g. a CANable 2.0 / STM32G431), or **bxCAN** on F1/F4/L4.
2. Add `src/ecm.c` and `src/eng_messages.c` to the build unchanged.
3. Add `src/can_stm32.c` and compile the whole firmware with `-DECM_TARGET_STM32`.
   Point `hfdcan1` at your generated handle; the skeleton there matches the
   `can_iface.h` contract. For bxCAN, swap `HAL_FDCAN_*` for `HAL_CAN_*` — the
   signatures and all control code stay the same.
4. Call `ecm_step()` from a 10 ms timer tick; drain RX in the tick or an IRQ.

Production note: this is a *bench* DUT. A shipping powertrain ECU would run on an
automotive MCU (AURIX/S32/RH850) under AUTOSAR + MISRA C / ISO 26262 — not
STM32Cube HAL. The portable-core structure here mirrors that layering on purpose.

## Verification

No hardware needed — the tests drive the *compiled* C against the Python plant:

```bash
make lib                              # -> build/libecm.so
python3 tests/test_byte_compat.py     # C eng_messages.c  ==  cantools/DBC (byte-for-byte)
python3 tests/test_closed_loop.py     # real C ECM + Python plant, full wire format
```

`test_closed_loop.py` asserts the controller actually works: warm start → idle,
closed-loop λ at cruise, cold-start enrichment, rev-limiter fuel cut, and
knock-retard under sustained high load.
