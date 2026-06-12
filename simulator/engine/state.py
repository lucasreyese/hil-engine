from dataclasses import dataclass


@dataclass
class EngineState:
    """
    Complete engine state snapshot transmitted over CAN to ECM.
    All values represent physical sensor outputs.
    """
    # Crank/Speed 
    rpm: float = 0.0 # rev/min, instantaneous
    rpm_filtered: float = 0.0 # rev/min, filtered (1 rev average)
    crank_angle_deg: float = 0.0 # degrees BTDC, 0-720 (4-stroke cycle)

    # Air
    maf_g_s: float = 0.0 # g/s, mass air flow
    tps_pct: float = 0.0 # %, throttle position 0-100 (read from ADC)
    iat_c: float = 25.0 # degrees C, intake air temperature

    # Thermal 
    ect_c: float = 25.0 # degrees C, engine coolant temperature

    # Combustion Quality
    lambda_: float = 1.0 # lambda, O2 sensor output (1.0 = stoich)
    knock_intensity: float = 0.0 # 0.0-1.0, knock sensor output

    # Simulation Internals (not sent over CAN)
    air_mass_g: float = 0.0 # g/cycle, computed air charge
    fuel_mass_g: float = 0.0 # g/cycle, computed fuel mass
    torque_nm: float = 0.0 # Nm, instantaneous torque
    torsional_rpm_delta: float = 0.0 # RPM perturbation from Fourier model


@dataclass
class ControlInputs:
    """
    Control outputs from the STM32 ECM received over CAN.
    Defaults are safe fallback values.
    """
    ignition_advance_deg: float = 10.0 # degrees BTDC
    injector_pw_ms: float = 2.5 # ms, injector pulse width
    fuel_trim_pct: float = 0.0 # %, closed loop lambda trim (-25 to +25)
    cold_start_enrich_pct: float = 0.0 # %, ECT based enrichment
    idle_target_rpm: float = 700.0 # RPM, idle control target
