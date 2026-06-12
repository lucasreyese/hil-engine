"""
Physical and simulation constants for the engine plant model.

Defaults describe a generic 2.0 L naturally-aspirated port-injected inline-4
gasoline engine. Everything here is a calibration knob -- change these to model a
different engine; nothing downstream hard-codes a value that lives in this file.

Units are SI unless the name says otherwise (`_C`, `_KPA`, `_MS`, `_RPM`, ...).
"""

import math

# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
N_CYL = 4
DISPLACEMENT_L = 2.0
DISPLACEMENT_M3 = DISPLACEMENT_L * 1.0e-3
CYL_VOLUME_M3 = DISPLACEMENT_M3 / N_CYL
COMPRESSION_RATIO = 10.5  # informational; combustion model is mean-value

# 4-stroke: each cylinder completes one power cycle every 720 deg (2 rev).
STROKES = 4
CYCLE_DEG = 720.0
# Firing events per crank revolution and their angular spacing.
FIRINGS_PER_REV = N_CYL / 2.0           # = 2.0 for an inline-4
FIRING_INTERVAL_DEG = CYCLE_DEG / N_CYL  # = 180 deg for an inline-4
# The whole displacement is inducted once per 2 revolutions.
REVS_PER_INTAKE_CYCLE = 2.0

# ---------------------------------------------------------------------------
# Working fluids / thermodynamics
# ---------------------------------------------------------------------------
R_AIR = 287.0          # J/(kg*K), specific gas constant for air
GAMMA = 1.4            # ratio of specific heats, for throttle flow
AFR_STOICH = 14.7      # stoichiometric air/fuel ratio, gasoline
FUEL_LHV_J_PER_G = 44000.0   # lower heating value of gasoline (44 MJ/kg)
FUEL_DENSITY_G_PER_CC = 0.745

# Ambient reference
P_AMB_KPA = 101.325
P_AMB_PA = P_AMB_KPA * 1000.0
T_AMB_C = 25.0
KELVIN = 273.15

def c_to_k(celsius: float) -> float:
    return celsius + KELVIN

# ---------------------------------------------------------------------------
# Intake manifold (filling/emptying plenum model)
# ---------------------------------------------------------------------------
MANIFOLD_VOLUME_M3 = 2.5e-3   # plenum + runners, ~2.5 L
# Effective throttle bore flow area at wide-open throttle.
THROTTLE_AREA_WOT_M2 = 9.0e-4
# Idle bypass / mechanical throttle stop: the throttle never fully seals, which
# is what lets the engine idle while the ECM does idle *speed* control via spark.
THROTTLE_AREA_MIN_M2 = 1.42e-5
THROTTLE_DISCHARGE_COEFF = 0.85

# ---------------------------------------------------------------------------
# Injectors (port injection, one pulse per cylinder per cycle)
# ---------------------------------------------------------------------------
INJ_FLOW_CC_PER_MIN = 320.0
INJ_FLOW_G_PER_MS = INJ_FLOW_CC_PER_MIN * FUEL_DENSITY_G_PER_CC / 60000.0
INJ_DEADTIME_MS = 1.0   # opening dead time (battery-voltage dependent on real HW)
# Wall-wetting / fuel-film first-order lag: fraction of injected fuel that lands
# as film, and how fast the film evaporates into the charge.
FUEL_FILM_FRACTION = 0.35
FUEL_FILM_TAU_S = 0.30

# ---------------------------------------------------------------------------
# Rotational dynamics
# ---------------------------------------------------------------------------
ENGINE_INERTIA_KGM2 = 0.18   # crankshaft + flywheel + reflected accessory load
IDLE_RPM_NOMINAL = 800.0
REDLINE_RPM = 6500.0

# Starter / cranking
CRANK_RPM = 250.0            # speed the starter motor sustains
STARTER_TORQUE_NM = 110.0    # peak starter torque (drooped to zero by CRANK_RPM)
RUN_RPM_THRESHOLD = 400.0    # above this the engine is "running" on its own
STALL_RPM = 200.0            # below this (not cranking) the engine has stalled

# External / accessory brake load (alternator, A/C, pumps...) at the crank.
ACCESSORY_LOAD_NM = 4.0

# ---------------------------------------------------------------------------
# Torsional (crankshaft speed ripple) model
# ---------------------------------------------------------------------------
# Each firing event torques the crank, so instantaneous speed ripples at the
# firing order (2nd order for an inline-4). Amplitude scales with combustion
# torque and firing-interval time, and shrinks as inertia/speed rise -- which is
# why idle shake is large and high-rpm ripple is small.
TORSIONAL_RIPPLE_GAIN = 0.5
TORSIONAL_HARMONIC_2 = 1.0   # dominant 2nd-order content
TORSIONAL_HARMONIC_4 = 0.3   # 4th-order overtone

# ---------------------------------------------------------------------------
# Combustion efficiency
# ---------------------------------------------------------------------------
ETA_IND_BASE = 0.37          # indicated thermal efficiency at MBT spark, stoich
# Lean misfire / rich flood limits (lambda). Outside this band torque collapses.
LAMBDA_LEAN_LIMIT = 1.45
LAMBDA_RICH_LIMIT = 0.70
# O2 (wideband) sensor: transport delay + first-order response, plus a light-off
# temperature below which the reading is not trustworthy (ECM stays open-loop).
O2_TRANSPORT_DELAY_S = 0.060
O2_TAU_S = 0.080
O2_LIGHTOFF_ECT_C = 40.0

# ---------------------------------------------------------------------------
# Thermal model
# ---------------------------------------------------------------------------
ECT_OPERATING_C = 90.0       # thermostat-regulated steady-state coolant temp
ECT_WARMUP_TAU_S = 70.0      # first-order warmup time constant when running
IAT_RISE_C = 6.0             # underhood soak above ambient at steady state
IAT_TAU_S = 30.0

# ---------------------------------------------------------------------------
# Knock model
# ---------------------------------------------------------------------------
# Knock intensity grows once spark advance exceeds the (load/rpm) borderline limit
# from tables.py. These shape that growth and the temperature sensitivity.
KNOCK_GAIN_PER_DEG = 0.12        # intensity per deg of advance past the limit
KNOCK_ECT_REF_C = 90.0
KNOCK_ECT_SENS_PER_C = 0.010     # hotter coolant -> more knock-prone
KNOCK_NOISE_STD = 0.015          # background knock-sensor noise (1-sigma)
KNOCK_DECAY_TAU_S = 0.10         # how fast measured intensity follows the cause

# ---------------------------------------------------------------------------
# Derived helpers
# ---------------------------------------------------------------------------
def torque_from_mep(mep_pa: float) -> float:
    """Convert a mean effective pressure (Pa) to crank torque (Nm).

    For a 4-stroke, work per cycle = mep * Vd is delivered over 2 revolutions
    (4*pi radians), so T = mep * Vd / (4*pi).
    """
    return mep_pa * DISPLACEMENT_M3 / (2.0 * REVS_PER_INTAKE_CYCLE * math.pi)
