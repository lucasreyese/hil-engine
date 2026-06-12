"""
Mean-value engine plant model.

Couples four subsystems and integrates them with a fixed-step explicit Euler
loop (call `step(dt)` at a small dt, e.g. 1 ms):

  1. Air path     -- throttle orifice flow filling an intake-manifold plenum,
                     emptied by the cylinders' volumetric pumping.
  2. Fuel path    -- injector pulse width -> fuel, with wall-film (X-tau) lag.
  3. Combustion   -- fuel + air -> indicated torque, scaled by spark-vs-MBT and
                     air/fuel-ratio efficiency; lambda reported through a lagged,
                     transport-delayed O2 sensor.
  4. Rotation     -- net torque integrated through crank inertia; cranking,
                     friction, pumping and accessory loads.

Plus thermal warmup, a knock model, and the torsional crank model. The output
is an `EngineState`, exactly the sensor picture the ECM receives over CAN.
"""

import math
import random
from collections import deque

from . import constants as k
from . import tables as t
from .crank_signal import CrankModel
from .state import ControlInputs, EngineState


def _throttle_flow_fn(pressure_ratio: float) -> float:
    """Compressible orifice flow function for P_down/P_up (0,1]."""
    pr = min(max(pressure_ratio, 1.0e-3), 1.0)
    pr_crit = (2.0 / (k.GAMMA + 1.0)) ** (k.GAMMA / (k.GAMMA - 1.0))
    if pr <= pr_crit:  # choked
        return math.sqrt(k.GAMMA) * (2.0 / (k.GAMMA + 1.0)) ** (
            (k.GAMMA + 1.0) / (2.0 * (k.GAMMA - 1.0))
        )
    return math.sqrt(
        (2.0 * k.GAMMA / (k.GAMMA - 1.0))
        * (pr ** (2.0 / k.GAMMA) - pr ** ((k.GAMMA + 1.0) / k.GAMMA))
    )


def _throttle_area_m2(tps_pct: float) -> float:
    """Effective throttle flow area for a throttle command, incl. idle bypass."""
    frac = min(max(tps_pct, 0.0), 100.0) / 100.0
    return k.THROTTLE_AREA_MIN_M2 + frac * (k.THROTTLE_AREA_WOT_M2 - k.THROTTLE_AREA_MIN_M2)


def _spark_efficiency(advance_deg: float, mbt_deg: float) -> float:
    """Torque ratio vs MBT spark: 1.0 at MBT, parabolic falloff either side."""
    delta = advance_deg - mbt_deg
    eff = 1.0 - (delta / 40.0) ** 2
    return max(0.4, min(1.0, eff))


def _combustion_efficiency(lam: float) -> float:
    """Air/fuel-ratio torque efficiency, peaking just rich of stoich."""
    peak = 0.95
    if lam >= peak:
        span = max(k.LAMBDA_LEAN_LIMIT - peak, 1e-3)
        eff = 1.0 - (min(lam, k.LAMBDA_LEAN_LIMIT) - peak) ** 2 / span ** 2
    else:
        span = max(peak - k.LAMBDA_RICH_LIMIT, 1e-3)
        eff = 1.0 - 0.6 * (peak - max(lam, k.LAMBDA_RICH_LIMIT)) ** 2 / span ** 2
    return max(0.0, min(1.0, eff))


class EngineModel:
    """Stateful engine simulation. One instance per simulated engine."""

    def __init__(self, ambient_c: float = k.T_AMB_C, seed: int | None = None):
        self.ambient_c = ambient_c
        self.rng = random.Random(seed)

        # Integrator state
        self.omega = 0.0                      # rad/s, mean crank speed
        self.map_pa = k.P_AMB_PA              # intake manifold pressure
        self.fuel_film_g = 0.0                # wall-film fuel mass
        self.ect_c = ambient_c
        self.iat_c = ambient_c
        self.maf_filt_g_s = 0.0
        self.knock_meas = 0.0

        self.crank = CrankModel()
        # O2 transport delay line, sized lazily to the call dt.
        self._o2_delay = deque()
        self._o2_meas = 1.0

        self.state = EngineState(ect_c=ambient_c, iat_c=ambient_c)

    # -- helpers ----------------------------------------------------------
    @property
    def rpm(self) -> float:
        return self.omega * 60.0 / (2.0 * math.pi)

    def _air_charge_per_cyl_g(self, m_dot_cyl_kg_s: float, rpm: float) -> float:
        """Air mass inducted by one cylinder over one engine cycle (grams)."""
        if rpm < 1.0:
            return 0.0
        cycle_time_s = k.REVS_PER_INTAKE_CYCLE / (rpm / 60.0)
        return m_dot_cyl_kg_s * cycle_time_s / k.N_CYL * 1000.0

    # -- main step --------------------------------------------------------
    def step(self, throttle_pct: float, ctrl: ControlInputs, dt: float,
             starter_on: bool = False, load_nm: float = 0.0) -> EngineState:
        """Advance the engine by dt seconds and return the sensor state.

        load_nm is an external brake load (dyno / driveline) at the crank, on top
        of internal friction, pumping and accessories. With load_nm=0 the engine
        is unloaded, so any throttle that makes net torque will climb to the
        ECM's rev limiter -- realistic for an engine on a stand.
        """
        rpm = self.rpm
        t_man_k = k.c_to_k(self.iat_c)

        # --- 1. Air path: throttle inflow vs cylinder pumping outflow -----
        area = _throttle_area_m2(throttle_pct)
        m_dot_thr = (
            k.THROTTLE_DISCHARGE_COEFF
            * area
            * k.P_AMB_PA
            / math.sqrt(k.R_AIR * k.c_to_k(self.ambient_c))
            * _throttle_flow_fn(self.map_pa / k.P_AMB_PA)
        )
        map_ratio = self.map_pa / k.P_AMB_PA
        ve = t.volumetric_efficiency(rpm, map_ratio)
        rho_man = self.map_pa / (k.R_AIR * t_man_k)
        m_dot_cyl = ve * rho_man * k.DISPLACEMENT_M3 * (rpm / 60.0) / k.REVS_PER_INTAKE_CYCLE

        # Plenum filling: dP/dt = (R*T / V) * (m_in - m_out)
        dmap = (k.R_AIR * t_man_k / k.MANIFOLD_VOLUME_M3) * (m_dot_thr - m_dot_cyl)
        self.map_pa = min(max(self.map_pa + dmap * dt, 2.0e3), k.P_AMB_PA)

        # MAF sensor (pre-throttle): lightly filtered inflow, g/s.
        maf_g_s = m_dot_thr * 1000.0
        a_maf = dt / (0.02 + dt)
        self.maf_filt_g_s += a_maf * (maf_g_s - self.maf_filt_g_s)

        air_charge_g = self._air_charge_per_cyl_g(m_dot_cyl, rpm)

        # --- 2. Fuel path: injector + wall film --------------------------
        fuel_per_inj_g = max(0.0, ctrl.injector_pw_ms - k.INJ_DEADTIME_MS) * k.INJ_FLOW_G_PER_MS
        # Convert per-injection fuel to a port mass flow (g/s).
        if rpm >= 1.0:
            inj_rate_hz = k.N_CYL * (rpm / 60.0) / k.REVS_PER_INTAKE_CYCLE
        else:
            inj_rate_hz = 0.0
        m_dot_inj = fuel_per_inj_g * inj_rate_hz  # g/s commanded into the ports

        # Wall film: a fraction sticks and evaporates with a time constant.
        film_in = k.FUEL_FILM_FRACTION * m_dot_inj
        film_out = self.fuel_film_g / k.FUEL_FILM_TAU_S
        self.fuel_film_g = max(0.0, self.fuel_film_g + (film_in - film_out) * dt)
        m_dot_charge = (1.0 - k.FUEL_FILM_FRACTION) * m_dot_inj + film_out  # g/s into cylinders

        if rpm >= 1.0:
            cycle_time_s = k.REVS_PER_INTAKE_CYCLE / (rpm / 60.0)
            fuel_charge_g = m_dot_charge * cycle_time_s / k.N_CYL
        else:
            fuel_charge_g = 0.0

        # --- 3. Combustion: lambda + indicated torque --------------------
        if fuel_charge_g > 1.0e-6:
            lam = (air_charge_g / fuel_charge_g) / k.AFR_STOICH
        else:
            lam = k.LAMBDA_LEAN_LIMIT  # no fuel -> infinitely lean (misfire)
        lam = min(max(lam, 0.30), 3.0)

        # O2 wideband sensor: pure transport delay then first-order lag.
        delay_steps = max(1, int(round(k.O2_TRANSPORT_DELAY_S / dt)))
        self._o2_delay.append(lam)
        while len(self._o2_delay) > delay_steps:
            self._o2_delay.popleft()
        lam_delayed = self._o2_delay[0]
        a_o2 = dt / (k.O2_TAU_S + dt)
        self._o2_meas += a_o2 * (lam_delayed - self._o2_meas)
        lambda_sensor = self._o2_meas + self.rng.gauss(0.0, 0.004)

        # Fuel actually burnable is limited by available air when rich.
        burnable_per_cyl_g = min(fuel_charge_g, air_charge_g / k.AFR_STOICH)
        mbt = t.mbt_advance_deg(rpm, map_ratio)
        spark_eff = _spark_efficiency(ctrl.ignition_advance_deg, mbt)
        comb_eff = _combustion_efficiency(lam)

        w_ind_cycle = (
            burnable_per_cyl_g * k.N_CYL
            * k.FUEL_LHV_J_PER_G
            * k.ETA_IND_BASE
            * spark_eff
            * comb_eff
        )
        t_ind = w_ind_cycle / (2.0 * k.REVS_PER_INTAKE_CYCLE * math.pi)

        # --- 4. Load torques + rotational dynamics -----------------------
        t_fric = k.torque_from_mep(t.friction_fmep_kpa(rpm) * 1000.0)
        t_pump = k.torque_from_mep(max(0.0, k.P_AMB_PA - self.map_pa))
        t_starter = 0.0
        if starter_on and rpm < k.RUN_RPM_THRESHOLD:
            t_starter = k.STARTER_TORQUE_NM * max(0.0, 1.0 - rpm / k.CRANK_RPM)

        t_net = t_ind - t_fric - t_pump - k.ACCESSORY_LOAD_NM - load_nm + t_starter
        self.omega = max(0.0, self.omega + (t_net / k.ENGINE_INERTIA_KGM2) * dt)
        rpm = self.rpm  # refresh after integration

        # --- 5. Thermal --------------------------------------------------
        if rpm > k.RUN_RPM_THRESHOLD:
            self.ect_c += (k.ECT_OPERATING_C - self.ect_c) * (dt / k.ECT_WARMUP_TAU_S)
        iat_target = self.ambient_c + k.IAT_RISE_C * (1.0 if rpm > k.RUN_RPM_THRESHOLD else 0.0)
        self.iat_c += (iat_target - self.iat_c) * (dt / k.IAT_TAU_S)

        # --- 6. Knock ----------------------------------------------------
        knock_limit = t.knock_limit_advance_deg(rpm, map_ratio)
        knock_limit -= k.KNOCK_ECT_SENS_PER_C * (self.ect_c - k.KNOCK_ECT_REF_C) / k.KNOCK_GAIN_PER_DEG
        over = ctrl.ignition_advance_deg - knock_limit
        rich_suppression = max(0.3, min(1.0, lam))  # rich charge cools, resists knock
        knock_cause = max(0.0, over) * k.KNOCK_GAIN_PER_DEG * rich_suppression
        knock_cause = min(1.0, knock_cause)
        a_knock = dt / (k.KNOCK_DECAY_TAU_S + dt)
        self.knock_meas += a_knock * (knock_cause - self.knock_meas)
        knock_out = max(0.0, self.knock_meas + abs(self.rng.gauss(0.0, k.KNOCK_NOISE_STD)))
        knock_out = min(1.0, knock_out)

        # --- 7. Crank angle + torsional ripple ---------------------------
        angle_deg, ripple_rpm = self.crank.step(rpm, t_ind, dt)
        rpm_instant = max(0.0, rpm + ripple_rpm)

        # --- 8. Publish sensor state -------------------------------------
        s = self.state
        s.rpm = rpm_instant
        s.rpm_filtered = rpm
        s.crank_angle_deg = angle_deg
        s.maf_g_s = self.maf_filt_g_s
        s.tps_pct = min(max(throttle_pct, 0.0), 100.0)
        s.iat_c = self.iat_c
        s.ect_c = self.ect_c
        s.lambda_ = lambda_sensor
        s.knock_intensity = knock_out
        # Simulation internals (not transmitted, useful for logging/plots)
        s.air_mass_g = air_charge_g
        s.fuel_mass_g = fuel_charge_g
        s.torque_nm = t_ind - t_fric - t_pump - k.ACCESSORY_LOAD_NM
        s.torsional_rpm_delta = ripple_rpm
        return s
