"""
Crank angle integration and torsional speed ripple.

A real crank speed sensor never sees a clean mean RPM: every combustion event
gives the crankshaft a kick, so instantaneous speed oscillates at the firing
order. For an inline-4 that's 2nd engine order (two firings per revolution),
with a smaller 4th-order overtone. This module turns mean speed + combustion
torque into (crank angle, instantaneous speed ripple).
"""

import math

from . import constants as k


class CrankModel:
    def __init__(self):
        self.angle_deg = 0.0  # 0..720, engine cycle position

    def step(self, rpm_mean: float, indicated_torque_nm: float, dt: float):
        """Advance crank angle by dt and return (angle_deg, ripple_rpm).

        ripple_rpm is the additive torsional perturbation on top of rpm_mean.
        """
        # Integrate cycle position.
        self.angle_deg = (self.angle_deg + rpm_mean * 360.0 / 60.0 * dt) % k.CYCLE_DEG

        if rpm_mean < 1.0:
            return self.angle_deg, 0.0

        # Time spent on one firing interval -- long at idle, short at speed.
        t_fire = (k.FIRING_INTERVAL_DEG / 360.0) / (rpm_mean / 60.0)
        # Peak angular-speed excursion from a combustion impulse: ~ T*dt / J.
        amp_rad_s = (
            k.TORSIONAL_RIPPLE_GAIN
            * max(indicated_torque_nm, 0.0)
            * t_fire
            / k.ENGINE_INERTIA_KGM2
        )
        amp_rpm = amp_rad_s * 60.0 / (2.0 * math.pi)

        # 2nd order repeats twice per revolution; reference it to revolution angle.
        theta_rev = math.radians(self.angle_deg % 360.0)
        ripple = amp_rpm * (
            k.TORSIONAL_HARMONIC_2 * math.sin(2.0 * theta_rev)
            + k.TORSIONAL_HARMONIC_4 * math.sin(4.0 * theta_rev)
        )
        return self.angle_deg, ripple
