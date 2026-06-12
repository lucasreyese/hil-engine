"""
Test stimulus for the plant: the "driver" (and dyno) side of the HIL rig.

The throttle is an *input to the simulator* (it is the operator pressing the
pedal), reported to the ECM as the TPS sensor. It is deliberately not commanded
by the ECM -- the ECM reacts to it. Profiles below are scripted throttle vs time,
optionally with a quadratic road/dyno load so part-throttle settles at a steady
speed rather than free-revving into the limiter.
"""

# Quadratic road/dyno load: Nm per (krpm)^2. Tuned so wide-ish throttle settles
# near the upper rev range and light throttle cruises mid-range.
ROAD_LOAD_QUAD = 3.0

# (time_s, throttle_pct) setpoints, held until the next entry (step input).
_PROFILES = {
    "idle": ([(0.0, 0.0)], False),
    "drive": (
        [
            (0.0, 0.0),    # crank + idle
            (4.0, 15.0),   # gentle tip-in
            (9.0, 40.0),   # part throttle
            (14.0, 15.0),  # back off
            (19.0, 70.0),  # hard pull (exercises knock control)
            (24.0, 0.0),   # overrun fuel-cut
            (29.0, 30.0),  # cruise
            (34.0, 0.0),   # return to idle
        ],
        True,
    ),
    "wot": ([(0.0, 0.0), (3.0, 100.0)], False),  # free-rev into the limiter
    "blip": (
        [(0.0, 0.0), (4.0, 60.0), (4.6, 0.0), (9.0, 60.0), (9.6, 0.0)],
        True,
    ),
}


class Driver:
    def __init__(self, profile: str = "drive"):
        if profile not in _PROFILES:
            raise ValueError(f"unknown profile {profile!r}; choose from {list(_PROFILES)}")
        self.profile = profile
        self._segments, self._loaded = _PROFILES[profile]

    def throttle_pct(self, t: float) -> float:
        value = self._segments[0][1]
        for ts, thr in self._segments:
            if t >= ts:
                value = thr
            else:
                break
        return value

    def load_nm(self, rpm: float) -> float:
        if not self._loaded:
            return 0.0
        return ROAD_LOAD_QUAD * (rpm / 1000.0) ** 2

    @property
    def duration_hint(self) -> float:
        """A sensible run length for this profile (last setpoint + tail)."""
        return self._segments[-1][0] + 6.0
