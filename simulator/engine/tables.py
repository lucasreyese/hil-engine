"""
Calibration lookup tables for the engine plant.

These describe the *physics* of the modeled engine (how well it breathes, how
much it rubs, where it knocks, where peak-torque spark sits). They are the
plant's ground truth -- distinct from whatever maps the ECM happens to carry.
The ECM only ever sees these through sensor outputs over CAN.

Pure-Python linear interpolation is used so the engine model imports with no
third-party dependencies (handy for offline unit tests).
"""

from bisect import bisect_right


def interp1d(x, xs, ys):
    """Clamped linear interpolation of ys(xs) at x. xs must be ascending."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect_right(xs, x) - 1
    t = (x - xs[i]) / (xs[i + 1] - xs[i])
    return ys[i] + t * (ys[i + 1] - ys[i])


def interp2d(x, y, xs, ys, grid):
    """Clamped bilinear interpolation. grid[i][j] aligns with xs[i], ys[j]."""
    # Interpolate along y for the two bracketing x rows, then across x.
    if x <= xs[0]:
        i, tx = 0, 0.0
    elif x >= xs[-1]:
        i, tx = len(xs) - 2, 1.0
    else:
        i = bisect_right(xs, x) - 1
        tx = (x - xs[i]) / (xs[i + 1] - xs[i])
    lo = interp1d(y, ys, grid[i])
    hi = interp1d(y, ys, grid[i + 1])
    return lo + tx * (hi - lo)


# ---------------------------------------------------------------------------
# Volumetric efficiency
# ---------------------------------------------------------------------------
# Breathing curve vs engine speed (peaks mid-range, like a real NA 4-cyl).
VE_RPM_BP = [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500]
VE_RPM_VAL = [0.45, 0.55, 0.65, 0.74, 0.82, 0.88, 0.92, 0.95, 0.94, 0.92, 0.90, 0.86, 0.80]

# Mild correction vs manifold-pressure ratio (more residuals at low MAP -> lower VE).
VE_MAPRATIO_BP = [0.10, 0.30, 0.50, 0.70, 1.00]
VE_MAPRATIO_VAL = [0.85, 0.92, 0.96, 0.99, 1.00]


def volumetric_efficiency(rpm: float, map_ratio: float) -> float:
    return interp1d(rpm, VE_RPM_BP, VE_RPM_VAL) * interp1d(
        map_ratio, VE_MAPRATIO_BP, VE_MAPRATIO_VAL
    )


# ---------------------------------------------------------------------------
# Friction + pumping (mechanical) mean effective pressure, kPa
# ---------------------------------------------------------------------------
# Rising with speed; this is the FMEP that opposes the indicated work. Pumping
# loss from throttling is captured separately by the intake-pressure model.
FMEP_RPM_BP = [500, 1000, 2000, 3000, 4000, 5000, 6000, 6500]
FMEP_KPA_VAL = [95, 100, 115, 135, 165, 205, 255, 285]


def friction_fmep_kpa(rpm: float) -> float:
    return interp1d(rpm, FMEP_RPM_BP, FMEP_KPA_VAL)


# ---------------------------------------------------------------------------
# MBT (minimum spark for best torque), deg BTDC
# ---------------------------------------------------------------------------
# Rows = rpm, cols = manifold-pressure ratio (load). More advance at high
# speed / light load; pulled back at high load where burn is fast/knock-limited.
MBT_RPM_BP = [800, 2000, 3500, 5000, 6500]
MBT_LOAD_BP = [0.20, 0.50, 0.80, 1.00]
MBT_DEG = [
    # load: 0.20  0.50  0.80  1.00
    [22.0, 18.0, 14.0, 12.0],  # 800 rpm
    [30.0, 26.0, 20.0, 17.0],  # 2000 rpm
    [36.0, 32.0, 26.0, 22.0],  # 3500 rpm
    [40.0, 36.0, 30.0, 26.0],  # 5000 rpm
    [42.0, 38.0, 32.0, 28.0],  # 6500 rpm
]


def mbt_advance_deg(rpm: float, map_ratio: float) -> float:
    return interp2d(rpm, map_ratio, MBT_RPM_BP, MBT_LOAD_BP, MBT_DEG)


# ---------------------------------------------------------------------------
# Borderline knock limit (max safe advance before knock onset), deg BTDC
# ---------------------------------------------------------------------------
# At light load the knock limit is well above MBT (knock-free). At high load it
# drops below MBT, so MBT spark would knock and a real ECM must retard. This is
# the reference at KNOCK_ECT_REF_C; the model adds a temperature sensitivity.
KNOCK_RPM_BP = [800, 2000, 3500, 5000, 6500]
KNOCK_LOAD_BP = [0.20, 0.50, 0.80, 1.00]
KNOCK_LIMIT_DEG = [
    # load:  0.20   0.50   0.80   1.00
    [55.0, 34.0, 18.0, 12.0],  # 800 rpm
    [60.0, 40.0, 24.0, 17.0],  # 2000 rpm
    [60.0, 44.0, 30.0, 23.0],  # 3500 rpm
    [60.0, 50.0, 36.0, 30.0],  # 5000 rpm
    [60.0, 55.0, 42.0, 36.0],  # 6500 rpm
]


def knock_limit_advance_deg(rpm: float, map_ratio: float) -> float:
    return interp2d(rpm, map_ratio, KNOCK_RPM_BP, KNOCK_LOAD_BP, KNOCK_LIMIT_DEG)
