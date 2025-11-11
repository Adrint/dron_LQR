
import numpy as np
from scipy.interpolate import interp1d
from pathlib import Path

PLANNED_PATH = Path("planned_path.npz")

def _resample_uniform(x, y, new_x):
    """Interpolate y(x) to new_x using linear interpolation with edge handling."""
    f = interp1d(x, y, kind="linear", fill_value="extrapolate", assume_sorted=False)
    return f(new_x)

def _rotate(points_xy, angle_rad):
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    R = np.array([[c, -s],
                  [s,  c]])
    return (R @ points_xy.T).T

def _generate_from_planned(Vel, dt, avoid_distance):
    data = np.load(PLANNED_PATH)

    # Planned path saved earlier (local meters, Z_ref is NED)
    Xp = data["X_ref"].astype(float)
    Yp = data["Y_ref"].astype(float)
    Zned = data["Z_ref"].astype(float)  # NED (negative = up)

    # Convert NED to "up" (positive up)
    Zup = -Zned

    # Align path so forward axis is +X (rotate by heading from first to last point)
    dx = Xp[-1] - Xp[0]
    dy = Yp[-1] - Yp[0]
    heading = np.arctan2(dy, dx)

    XY = np.column_stack([Xp - Xp[0], Yp - Yp[0]])
    XYr = _rotate(XY, -heading)  # now approx along +X
    X_along = XYr[:, 0]
    Y_lat = XYr[:, 1]

    # Arc-length in XY for monotonic forward coordinate
    dxy = np.hypot(np.diff(X_along), np.diff(Y_lat))
    s = np.concatenate([[0.0], np.cumsum(dxy)])

    # Resample to uniform dx = Vel*dt
    dx_u = max(Vel * dt, 1e-6)
    X_ref = np.arange(0.0, float(s[-1]) + dx_u, dx_u)

    # Interpolate lateral and vertical profiles along s
    Y_ref = _resample_uniform(s, Y_lat, X_ref)
    Z_up_ref = _resample_uniform(s, Zup, X_ref)

    # Convert vertical profile to NED
    Z_ref = -Z_up_ref

    # Terrain baseline (for visualizer): keep a constant margin below ref
    Z_terr = Z_ref + avoid_distance  # NED: closer to 0 (less negative) = lower
    Y_terr = np.zeros_like(X_ref)    # baseline centered terrain

    # Slopes for guidance
    dZ_dX = np.gradient(Z_ref, dx_u)
    alpha = np.arctan(dZ_dX)  # pitch slope from reference

    dY_dX = np.gradient(Y_ref, dx_u)
    beta = np.arctan(dY_dX)   # lateral slope from reference

    return X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta

def _generate_synthetic(Vel, dt, avoid_distance, X_max=50):
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    Y_terr = []

    for X in X_ref:
        if X <= 10.0:
            Z = -1.0
            Y = 0.0
        elif X < 20.0:
            progress = (X - 10) / 10.0
            Z = -1.0 - progress * 5.0
            Y = progress * 5.0
        elif X <= 30.0:
            Z = -6.0
            Y = 5.0
        elif X < 40.0:
            progress = (X - 30.0) / 10.0
            Z = -6.0 + progress * 5.0
            Y = 5.0 - progress * 5.0
        else:
            Z = -1.0
            Y = 0.0
        Z_terr.append(Z); Y_terr.append(Y)

    Z_terr = np.array(Z_terr, dtype=float)
    Y_terr = np.array(Y_terr, dtype=float)

    dZ_dX = np.gradient(Z_terr, dx)
    alpha = np.arctan(dZ_dX)

    dY_dX = np.gradient(Y_terr, dx)
    beta = np.arctan(dY_dX)

    Z_ref = Z_terr - avoid_distance
    Y_ref = Y_terr.copy()

    return X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta

def generate_reference_profile(Vel, dt, avoid_distance, **kwargs):

    try:
        if PLANNED_PATH.exists():
            return _generate_from_planned(Vel, dt, avoid_distance)
    except Exception as e:
        # W razie problemu – fallback
        print(f"[trajectory] ⚠️ Nie udało się wczytać planned_path.npz ({e}). Używam profilu syntetycznego.")
    return _generate_synthetic(Vel, dt, avoid_distance)
