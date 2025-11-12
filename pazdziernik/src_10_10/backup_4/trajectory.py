import numpy as np


def aa_trajectory(X, Vel, dt):
    """
    Calculate terrain height and slope angle at position X

    Parameters:
    -----------
    X : float
        Current horizontal position
    Vel : float
        Velocity
    dt : float
        Time step

    Returns:
    --------
    Z : float
        Terrain height at position X
    alpha : float
        Slope angle (radians)
    """
    # Calculate terrain height at current position
    if X <= 10.0:
        Z = 1.0
    elif X < 20:
        Z = 1.0 + (X - 1.0) * 10.0
    elif X <= 30:
        Z = 6.0
    elif X <= 40:
        Z = 6.0 - (X - 2.0) * 10.0
    else:
        Z = 1.0

    Z1 = Z

    # Calculate terrain height at next position
    dx = Vel * dt
    X_next = X + dx

    if X_next <= 10.0:
        Z = 1.0
    elif X_next < 20.0:
        Z = 1.0 + (X_next - 10.0) * (5.0 / 10.0)  # 1 → 6
    elif X_next <= 30.0:
        Z = 6.0
    elif X_next <= 40.0:
        Z = 6.0 - (X_next - 30.0) * (5.0 / 10.0)  # 6 → 1
    else:
        Z = 1.0

    # Calculate slope angle
    alpha = np.arctan2(Z - Z1, dx)

    return Z1, alpha

def generate_reference_profile(Vel, dt, X_max=50):
    """
    Generate reference trajectory profile including terrain and flight altitude.

    Parameters:
    -----------
    Vel : float
        Drone velocity (m/s)
    dt : float
        Time step (s)
    X_max : float
        Maximum X range for generating trajectory

    Returns:
    --------
    X_ref : ndarray
        Horizontal reference positions
    Z_terr : ndarray
        Terrain heights at each X
    Z_ref : ndarray
        Reference flight altitudes above terrain
    alpha : ndarray
        Slope angles of the terrain (in radians)
    """
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    for X in X_ref:
        if X <= 10.0:
            Z = 1.0
        elif X < 20.0:
            Z = 1.0 + (X - 10.0) * (5.0 / 10.0)  # 1 → 6
        elif X <= 30.0:
            Z = 6.0
        elif X <= 40.0:
            Z = 6.0 - (X - 30.0) * (5.0 / 10.0)  # 6 → 1
        else:
            Z = 1.0
        Z_terr.append(Z)

    Z_terr = np.array(Z_terr)
    alpha = np.gradient(Z_terr, dx)       # nachylenie (pochodna)
    alpha = np.arctan(alpha)              # konwersja do radianów

    Z_ref = Z_terr + 2.0  # Lot 2m nad terenem

    return X_ref, Z_terr, Z_ref, alpha

def generate_terrain_profile(Vel, dt, X_max=50):
    """
    Generate terrain profile (Z_terr) based on predefined terrain rules.
    Returns X and Z arrays.

    Parameters:
    - Vel: reference velocity
    - dt: time step
    - X_max: max distance (m)

    Returns:
    - X: ndarray of horizontal positions
    - Z_terr: ndarray of terrain height at each X
    """
    dx = Vel * dt
    X = np.arange(0, X_max + dx, dx)
    Z_terr = []

    for x in X:
        # Define piecewise terrain
        if x <= 10.0:
            Z = 1.0
        elif x < 20.0:
            Z = 1.0 + (x - 10.0) * 2.0  # slope up fast
        elif x <= 30.0:
            Z = 21.0
        elif x <= 40.0:
            Z = 21.0 - (x - 30.0) * 1.5  # slope down
        else:
            Z = 6.0
        Z_terr.append(Z)

    return X, np.array(Z_terr)

def generate_reference_from_terrain(X, Z_terr, offset=2.0, mode="perpendicular"):
    """
    Create a reference path based on terrain profile using selected strategy.

    Parameters:
    - X: ndarray of horizontal positions
    - Z_terr: ndarray of terrain height
    - offset: float, distance to keep from terrain
    - mode: "perpendicular" | "vertical" | "fixed" — defines offset direction

    Returns:
    - X_ref: ndarray of reference X positions
    - Z_ref: ndarray of reference Z positions
    - alpha: ndarray of slope angles (radians)
    """
    dx = X[1] - X[0]  # spacing
    dz_dx = np.gradient(Z_terr, dx)
    alpha = np.arctan(dz_dx)  # slope angle (rad)

    if mode == "vertical":
        X_ref = X.copy()
        Z_ref = Z_terr + offset

    elif mode == "perpendicular":
        X_ref = X - offset * np.sin(alpha)
        Z_ref = Z_terr + offset * np.cos(alpha)

    elif mode == "fixed":
        # Fixed offset regardless of slope (can cause overlap on steep terrain)
        X_ref = X.copy()
        Z_ref = Z_terr + offset

    else:
        raise ValueError(f"Unknown mode: {mode}")

    return X_ref, Z_ref, alpha
