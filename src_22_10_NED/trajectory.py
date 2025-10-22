import numpy as np
import math

def aa_trajectory(X, Vel, dt):
    """
    Calculate terrain height and slope angle at position X
    
    NED Coordinate System: Z positive downward
    - Ground level: Z = 0
    - Terrain features: Z <= 0 (negative values = above ground)

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
        Terrain height at position X (NED: negative = above ground)
    alpha : float
        Slope angle (radians)
    """
    # Calculate terrain height at current position
    if X <= 10.0:
        Z = -1.0  # 1m above ground
    elif X < 20.0:
        Z = -1.0 - (X - 10.0) * (5.0 / 10.0)  # Linear rise from -1m to -6m
    elif X <= 30.0:
        Z = -6.0  # 6m above ground
    elif X <= 40.0:
        Z = -6.0 + (X - 30.0) * (5.0 / 10.0)  # Linear descent from -6m to -1m
    else:
        Z = -1.0

    Z1 = Z

    # Calculate terrain height at next position
    dx = Vel * dt
    X_next = X + dx

    if X_next <= 10.0:
        Z = -1.0
    elif X_next < 20.0:
        Z = -1.0 - (X_next - 10.0) * (5.0 / 10.0)  # -1 → -6
    elif X_next <= 30.0:
        Z = -6.0
    elif X_next <= 40.0:
        Z = -6.0 + (X_next - 30.0) * (5.0 / 10.0)  # -6 → -1
    else:
        Z = -1.0

    # Calculate slope angle
    alpha = np.arctan2(Z - Z1, dx)

    return Z1, alpha

def generate_reference_profile(Vel, dt, X_max=50):
    """
    Generate reference trajectory profile including terrain and flight altitude.
    
    NED Coordinate System:
    - Z positive = downward
    - Ground at Z = 0
    - Terrain/flight altitudes: Z < 0 (negative values)

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
        Terrain heights at each X (NED: negative values)
    Z_ref : ndarray
        Reference flight altitudes above terrain (NED: negative values)
    alpha : ndarray
        Slope angles of the terrain (in radians)
    """
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    for X in X_ref:
        if X <= 10.0:
            Z = -1.0  # 1m above ground
        elif X < 20.0:
            Z = -1.0 - (X - 10.0) * (5.0 / 10.0)  # -1 → -6
        elif X <= 30.0:
            Z = -6.0  # 6m above ground
        elif X <= 40.0:
            Z = -6.0 + (X - 30.0) * (5.0 / 10.0)  # -6 → -1
        else:
            Z = -1.0
        Z_terr.append(Z)

    Z_terr = np.array(Z_terr)
    alpha = np.gradient(Z_terr, dx)       # nachylenie (pochodna)
    alpha = np.arctan(alpha)              # konwersja do radianów

    Z_ref = Z_terr - 2.0  # Lot 2m nad terenem (w NED: -2m = dalej od ziemi)

    return X_ref, Z_terr, Z_ref, alpha





