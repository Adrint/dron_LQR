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
    if X <= 1.0:
        Z = 1.0
    elif X > 1.0 and X < 1.5:
        Z = 1.0 + (X - 1.0) * 10.0
    elif X >= 1.5 and X <= 2.0:
        Z = 6.0
    elif X >= 2.0 and X <= 2.5:
        Z = 6.0 - (X - 2.0) * 10.0
    else:  # X >= 2.5
        Z = 1.0

    Z1 = Z

    # Calculate terrain height at next position
    dx = Vel * dt
    X_next = X + dx

    if X_next <= 1.0:
        Z = 1.0
    elif X_next > 1.0 and X_next < 1.5:
        Z = 1.0 + (X_next - 1.0) * 10.0
    elif X_next >= 1.5 and X_next <= 2.0:
        Z = 6.0
    elif X_next >= 2.0 and X_next <= 2.5:
        Z = 6.0 - (X_next - 2.0) * 10.0
    else:  # X_next >= 2.5
        Z = 1.0

    # Calculate slope angle
    alpha = np.arctan2(Z - Z1, dx)

    return Z1, alpha

def generate_reference_profile(Vel, dt):
    dx = Vel * dt
    X_ref = np.arange(0, 5 + dx, dx)

    Z_terr = []
    for X in X_ref:
        if X <= 1.0:
            Z = 1.0
        elif X < 1.5:
            Z = 1.0 + (X - 1.0) * 10.0
        elif X <= 2.0:
            Z = 6.0
        elif X <= 2.5:
            Z = 6.0 - (X - 2.0) * 10.0
        else:
            Z = 1.0
        Z_terr.append(Z)

    Z_terr = np.array(Z_terr)
    alpha = np.gradient(Z_terr, dx)  # zamiast ręcznie liczyć różnice
    alpha = np.arctan(alpha)  # nachylenie w radianach

    return X_ref, Z_terr, Z_terr + 1.0, alpha  # h_flight = 1.0
