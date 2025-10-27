import numpy as np
import math


def generate_reference_profile(Vel, dt, X_max=50):
    """
    Generate reference trajectory profile including terrain and flight altitude.
    
    NED Coordinate System:
    - Z positive = downward
    - Ground at Z = 0
    - Terrain/flight altitudes: Z < 0 (negative values)
    - Y is lateral position (left/right)

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
        Horizontal reference positions (forward)
    Y_ref : ndarray
        Lateral reference positions (left/right)
    Z_terr : ndarray
        Terrain heights at each X (NED: negative values)
    Z_ref : ndarray
        Reference flight altitudes above terrain (NED: negative values)
    alpha : ndarray
        Slope angles in Z direction (pitch, in radians)
    beta : ndarray
        Slope angles in Y direction (lateral, in radians)
    """
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    Y_ref = []
    
    for X in X_ref:
        if X <= 10.0:
            Z = 0.0  # Ground level
            y = 0.0  # Straight line
        elif X < 20.0:
            Z = -(X - 10.0) * (5.0 / 10.0)  # Climbing
            y = (X - 10.0) * (3.0 / 10.0)   # Move right gradually
        elif X <= 30.0:
            Z = -5.0  # Constant altitude
            y = 3.0   # Straight at y=3
        elif X <= 40.0:
            Z = -5.0 + (X - 30.0) * (5.0 / 10.0)  # Descending
            y = 3.0 - (X - 30.0) * (3.0 / 10.0)   # Move back to center
        else:
            Z = 0.0   # Ground level
            y = 0.0   # Back to center
            
        Z_terr.append(Z)
        Y_ref.append(y)

    Z_terr = np.array(Z_terr)
    Y_ref = np.array(Y_ref)
    
    # Calculate slopes (gradients with respect to X)
    alpha = np.gradient(Z_terr, dx)  # dZ/dX slope (pitch)
    alpha = np.arctan(alpha)         # Convert to radians
    
    beta = np.gradient(Y_ref, dx)    # dY/dX slope (lateral)
    beta = np.arctan(beta)           # Convert to radians

    Z_ref = Z_terr - 2.0  # Flight 2m above terrain (in NED: -2m = farther from ground)

    return X_ref, Y_ref, Z_terr, Z_ref, alpha, beta
