import numpy as np
import math
import pickle
import os
from scipy.interpolate import interp1d
from geopy.distance import geodesic

def generate_reference_profile(Vel, dt, avoid_distance, X_max=50):
    """
    Generate 3D reference trajectory profile including terrain and flight altitude.
    
    NED Coordinate System:
    - X: Forward (North)
    - Y: Right (East)
    - Z: Down (positive downward)
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
        Horizontal reference positions (forward)
    Y_terr : ndarray
        Terrain lateral positions at each X (NED: Y positive = right)
    Z_terr : ndarray
        Terrain heights at each X (NED: negative values = above ground)
    Y_ref : ndarray
        Reference lateral positions for flight
    Z_ref : ndarray
        Reference flight altitudes above terrain (NED: negative values)
    alpha : ndarray
        Pitch angles - slope angles in X-Z plane (radians)
    beta : ndarray
        Roll angles - slope angles in X-Y plane (radians)
    """
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    Y_terr = []

    # Generate terrain profile with both vertical (Z) and lateral (Y) variations
    for X in X_ref:
        if X <= 10.0:
            # Flat section at low altitude, centered
            Z = -1.0  # 1 m above ground (NED: negative = altitude)
            Y = 0.0   # Centered (no lateral deviation)
            
        elif X < 20.0:
            # Climbing and moving right
            progress = (X - 10) / 10.0  # 0 to 1
            Z = -1.0 - progress * 5.0     # -1m → -6m (climbing)
            Y = progress * 5.0            # 0 m → 5 m (moving right)
            
        elif X <= 30.0:
            # High altitude, offset to the right
            Z = -6.0  # 6 m above ground
            Y = 5.0   # 5 m to the right
            
        elif X < 40.0:
            # Descending and moving back to center
            progress = (X - 30.0) / 10.0  # 0 to 1
            Z = -6.0 + progress * 5.0     # -6 m → -1 m (descending)
            Y = 5.0 - progress * 5.0      # 5 m → 0 m (moving left/center)
            
        else:
            # Final flat section at low altitude, centered
            Z = -1.0
            Y = 0.0
            
        Z_terr.append(Z)
        Y_terr.append(Y)

    Z_terr = np.array(Z_terr)
    Y_terr = np.array(Y_terr)
    
    # Calculate slope angles
    # Alpha: pitch angle (slope in X-Z plane)
    dZ_dX = np.gradient(Z_terr, dx)
    alpha = np.arctan(dZ_dX)
    
    # Beta: roll angle needed for lateral trajectory (slope in X-Y plane)
    dY_dX = np.gradient(Y_terr, dx)
    beta = np.arctan(dY_dX)

    # Reference trajectory: fly 2 m above terrain
    # In NED: subtracting 2 means going more negative (higher altitude)
    Z_ref = Z_terr - avoid_distance  # Fly 2 m above terrain
    Y_ref = Y_terr.copy()

    return X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta

