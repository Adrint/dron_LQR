import numpy as np
import math
import pickle
import os
from scipy.interpolate import interp1d
from geopy.distance import geodesic

def generate_reference_profile(Vel, dt, avoid_distance, X_max=50):

    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    Y_terr = []

    # ========================================================================
    # Generate terrain profile with vertical (Z) and lateral (Y) variations
    # ========================================================================
    for X in X_ref:
        if X <= 10.0:
            # Segment 1: Flat section at low altitude, centered
            Z = -1.0  # 1 meter above ground (AGL = 1m)
            Y = 0.0  # Centered (no lateral deviation)

        elif X < 20.0:
            # Segment 2: Climbing and moving right
            progress = (X - 10) / 10.0  # Linear interpolation: 0 → 1
            Z = -1.0 - progress * 5.0  # Climb: -1m → -6m (AGL: 1m → 6m)
            Y = progress * 5.0  # Move right: 0m → 5m

        elif X <= 30.0:
            # Segment 3: High altitude cruise, offset to the right
            Z = -6.0  # Maintain 6 meters above ground (AGL = 6m)
            Y = 5.0  # Maintain 5 meters right offset

        elif X < 40.0:
            # Segment 4: Descending and returning to center
            progress = (X - 30.0) / 10.0  # Linear interpolation: 0 → 1
            Z = -6.0 + progress * 5.0  # Descend: -6m → -1m (AGL: 6m → 1m)
            Y = 5.0 - progress * 5.0  # Return to center: 5m → 0m

        else:
            # Segment 5: Final flat section at low altitude, centered
            Z = -1.0  # Back to 1 meter above ground
            Y = 0.0  # Centered

        Z_terr.append(Z)
        Y_terr.append(Y)

    Z_terr = np.array(Z_terr)
    Y_terr = np.array(Y_terr)

    # ========================================================================
    # Calculate slope angles for trajectory following
    # ========================================================================

    # Alpha: pitch angle (vertical slope in X-Z plane)
    # - Negative alpha = climbing (Z decreasing)
    # - Positive alpha = descending (Z increasing)
    dZ_dX = np.gradient(Z_terr, dx)
    alpha = np.arctan(dZ_dX)

    # Beta: roll angle for lateral trajectory (horizontal slope in X-Y plane)
    # Used for coordinated turns when following lateral path
    dY_dX = np.gradient(Y_terr, dx)
    beta = np.arctan(dY_dX)

    # ========================================================================
    # Calculate reference flight path (maintain safety margin above terrain)
    # ========================================================================

    # Reference altitude: fly 'avoid_distance' meters above terrain
    # In NED: subtracting avoid_distance makes Z MORE negative (higher altitude)
    # Example: If Z_terr = -3m (3m AGL) and avoid_distance = 2m
    #          Then Z_ref = -3 - 2 = -5m (5m AGL)
    Z_ref = Z_terr - avoid_distance
    Y_ref = Y_terr.copy()

    return X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta