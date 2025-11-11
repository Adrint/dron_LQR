import numpy as np
import math
import pickle
import os
from scipy.interpolate import interp1d
from geopy.distance import geodesic


def load_planned_path(path_file='planned_path.npz'):
    """
    Wczytaj zaplanowaną ścieżkę z pliku.

    Parameters:
    -----------
    path_file : str
        Ścieżka do pliku z zaplanowaną trajektorią

    Returns:
    --------
    path_data : dict
        Słownik z danymi ścieżki lub None jeśli plik nie istnieje
    """
    if not os.path.exists(path_file):
        print(f"⚠ Brak pliku: {path_file}")
        return None

    try:
        data = np.load(path_file)
        print(f"✓ Wczytano zaplanowaną ścieżkę z {path_file}")
        print(f"  - Punktów: {len(data['X_ref'])}")

        return {
            'X_ref': data['X_ref'],
            'Y_ref': data['Y_ref'],
            'Z_ref': data['Z_ref'],
            'from_planner': True
        }
    except Exception as e:
        print(f"✗ Błąd wczytywania: {e}")
        return None


def generate_reference_profile(Vel, dt, avoid_distance, X_max=50, use_planned_path=True, path_file="planned_path.npz"):
    """
    Generate 3D reference trajectory profile including terrain and flight altitude.

    ============================================================================
    NED COORDINATE SYSTEM (North-East-Down):
    ============================================================================
    - X: Forward (North direction)
    - Y: Right (East direction)
    - Z: Down (positive Z points DOWNWARD)

    ALTITUDE CONVENTION:
    - Ground level: Z = 0
    - Above ground: Z < 0 (negative values)
      * Example: Z = -5.0 means 5 meters ABOVE ground
    - Below ground: Z > 0 (positive values)
      * Example: Z = +2.0 means 2 meters BELOW ground

    CLIMBING vs DESCENDING:
    - Climbing: Z becomes MORE negative (Z decreases: -1 → -6)
    - Descending: Z becomes LESS negative (Z increases: -6 → -1)

    PITCH ANGLE (alpha):
    - Positive alpha: Nose down (descending, Z increasing)
    - Negative alpha: Nose up (climbing, Z decreasing)
    - When alpha < 0: vz = V·sin(alpha) < 0 (upward velocity in NED)
    - When alpha > 0: vz = V·sin(alpha) > 0 (downward velocity in NED)
    ============================================================================

    Parameters:
    -----------
    Vel : float
        Drone velocity (m/s)
    dt : float
        Time step (s)
    avoid_distance : float
        Safety margin above terrain (m)
    X_max : float
        Maximum X range for generating trajectory (default: 50m)

    Returns:
    --------
    X_ref : ndarray
        Horizontal reference positions in forward direction (m)
    Y_terr : ndarray
        Terrain lateral positions at each X (m)
        NED: Y positive = right/east
    Z_terr : ndarray
        Terrain heights at each X (m)
        NED: negative values = above ground level
        Example: Z_terr = -3.0 means terrain is 3m above ground reference
    Y_ref : ndarray
        Reference lateral positions for flight path (m)
    Z_ref : ndarray
        Reference flight altitudes (m)
        Always MORE negative than Z_terr to fly above terrain
        Example: If Z_terr = -3.0 and avoid_distance = 2.0,
                 then Z_ref = -5.0 (flying 5m above ground)
    alpha : ndarray
        Pitch angles - slope angles in X-Z plane (radians)
        Negative = climbing, Positive = descending
    beta : ndarray
        Roll angles - slope angles in X-Y plane (radians)
        For coordinated turns in lateral direction

    Example Usage:
    --------------
    # Generate trajectory with 5 m/s velocity, avoiding terrain by 2m
    X, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta = generate_reference_profile(
        Vel=5.0, dt=0.02, avoid_distance=2.0
    )

    # At any point:
    # - Z_terr[i] = terrain altitude (e.g., -3.0 = 3m AGL)
    # - Z_ref[i] = flight altitude (e.g., -5.0 = 5m AGL)
    # - Actual altitude above ground = -Z_ref[i]
    """
    # ========================================================================
    # Spróbuj załadować zaplanowaną ścieżkę z plannera
    # ========================================================================
    if use_planned_path:
        planned = load_planned_path(path_file)
        if planned is not None:
            print("[Trajectory] Używam zaplanowanej ścieżki z A*")

            X_ref = planned['X_ref']
            Y_ref = planned['Y_ref']
            Z_ref = planned['Z_ref']

            # Dla kompatybilności z resztą kodu
            Y_terr = Y_ref.copy()
            Z_terr = Z_ref.copy()

            # Oblicz kąty alpha i beta
            dX = np.gradient(X_ref)
            dY = np.gradient(Y_ref)
            dZ = np.gradient(Z_ref)

            # Alpha: kąt pitch (pionowy)
            alpha = np.arctan2(dZ, np.sqrt(dX ** 2 + dY ** 2))

            # Beta: kąt roll/lateral
            beta = np.arctan2(dY, dX)

            return X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta

    # ========================================================================
    # Jeśli brak zaplanowanej ścieżki, generuj sztucznie (legacy)
    # ========================================================================
    print("[Trajectory] Generuję ścieżkę syntetyczną (legacy mode)")

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