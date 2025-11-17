import numpy as np
from pathlib import Path
import pandas as pd

# Ścieżki plików
PLANNED_PATH = Path("planned_path.npz")
DATA_DIR = Path("data")
BUILDINGS_FILE = DATA_DIR / "warsaw_buildings.pkl"


def _generate_from_planned(Vel, dt, avoid_distance):
    """Generuj profil referencyjny z planned_path.npz wygenerowanego przez algorytm A*"""

    # Wczytaj ścieżkę z algorytmu A*
    data = np.load(PLANNED_PATH)
    s_planned = data['s_ref']  # Dystans wzdłuż trasy
    Y_planned = data['Y_ref']  # Współrzędne Y w lokalnych metrach
    Z_planned = data['Z_ref']  # Współrzędne Z w NED (ujemne dla wysokości)
    X_planned = data['X_ref']  # Współrzędne X (dla map/wizualizacji)

    print(f"\n[Trajectory] Wczytano ścieżkę z {PLANNED_PATH}")
    print(f"  Punkty: {len(s_planned)}")
    print(f"  Dystans: 0 -> {s_planned[-1]:.1f} m")

    total_distance = s_planned[-1]

    # Resample do równych odstępów bazując na prędkości
    ds_uniform = Vel * dt
    num_points = int(total_distance / ds_uniform) + 1
    s_uniform = np.linspace(0, total_distance, num_points)

    # Interpolacja do równych odstępów
    Y_ref = np.interp(s_uniform, s_planned, Y_planned)
    Z_ref = np.interp(s_uniform, s_planned, Z_planned)
    X_ref = np.interp(s_uniform, s_planned, X_planned)

    # Oblicz kąty alpha (pitch) i beta (yaw heading) dla regulatora
    dX = np.gradient(X_ref, ds_uniform)
    dY = np.gradient(Y_ref, ds_uniform)
    dZ = np.gradient(Z_ref, ds_uniform)

    # Normalizacja wektora kierunku
    norm = np.sqrt(dX ** 2 + dY ** 2 + dZ ** 2)
    norm = np.where(norm > 1e-6, norm, 1.0)

    dX_norm = dX / norm
    dY_norm = dY / norm
    dZ_norm = dZ / norm

    # Alpha - kąt nachylenia pionowego (pitch)
    # Ujemny bo w NED dodatnie Z to w dół
    alpha = np.arctan2(dZ_norm, np.sqrt(dX_norm ** 2 + dY_norm ** 2))

    # Beta - kąt kierunku poziomego (heading)
    beta = np.arctan2(dY_norm, dX_norm)

    # Zwracamy: dystans, Y, Z, alpha, beta, X
    return s_uniform, Y_ref, Z_ref, alpha, beta, X_ref


def _generate_synthetic(Vel, dt, avoid_distance):
    """Generuj syntetyczny profil (backup gdy brak planned_path.npz)"""

    s_max = 100.0  # Maksymalny dystans
    ds = Vel * dt
    s_ref = np.arange(0, s_max + ds, ds)

    # Prosty profil testowy
    num_points = len(s_ref)
    X_ref = s_ref.copy()  # X = dystans
    Y_ref = 10 * np.sin(2 * np.pi * s_ref / s_max)  # Oscylacja boczna
    Z_ref = -5 - 3 * np.sin(4 * np.pi * s_ref / s_max)  # Oscylacja wysokości (NED)

    # Oblicz kąty
    dX = np.gradient(X_ref, ds)
    dY = np.gradient(Y_ref, ds)
    dZ = np.gradient(Z_ref, ds)

    norm = np.sqrt(dX ** 2 + dY ** 2 + dZ ** 2)
    norm = np.where(norm > 1e-6, norm, 1.0)

    alpha = np.arctan2(dZ / norm, np.sqrt((dX / norm) ** 2 + (dY / norm) ** 2))
    beta = np.arctan2(dY / norm, dX / norm)

    return s_ref, Y_ref, Z_ref, alpha, beta, X_ref


def generate_reference_profile(Vel, dt, avoid_distance):
    """Główna funkcja generowania profilu referencyjnego

    Returns:
        s_ref: dystans wzdłuż trasy [m]
        Y_ref: współrzędna Y (East) [m]
        Z_ref: współrzędna Z (Down, NED) [m]
        alpha: kąt nachylenia (pitch) [rad]
        beta: kąt kierunku (heading) [rad]
        X_ref: współrzędna X (North) [m]
    """

    if PLANNED_PATH.exists():
        try:
            return _generate_from_planned(Vel, dt, avoid_distance)
        except Exception as e:
            print(f"[trajectory] ⚠️ Błąd wczytywania planned_path.npz: {e}")
            print("[trajectory] Używam profilu syntetycznego.")
    else:
        print(f"[trajectory] ⚠️ Brak pliku {PLANNED_PATH}. Używam profilu syntetycznego.")

    return _generate_synthetic(Vel, dt, avoid_distance)