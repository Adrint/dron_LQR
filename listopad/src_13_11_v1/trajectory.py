import numpy as np
from scipy.interpolate import interp1d
from pathlib import Path
import pandas as pd
from shapely.geometry import Point
from geo_utils import create_transformer, wgs84_to_local, estimate_building_height

# Ścieżki plików
PLANNED_PATH = Path("planned_path.npz")
DATA_DIR = Path("data")
BUILDINGS_FILE = DATA_DIR / "warsaw_buildings.pkl"


def _generate_from_planned(Vel, dt, avoid_distance):
    """Generuj profil referencyjny z planned_path.npz wygenerowanego przez algorytm A*"""

    # Wczytaj ścieżkę z algorytmu A*
    data = np.load(PLANNED_PATH)
    s_planned = data['s_ref']  # Dystans wzdłuż trasy (oś X)
    Y_planned = data['Y_ref']  # Współrzędne Y w lokalnych metrach
    Z_planned = data['Z_ref']  # Współrzędne Z w NED (ujemne dla wysokości)
    X_planned = data['X_ref']  # Współrzędne X (dla map/wizualizacji)

    print(f"\n[Trajectory] Wczytano ścieżkę z {PLANNED_PATH}")
    print(f"  Punkty: {len(s_planned)}")
    print(f"  Dystans: 0 -> {s_planned[-1]:.1f} m")
    print(f"  Y: {Y_planned.min():.1f} -> {Y_planned.max():.1f} m")
    print(f"  Z: {Z_planned.min():.1f} -> {Z_planned.max():.1f} m (NED)")

    total_distance = s_planned[-1]
    print(f"  Całkowita długość: {total_distance:.1f} m")

    # Resample do równych odstępów bazując na prędkości
    ds_uniform = Vel * dt
    num_points = int(total_distance / ds_uniform) + 1
    s_uniform = np.linspace(0, total_distance, num_points)

    # Interpolacja do równych odstępów
    Y_ref = np.interp(s_uniform, s_planned, Y_planned)
    Z_ref = np.interp(s_uniform, s_planned, Z_planned)
    X_ref = np.interp(s_uniform, s_planned, X_planned)  # Dla kompatybilności

    # Oblicz kąty alpha (pitch) i beta (roll) dla regulatora
    # Używamy X, Y, Z dla obliczeń kierunku
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
    alpha = np.arctan2(dZ_norm, np.sqrt(dX_norm ** 2 + dY_norm ** 2))

    # Beta - kąt odchylenia bocznego (roll)
    beta = np.arctan2(dY_norm, dX_norm)

    # ====================================================================
    # WCZYTAJ BUDYNKI I OBLICZ WYSOKOŚCI "TERENU" (ZABUDOWY)
    # ====================================================================

    Y_terr = Y_ref.copy()
    Z_terr = np.zeros_like(Z_ref)  # Domyślnie grunt na poziomie 0

    if BUILDINGS_FILE.exists():
        try:
            buildings = pd.read_pickle(BUILDINGS_FILE)
            print(f"  Wczytano {len(buildings)} budynków")

            # Dla każdego punktu na ścieżce znajdź wysokość budynku pod nim
            # To jest uproszczona wersja - dla pełnej precyzji potrzebna byłaby
            # konwersja między lokalnym układem metrów a współrzędnymi geo

            # Dla wizualizacji wystarczy pokazać maksymalną wysokość w regionie
            max_height = 0.0
            for _, bldg in buildings.iterrows():
                try:
                    h = estimate_building_height(bldg)
                    if h > max_height:
                        max_height = h
                except Exception:
                    continue

            # Utwórz profil wysokości budynków (uproszczony - stała wysokość)
            Z_terr = np.full_like(Z_ref, -max_height * 0.3)  # 30% max wysokości jako "teren"

            print(f"  Maksymalna wysokość budynków: {max_height:.1f} m")

        except Exception as e:
            print(f"  ⚠️ Błąd wczytywania budynków: {e}")
            Z_terr = np.zeros_like(Z_ref)
    else:
        print(f"  ⚠️ Brak pliku z budynkami: {BUILDINGS_FILE}")
        Z_terr = np.zeros_like(Z_ref)

    # PROBLEM 1: Zwróć dystans jako pierwszą współrzędną (zamiast X_ref)
    # Ale również zwróć X_ref dla wykresów 3D
    return s_uniform, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta, X_ref


def _generate_synthetic(Vel, dt, avoid_distance, s_max=50):
    """Generuj syntetyczny profil (backup gdy brak planned_path.npz)"""
    ds = Vel * dt
    s_ref = np.arange(0, s_max + ds, ds)  # Dystans zamiast X

    Z_terr = []
    Y_terr = []

    for s in s_ref:
        if s <= 10.0:
            Z = -1.0
            Y = 0.0
        elif s < 20.0:
            progress = (s - 10) / 10.0
            Z = -1.0 - progress * 5.0
            Y = progress * 5.0
        elif s <= 30.0:
            Z = -6.0
            Y = 5.0
        elif s < 40.0:
            progress = (s - 30.0) / 10.0
            Z = -6.0 + progress * 5.0
            Y = 5.0 - progress * 5.0
        else:
            Z = -1.0
            Y = 0.0
        Z_terr.append(Z);
        Y_terr.append(Y)

    Z_terr = np.array(Z_terr, dtype=float)
    Y_terr = np.array(Y_terr, dtype=float)

    dZ_ds = np.gradient(Z_terr, ds)
    alpha = np.arctan(dZ_ds)

    dY_ds = np.gradient(Y_terr, ds)
    beta = np.arctan(dY_ds)

    Z_ref = Z_terr - avoid_distance
    Y_ref = Y_terr.copy()

    # Dla uproszczenia, X = s (dystans = współrzędna X)
    X_ref = s_ref.copy()

    return s_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta, X_ref


def generate_reference_profile(Vel, dt, avoid_distance, **kwargs):
    """Główna funkcja generowania profilu referencyjnego"""

    # Spróbuj wczytać ścieżkę z algorytmu A*
    if PLANNED_PATH.exists():
        try:
            return _generate_from_planned(Vel, dt, avoid_distance)
        except Exception as e:
            print(f"[trajectory] ⚠️ Nie udało się wczytać planned_path.npz ({e}). Używam profilu syntetycznego.")
    else:
        print(f"[trajectory] ⚠️ Brak pliku {PLANNED_PATH}. Używam profilu syntetycznego.")

    return _generate_synthetic(Vel, dt, avoid_distance)