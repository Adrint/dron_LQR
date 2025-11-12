import numpy as np
from scipy.interpolate import interp1d
from pathlib import Path
import pandas as pd
from shapely.geometry import Point

# Ścieżki plików
PLANNED_PATH = Path("planned_path.npz")
DATA_DIR = Path("data")
BUILDINGS_FILE = DATA_DIR / "warsaw_buildings.pkl"


def _estimate_height(row):
    """Oszacuj wysokość budynku z danych OSM"""
    h = None
    if "height" in row.index and pd.notna(row["height"]):
        try:
            s = str(row["height"]).lower().replace("m", "").strip()
            h = float(s)
        except Exception:
            pass
    if h is None and "building:levels" in row.index and pd.notna(row["building:levels"]):
        try:
            levels = float(row["building:levels"])
            h = levels * 3.0
        except Exception:
            pass
    return float(h if h is not None else 10.0)


def _generate_from_planned(Vel, dt, avoid_distance):
    """Generuj profil referencyjny z planned_path.npz wygenerowanego przez algorytm A*"""

    # Wczytaj ścieżkę z algorytmu A*
    data = np.load(PLANNED_PATH)
    X_planned = data['X_ref']  # Współrzędne X w lokalnych metrach
    Y_planned = data['Y_ref']  # Współrzędne Y w lokalnych metrach
    Z_planned = data['Z_ref']  # Współrzędne Z w NED (ujemne dla wysokości)

    print(f"\n[Trajectory] Wczytano ścieżkę z {PLANNED_PATH}")
    print(f"  Punkty: {len(X_planned)}")
    print(f"  X: {X_planned[0]:.1f} -> {X_planned[-1]:.1f} m")
    print(f"  Y: {Y_planned.min():.1f} -> {Y_planned.max():.1f} m")
    print(f"  Z: {Z_planned.min():.1f} -> {Z_planned.max():.1f} m (NED)")

    # Oblicz długość ścieżki
    dx = np.diff(X_planned)
    dy = np.diff(Y_planned)
    dz = np.diff(Z_planned)
    ds = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
    s = np.concatenate([[0.0], np.cumsum(ds)])

    total_distance = s[-1]
    print(f"  Całkowita długość: {total_distance:.1f} m")

    # Resample do równych odstępów bazując na prędkości
    dx_uniform = Vel * dt
    num_points = int(total_distance / dx_uniform) + 1
    s_uniform = np.linspace(0, total_distance, num_points)

    # Interpolacja do równych odstępów
    X_ref = np.interp(s_uniform, s, X_planned)
    Y_ref = np.interp(s_uniform, s, Y_planned)
    Z_ref = np.interp(s_uniform, s, Z_planned)

    # Oblicz kąty alpha (pitch) i beta (roll) dla regulatora
    dX = np.gradient(X_ref, dx_uniform)
    dY = np.gradient(Y_ref, dx_uniform)
    dZ = np.gradient(Z_ref, dx_uniform)

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
                    h = _estimate_height(bldg)
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

    return X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta


def _generate_synthetic(Vel, dt, avoid_distance, X_max=50):
    """Generuj syntetyczny profil (backup gdy brak planned_path.npz)"""
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
        Z_terr.append(Z);
        Y_terr.append(Y)

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