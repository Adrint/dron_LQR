import numpy as np
import pandas as pd
import pickle
from pathlib import Path
from heapq import heappush, heappop
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geo_utils import create_transformer, wgs84_to_local, local_to_wgs84, estimate_building_height

DATA_DIR = Path("data")
BUILDINGS_FILE = DATA_DIR / "warsaw_buildings.pkl"


class DroneRoutePlanner:
    def __init__(self, config):
        self.config = config
        self.buildings_gdf = None
        self.grid_3d = None
        self.ref_lat = None
        self.ref_lon = None
        self.transformer = None

    @staticmethod
    def latlon_to_meters(lat, lon, alt, ref_lat, ref_lon):
        transformer = create_transformer(ref_lat, ref_lon)
        x, y = wgs84_to_local(lat, lon, transformer)
        z = -alt  # NED: Z ujemne w górę
        return x, y, z

    @staticmethod
    def _estimate_height(row):
        return estimate_building_height(row)

    def _load_buildings(self):
        if not BUILDINGS_FILE.exists():
            print(f"⚠️ Brak pliku budynków: {BUILDINGS_FILE}")
            return None
        try:
            self.buildings_gdf = pd.read_pickle(BUILDINGS_FILE)
            print(f"✓ Wczytano {len(self.buildings_gdf)} budynków")
            return self.buildings_gdf
        except Exception as e:
            print(f"✗ Błąd wczytywania budynków: {e}")
            return None

    def _create_3d_grid(self, point_a, point_b):
        """Tworzy siatkę 3D z przeszkodami"""
        lat_a, lon_a = point_a
        lat_b, lon_b = point_b

        self.ref_lat = (lat_a + lat_b) / 2
        self.ref_lon = (lon_a + lon_b) / 2
        self.transformer = create_transformer(self.ref_lat, self.ref_lon)

        xa, ya = wgs84_to_local(lat_a, lon_a, self.transformer)
        xb, yb = wgs84_to_local(lat_b, lon_b, self.transformer)

        margin = 50.0
        x_min = min(xa, xb) - margin
        x_max = max(xa, xb) + margin
        y_min = min(ya, yb) - margin
        y_max = max(ya, yb) + margin

        res = self.config.grid_resolution
        nx = int((x_max - x_min) / res) + 1
        ny = int((y_max - y_min) / res) + 1

        z_min = 0.0
        z_max = self.config.altitude_cruise_max + 20.0
        nz = int(z_max / res) + 1

        print(f"\n[Grid] Wymiary: {nx} x {ny} x {nz}")
        print(f"  X: {x_min:.1f} -> {x_max:.1f} m")
        print(f"  Y: {y_min:.1f} -> {y_max:.1f} m")
        print(f"  Z: {z_min:.1f} -> {z_max:.1f} m (AGL)")

        grid = np.zeros((nx, ny, nz), dtype=np.uint8)

        if self.buildings_gdf is None or len(self.buildings_gdf) == 0:
            print("  ⚠️ Brak budynków - grid pusty")
            return grid, (x_min, x_max, y_min, y_max, z_min, z_max), (xa, ya, xb, yb)

        # ============================================================
        # NOWE: filtr budynków NA PODSTAWIE OBSZARU GRIDU (jak w _plot_3d_path)
        # ============================================================
        try:
            # ten sam margin co wyżej (w metrach), tylko w obie strony
            lat_min, lon_min = local_to_wgs84(x_min - margin, y_min - margin, self.transformer)
            lat_max, lon_max = local_to_wgs84(x_max + margin, y_max + margin, self.transformer)

            buildings_in = self.buildings_gdf.cx[
                           min(lon_min, lon_max):max(lon_min, lon_max),
                           min(lat_min, lat_max):max(lat_min, lat_max)
                           ]
        except Exception as e:
            print(f"  ⚠️ Błąd filtrowania budynków, używam wszystkich: {e}")
            buildings_in = self.buildings_gdf

        self.buildings_in_area = buildings_in
        print(f"  Budynki w obszarze: {len(buildings_in)}")

        avoid = self.config.avoid_distance

        # Wypełnij grid przeszkodami
        for idx, row in buildings_in.iterrows():
            try:
                geom = row.geometry
                if geom is None:
                    continue

                h = estimate_building_height(row)
                if h <= 0:
                    continue

                if geom.geom_type == "Polygon":
                    coords = list(geom.exterior.coords)
                elif geom.geom_type == "MultiPolygon":
                    coords = list(geom.geoms[0].exterior.coords)
                else:
                    continue

                # Konwertuj do lokalnych metrów
                bx, by = [], []
                for lon_v, lat_v in coords:
                    x, y = wgs84_to_local(lat_v, lon_v, self.transformer)
                    bx.append(x)
                    by.append(y)

                if len(bx) == 0:
                    continue

                bx_min = min(bx) - avoid
                bx_max = max(bx) + avoid
                by_min = min(by) - avoid
                by_max = max(by) + avoid
                bz_max = h + avoid

                # Indeksy w gridzie
                ix_min = max(0, int((bx_min - x_min) / res))
                ix_max = min(nx - 1, int((bx_max - x_min) / res))
                iy_min = max(0, int((by_min - y_min) / res))
                iy_max = min(ny - 1, int((by_max - y_min) / res))
                iz_max = min(nz - 1, int(bz_max / res))

                if ix_min > ix_max or iy_min > iy_max or iz_max < 0:
                    continue

                grid[ix_min:ix_max + 1, iy_min:iy_max + 1, 0:iz_max + 1] = 1

            except Exception as e:
                print(f"  ⚠️ Błąd budynku {idx}: {e}")
                continue

        occupied = np.sum(grid)
        total = grid.size
        print(f"  Komórki zajęte: {occupied:,} / {total:,} ({100 * occupied / total:.1f}%)")

        return grid, (x_min, x_max, y_min, y_max, z_min, z_max), (xa, ya, xb, yb)

    def _astar_3d(self, grid, start_idx, goal_idx, bounds):
        """A* 3D z preferencją ruchu poziomego"""
        nx, ny, nz = grid.shape
        x_min, x_max, y_min, y_max, z_min, z_max = bounds

        def heuristic(a, b):
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            dz = abs(a[2] - b[2])
            return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        def get_neighbors(node):
            x, y, z = node
            neighbors = []
            # Preferuj ruch poziomy (dx, dy) nad pionowy (dz)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in [-1, 0, 1]:
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        nx_new = x + dx
                        ny_new = y + dy
                        nz_new = z + dz
                        if 0 <= nx_new < nx and 0 <= ny_new < ny and 0 <= nz_new < nz:
                            if grid[nx_new, ny_new, nz_new] == 0:
                                # Koszt: preferuj ruch poziomy (dz droższy)
                                if dz != 0:
                                    cost = 1.5 * np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
                                else:
                                    cost = np.sqrt(dx ** 2 + dy ** 2)
                                neighbors.append(((nx_new, ny_new, nz_new), cost))
            return neighbors

        open_set = []
        heappush(open_set, (0, start_idx))
        came_from = {}
        g_score = {start_idx: 0}
        f_score = {start_idx: heuristic(start_idx, goal_idx)}

        visited = 0
        while open_set:
            visited += 1
            _, current = heappop(open_set)

            if current == goal_idx:
                print(f"  ✓ Znaleziono ścieżkę (odwiedzone węzły: {visited:,})")
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_idx)
                return path[::-1]

            for neighbor, cost in get_neighbors(current):
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal_idx)
                    heappush(open_set, (f_score[neighbor], neighbor))

        print("  ✗ Nie znaleziono ścieżki")
        return None

    def plan(self, point_a, point_b, show_plots=True):
        """
        GŁÓWNA FUNKCJA - POPRAWIONA:
        1. Start na altitude_start
        2. Wznoszenie do altitude_cruise_min
        3. Przelot A* na cruise altitude z omijaniem
        4. Opadanie do altitude_end
        """
        print("\n" + "=" * 80)
        print("PLANOWANIE TRASY 3D Z SEGMENTAMI START/CRUISE/END")
        print("=" * 80)

        # Parametry
        alt_start = self.config.altitude_start
        alt_cruise_min = self.config.altitude_cruise_min
        alt_cruise_max = self.config.altitude_cruise_max
        alt_end = self.config.altitude_end
        res = self.config.grid_resolution

        print(f"\nWysokości:")
        print(f"  Start: {alt_start:.1f} m AGL")
        print(f"  Cruise: {alt_cruise_min:.1f} - {alt_cruise_max:.1f} m AGL")
        print(f"  End: {alt_end:.1f} m AGL")

        # Domyślne punkty jeśli None
        if point_a is None or point_b is None:
            point_a = (52.2297, 21.0122)  # Plac Zamkowy
            point_b = (52.2330, 21.0180)  # Katedra
            print(f"\n⚠️ Używam domyślnych punktów:")
            print(f"  A: {point_a}")
            print(f"  B: {point_b}")

        # Wczytaj budynki i utwórz grid
        self._load_buildings()
        grid, bounds, (xa, ya, xb, yb) = self._create_3d_grid(point_a, point_b)
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        nx, ny, nz = grid.shape

        # === PROBLEM 3: WALIDACJA WYSOKOŚCI STARTOWEJ ===
        ix_a = int((xa - x_min) / res)
        iy_a = int((ya - y_min) / res)
        iz_start = int(alt_start / res)
        iz_start = np.clip(iz_start, 0, nz - 1)

        if grid[ix_a, iy_a, iz_start] == 1:
            print(f"⚠️ UWAGA: Punkt startowy na {alt_start:.1f}m w przeszkodzie!")
            # Znajdź najbliższą wolną wysokość
            for dz in range(0, 30):
                iz_test = int((alt_start + dz * res) / res)
                if iz_test < nz and grid[ix_a, iy_a, iz_test] == 0:
                    alt_start = iz_test * res
                    print(f"  ✓ Skorygowano wysokość startu: {alt_start:.1f}m")
                    break

        # === SEGMENT 1: START (wznoszenie od alt_start do alt_cruise_min W LOCIE) ===
        print(f"\n[SEGMENT 1] START - wznoszenie {alt_start:.1f} -> {alt_cruise_min:.1f} m (w locie)")

        # Pozycja startowa (A)
        z_start_agl = alt_start
        z_cruise_agl = alt_cruise_min

        # PROBLEM 2: Wznoszenie podczas ruchu poziomego
        climb_distance = 30.0  # 30m ruchu poziomego podczas wznoszenia
        n_climb = max(10, int((alt_cruise_min - alt_start) / res))

        # Kierunek do punktu B
        dx_ab = xb - xa
        dy_ab = yb - ya
        dist_ab = np.sqrt(dx_ab ** 2 + dy_ab ** 2)

        if dist_ab > 0:
            # Znormalizowany kierunek
            dir_x = dx_ab / dist_ab
            dir_y = dy_ab / dist_ab
            # Ruch w kierunku B podczas wznoszenia
            x_climb = np.linspace(xa, xa + climb_distance * dir_x, n_climb)
            y_climb = np.linspace(ya, ya + climb_distance * dir_y, n_climb)
        else:
            # Jeśli A i B w tym samym miejscu, wznoś pionowo
            x_climb = np.linspace(xa, xa, n_climb)
            y_climb = np.linspace(ya, ya, n_climb)

        z_climb_agl = np.linspace(z_start_agl, z_cruise_agl, n_climb)

        # === SEGMENT 2: CRUISE (przelot A* na wysokości cruise) ===
        print(f"\n[SEGMENT 2] CRUISE - przelot A -> B na ~{z_cruise_agl:.1f} m")

        # Start A* od pozycji po wzniesieniu (koniec segmentu climb)
        xa_after_climb = x_climb[-1]
        ya_after_climb = y_climb[-1]

        ix_a = int((xa_after_climb - x_min) / res)
        iy_a = int((ya_after_climb - y_min) / res)
        iz_cruise = int(z_cruise_agl / res)
        iz_cruise = np.clip(iz_cruise, 0, nz - 1)

        # Sprawdź czy punkt startowy wolny
        if grid[ix_a, iy_a, iz_cruise] == 1:
            # Znajdź najbliższy wolny punkt
            for dz in range(-5, 10):
                iz_test = iz_cruise + dz
                if 0 <= iz_test < nz and grid[ix_a, iy_a, iz_test] == 0:
                    iz_cruise = iz_test
                    z_cruise_agl = iz_cruise * res
                    print(f"  ⚠️ Skorygowano wysokość cruise: {z_cruise_agl:.1f} m")
                    break

        # Cel B na tej samej wysokości
        ix_b = int((xb - x_min) / res)
        iy_b = int((yb - y_min) / res)
        iz_b = iz_cruise

        # Sprawdź cel
        if grid[ix_b, iy_b, iz_b] == 1:
            for dz in range(-5, 10):
                iz_test = iz_b + dz
                if 0 <= iz_test < nz and grid[ix_b, iy_b, iz_test] == 0:
                    iz_b = iz_test
                    break

        start_idx = (ix_a, iy_a, iz_cruise)
        goal_idx = (ix_b, iy_b, iz_b)

        print(f"  Start: grid{start_idx} -> ({xa_after_climb:.1f}, {ya_after_climb:.1f}, {z_cruise_agl:.1f})")
        print(f"  Cel:   grid{goal_idx} -> ({xb:.1f}, {yb:.1f}, {iz_b * res:.1f})")

        path_indices = self._astar_3d(grid, start_idx, goal_idx, bounds)

        if path_indices is None:
            print("✗ A* nie znalazł ścieżki - używam linii prostej")
            n_direct = 50
            x_cruise = np.linspace(xa_after_climb, xb, n_direct)
            y_cruise = np.linspace(ya_after_climb, yb, n_direct)
            z_cruise_agl_arr = np.full(n_direct, z_cruise_agl)
        else:
            # Konwertuj indeksy na metry
            x_cruise = []
            y_cruise = []
            z_cruise_grid = []
            for ix, iy, iz in path_indices:
                x = x_min + ix * res
                y = y_min + iy * res
                z_agl = iz * res
                x_cruise.append(x)
                y_cruise.append(y)
                z_cruise_grid.append(z_agl)
            x_cruise = np.array(x_cruise)
            y_cruise = np.array(y_cruise)
            z_cruise_grid = np.array(z_cruise_grid)

            # POPRAWKA: Jeśli wysokość cruise jest mniejsza niż rozdzielczość gridu,
            # A* leci na wysokości 0m (iz=0). Musimy przeskalować do rzeczywistej wysokości.
            z_cruise_agl_arr = np.where(z_cruise_grid == 0, z_cruise_agl, z_cruise_grid)

            print(f"  ✓ Znaleziono ścieżkę (długość: {len(path_indices)} punktów)")
            if z_cruise_grid.mean() != z_cruise_agl_arr.mean():
                print(f"  ✓ Przeskalowano wysokość z {z_cruise_grid.mean():.1f}m do {z_cruise_agl_arr.mean():.1f}m")

        # === SEGMENT 3: END (opadanie do alt_end W LOCIE) ===
        print(f"\n[SEGMENT 3] END - opadanie {z_cruise_agl:.1f} -> {alt_end:.1f} m (w locie)")

        z_final_cruise = z_cruise_agl_arr[-1]
        n_descent = max(10, int((z_final_cruise - alt_end) / res))

        # PROBLEM 2: Opadanie podczas ruchu poziomego
        descent_distance = 30.0  # 30m ruchu poziomego podczas opadania

        # Pozycja końca cruise
        x_cruise_end = x_cruise[-1]
        y_cruise_end = y_cruise[-1]

        # Kierunek do punktu B (jeśli jeszcze nie doszliśmy)
        dx_to_b = xb - x_cruise_end
        dy_to_b = yb - y_cruise_end
        dist_to_b = np.sqrt(dx_to_b ** 2 + dy_to_b ** 2)

        if dist_to_b > 5.0:  # Jeśli jesteśmy dalej niż 5m od B
            dir_x = dx_to_b / dist_to_b
            dir_y = dy_to_b / dist_to_b
            # Kontynuuj ruch w kierunku B
            x_descent = np.linspace(x_cruise_end, x_cruise_end + descent_distance * dir_x, n_descent)
            y_descent = np.linspace(y_cruise_end, y_cruise_end + descent_distance * dir_y, n_descent)
        else:
            # Jesteśmy blisko B, opadaj w miejscu
            x_descent = np.linspace(x_cruise_end, xb, n_descent)
            y_descent = np.linspace(y_cruise_end, yb, n_descent)

        z_descent_agl = np.linspace(z_final_cruise, alt_end, n_descent)

        # === POŁĄCZ SEGMENTY ===
        X_full = np.concatenate([x_climb, x_cruise, x_descent])
        Y_full = np.concatenate([y_climb, y_cruise, y_descent])
        Z_full_agl = np.concatenate([z_climb_agl, z_cruise_agl_arr, z_descent_agl])

        # PROBLEM 1: Oblicz dystans kumulatywny wzdłuż trasy
        dx = np.diff(X_full)
        dy = np.diff(Y_full)
        dz = np.diff(Z_full_agl)
        ds = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        s_full = np.concatenate([[0.0], np.cumsum(ds)])  # Dystans od startu (s >= 0)

        # NED: Z ujemne w górę
        Z_full_ned = -Z_full_agl

        print(f"\n[Trasa końcowa]")
        print(f"  Punkty: {len(X_full)}")
        print(f"  Dystans całkowity: {s_full[-1]:.1f} m")
        print(f"  Z (AGL): {Z_full_agl.min():.1f} -> {Z_full_agl.max():.1f} m")

        # Zapisz do pliku - PROBLEM 1: s_ref zamiast X_ref jako główna współrzędna
        np.savez("planned_path.npz",
                 s_ref=s_full,  # Dystans wzdłuż trasy (oś X na wykresach)
                 Y_ref=Y_full,  # Współrzędna Y (odchylenie boczne)
                 Z_ref=Z_full_ned,  # Wysokość NED
                 X_ref=X_full)  # Zachowaj dla kompatybilności/mapy
        print(f"✓ Zapisano: planned_path.npz (dystans: 0 -> {s_full[-1]:.1f}m)")

        # Konwertuj do lat/lon dla animacji
        path_latlon = []
        for x, y, z_agl in zip(X_full, Y_full, Z_full_agl):
            lat, lon = local_to_wgs84(x, y, self.transformer)
            path_latlon.append((lat, lon, z_agl))

        # Wizualizacja
        if show_plots:
            self._plot_3d_path(X_full, Y_full, Z_full_agl, s_full, xa, ya, xb, yb, bounds, grid)

        return {
            "path_latlon": path_latlon,
            "s_ref": s_full,  # Dystans wzdłuż trasy
            "X_ref": X_full,  # Współrzędna X
            "Y_ref": Y_full,  # Współrzędna Y
            "Z_ref": Z_full_ned,  # Wysokość NED
            "Z_agl": Z_full_agl  # Wysokość AGL
        }

    def _plot_3d_path(self, X, Y, Z_agl, s, xa, ya, xb, yb, bounds, grid):
        """Wizualizacja: 3D + widok z góry + profil wysokości"""

        # Wczytaj budynki w obszarze
        buildings_in = None
        if self.buildings_gdf is not None:
            x_min, x_max, y_min, y_max, z_min, z_max = bounds
            margin = 50.0
            try:
                lat_min, lon_min = local_to_wgs84(x_min - margin, y_min - margin, self.transformer)
                lat_max, lon_max = local_to_wgs84(x_max + margin, y_max + margin, self.transformer)

                buildings_in = self.buildings_gdf.cx[
                               min(lon_min, lon_max):max(lon_min, lon_max),
                               min(lat_min, lat_max):max(lat_min, lat_max)
                               ]
                print(f"[Wykresy] Budynki w obszarze: {len(buildings_in)}")
            except Exception as e:
                print(f"[Wykresy] ⚠️ Błąd ładowania budynków: {e}")
                buildings_in = None

        # Oblicz kąt rotacji dla widoku z góry
        angle = -np.arctan2((yb - ya), (xb - xa))
        c, s_rot = np.cos(angle), np.sin(angle)

        # Rotuj trasę
        X_r = c * X - s_rot * Y
        Y_r = s_rot * X + c * Y
        xa_r = c * xa - s_rot * ya
        ya_r = s_rot * xa + c * ya
        xb_r = c * xb - s_rot * yb
        yb_r = s_rot * xb + c * yb

        # Konwersja X na dystans (przesunięcie tak aby A było w 0)
        X_r_dist = X_r - xa_r
        dist_total = s[-1]

        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], width_ratios=[1.2, 1.0])
        ax1 = fig.add_subplot(gs[:, 0], projection='3d')
        ax2 = fig.add_subplot(gs[0, 1]) # prawa górna
        ax3 = fig.add_subplot(gs[1, 1]) # prawa dolna

        # ===================================================================
        # Wykres 1: 3D z dystansem na osi X
        # ===================================================================

        ax1.plot(s, Y, Z_agl, 'r-', linewidth=3, label='Trasa')
        ax1.scatter([0], [ya], [self.config.altitude_start], c='g', s=150, marker='o', label='Start',
                    edgecolor='black', linewidth=2)
        ax1.scatter([dist_total], [yb], [self.config.altitude_end], c='r', s=150, marker='o', label='Cel',
                    edgecolor='black', linewidth=2)

        # ➕ Dodaj budynki 3D jako prostokąty/pudełka
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        # ➕ Dodaj budynki jako bryły 3D (ściany + dach)
        if buildings_in is not None and len(buildings_in) > 0:
            try:
                for _, row in buildings_in.iterrows():
                    geom = row.geometry
                    try:
                        h = estimate_building_height(row)
                        if h <= 0:
                            continue

                        if geom.geom_type == "Polygon":
                            coords = list(geom.exterior.coords)
                        elif geom.geom_type == "MultiPolygon":
                            coords = list(geom.geoms[0].exterior.coords)
                        else:
                            continue

                        bx, by = [], []
                        for lon_v, lat_v in coords:
                            x_loc, y_loc = wgs84_to_local(lat_v, lon_v, self.transformer)
                            bx.append(x_loc)
                            by.append(y_loc)

                        bx = np.array(bx)
                        by = np.array(by)

                        # Rotacja i przesunięcie na oś X jako dystans
                        bx_r = c * bx - s_rot * by
                        by_r = s_rot * bx + c * by
                        bx_r_dist = bx_r - xa_r

                        # Zamknij wielokąt jeśli nie jest zamknięty
                        if not (bx_r_dist[0] == bx_r_dist[-1] and by_r[0] == by_r[-1]):
                            bx_r_dist = np.append(bx_r_dist, bx_r_dist[0])
                            by_r = np.append(by_r, by_r[0])

                        # Buduj ściany
                        walls = []
                        for i in range(len(bx_r_dist) - 1):
                            x0, x1 = bx_r_dist[i], bx_r_dist[i + 1]
                            y0, y1 = by_r[i], by_r[i + 1]
                            wall = [
                                [x0, y0, 0],
                                [x1, y1, 0],
                                [x1, y1, h],
                                [x0, y0, h]
                            ]
                            walls.append(wall)

                        # Buduj dach
                        roof = [[bx_r_dist[i], by_r[i], h] for i in range(len(bx_r_dist))]

                        ax1.add_collection3d(Poly3DCollection(walls, facecolors='orange', edgecolors='gray',
                                                              linewidths=0.3, alpha=0.5, zorder=10))
                        ax1.add_collection3d(Poly3DCollection([roof], facecolors='orange', edgecolors='gray',
                                                              linewidths=0.5, alpha=0.6, zorder=15))
                    except Exception as e:
                        print(f"[3D Budynki] ⚠️ {e}")
                        continue
            except Exception as e:
                print(f"[3D Budynki] ⚠️ Błąd ogólny: {e}")

        ax1.set_xlabel('Dystans [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Wysokość [m]')
        ax1.set_title('Trasa 3D (dystans, Y, wysokość)')
        ax1.legend(loc='upper left')  # lub 'upper right' jak chcesz
        ax1.grid(True, alpha=0.3)

        # ===================================================================
        # Wykres 2: Y(dystans) - widok z góry z budynkami
        # ===================================================================
        # ===================================================================
        # Wykres 2: widok z góry w układzie lokalnym X-Y (bez rotacji)
        # ===================================================================
        ax2.set_facecolor("#f9f9f9")

        # Budynki w układzie lokalnym
        if buildings_in is not None and len(buildings_in) > 0:
            try:
                for _, row in buildings_in.iterrows():
                    try:
                        geom = row.geometry
                        if geom.geom_type == "Polygon":
                            coords = list(geom.exterior.coords)
                        elif geom.geom_type == "MultiPolygon":
                            coords = list(geom.geoms[0].exterior.coords)
                        else:
                            continue

                        bx, by = [], []
                        for lon_v, lat_v in coords:
                            x_loc, y_loc = wgs84_to_local(lat_v, lon_v, self.transformer)
                            bx.append(x_loc)
                            by.append(y_loc)

                        bx = np.array(bx)
                        by = np.array(by)

                        ax2.fill(bx, by,
                                 facecolor="orange",
                                 edgecolor="darkgray",
                                 alpha=0.5,
                                 linewidth=0.5,
                                 zorder=10)
                    except Exception:
                        continue
            except Exception:
                pass

        # Trasa w układzie lokalnym
        ax2.plot(X, Y, 'r-', linewidth=3,
                 label=f'Trasa ({dist_total:.0f} m)',
                 zorder=50)

        # Punkt startu i celu w lokalnym X-Y
        ax2.plot([xa], [ya], "o",
                 color='green',
                 markersize=14,
                 markeredgecolor="black",
                 markeredgewidth=2,
                 zorder=100)
        ax2.plot([xb], [yb], "o",
                 color='red',
                 markersize=14,
                 markeredgecolor="black",
                 markeredgewidth=2,
                 zorder=100)
        ax2.text(xa, ya, "  A", fontsize=13, fontweight="bold")
        ax2.text(xb, yb, "  B", fontsize=13, fontweight="bold")

        ax2.set_xlabel("X [m]")
        ax2.set_ylabel("Y [m]")
        ax2.set_title("Widok z góry: układ lokalny X–Y")
        ax2.grid(True, alpha=0.3, linestyle="--")
        ax2.legend()

        # sensowne granice osi + równy skale
        margin_xy = 20.0
        ax2.set_xlim(
            min(X.min(), xa, xb) - margin_xy,
            max(X.max(), xa, xb) + margin_xy
        )
        ax2.set_ylim(
            min(Y.min(), ya, yb) - margin_xy,
            max(Y.max(), ya, yb) + margin_xy
        )
        ax2.set_aspect('equal', adjustable='box')

        # ===================================================================
        # Wykres 3: Z(dystans) - profil wysokości z zabudową
        # ===================================================================
        ax3.set_facecolor("#f9f9f9")

        # Zabudowa pod trasą
        if buildings_in is not None and len(buildings_in) > 0:
            try:
                from shapely.geometry import LineString
                # Utwórz linię trasy w lat/lon
                route_coords = []
                for x_loc, y_loc in zip(X, Y):
                    lat, lon = local_to_wgs84(x_loc, y_loc, self.transformer)
                    route_coords.append((lon, lat))
                route_line = LineString(route_coords)
                buf = 15.0 / 111000.0  # 15m buffer
                rbuf = route_line.buffer(buf)

                for _, row in buildings_in.iterrows():
                    geom = row.geometry
                    if geom is None or not geom.intersects(rbuf):
                        continue
                    try:
                        h = estimate_building_height(row)

                        if geom.geom_type == "Polygon":
                            coords = list(geom.exterior.coords)
                        elif geom.geom_type == "MultiPolygon":
                            coords = list(geom.geoms[0].exterior.coords)
                        else:
                            continue

                        bx, by = [], []
                        for lon_v, lat_v in coords:
                            x_loc, y_loc = wgs84_to_local(lat_v, lon_v, self.transformer)
                            bx.append(x_loc)
                            by.append(y_loc)

                        bx = np.array(bx)
                        by = np.array(by)
                        # Rotacja
                        bx_r = c * bx - s_rot * by
                        # Konwersja na dystans
                        bx_r_dist = bx_r - xa_r

                        if len(bx_r_dist) > 0:
                            w = max(float(np.max(bx_r_dist) - np.min(bx_r_dist)), 3.0)
                            xc = 0.5 * (float(np.max(bx_r_dist)) + float(np.min(bx_r_dist)))
                            rect = plt.Rectangle((xc - w / 2, 0), w, h,
                                                 facecolor="orange", edgecolor="darkgray",
                                                 alpha=0.6, linewidth=1, zorder=30)
                            ax3.add_patch(rect)
                    except:
                        continue
            except Exception as e:
                print(f"[Profil wysokości] ⚠️ Błąd: {e}")

        # Trasa
        ax3.plot(s, Z_agl, 'r-', linewidth=3, label='Wysokość trasy', zorder=100)
        ax3.set_xlabel('Dystans [m]')
        ax3.set_ylabel('Wysokość AGL [m]')
        ax3.set_title('Profil wysokości Z(dystans)')
        ax3.grid(True, alpha=0.3, linestyle="--")
        ax3.legend(loc='upper right')
        ax3.set_xlim(-50, dist_total + 50)
        ax3.set_ylim(bottom=-2)

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    # Test
    from config_parameters import input_parameters
    from config import config

    input_parameters()
    planner = DroneRoutePlanner(config)
    result = planner.plan(None, None, show_plots=True)