import numpy as np
import pandas as pd
from pathlib import Path
from heapq import heappush, heappop
import matplotlib.pyplot as plt
from geo_utils import create_transformer, wgs84_to_local, local_to_wgs84, estimate_building_height
import time

DATA_DIR = Path("data")
BUILDINGS_FILE = DATA_DIR / "warsaw_buildings.pkl"

COLORS = {
    "building_face": "#FFA500",
    "building_edge": "darkgray",
    "track": "red",
    "drone": "blue",
}


class DroneRoutePlanner:
    def __init__(self, config):
        self.config = config
        self.buildings_gdf = None
        self.grid_3d = None
        self.ref_lat = None
        self.ref_lon = None
        self.transformer = None
        self.buildings_in_area = None

    @staticmethod
    def latlon_to_meters(lat, lon, alt, ref_lat, ref_lon):
        transformer = create_transformer(ref_lat, ref_lon)
        x, y = wgs84_to_local(lat, lon, transformer)
        z = -alt  # NED: Z ujemne w górę
        return x, y, z

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

    def _create_3d_grid(self, point_a, point_b, margin=200.0, avoid_distance=None):
        """Tworzy siatkę 3D z przeszkodami"""
        lat_a, lon_a = point_a
        lat_b, lon_b = point_b

        self.ref_lat = (lat_a + lat_b) / 2
        self.ref_lon = (lon_a + lon_b) / 2
        self.transformer = create_transformer(self.ref_lat, self.ref_lon)

        xa, ya = wgs84_to_local(lat_a, lon_a, self.transformer)
        xb, yb = wgs84_to_local(lat_b, lon_b, self.transformer)

        x_min = min(xa, xb) - margin
        x_max = max(xa, xb) + margin
        y_min = min(ya, yb) - margin
        y_max = max(ya, yb) + margin

        res = self.config.grid_resolution

        nx = int((x_max - x_min) / res) + 1
        ny = int((y_max - y_min) / res) + 1

        z_min = 0.0
        z_max = self.config.altitude_cruise_max + self.config.avoid_distance + 50  # Dodatkowy zapas
        nz = int(z_max / res) + 1

        print(f"\n[Grid] Wymiary: {nx} x {ny} x {nz}")
        print(f"  X: {x_min:.1f} -> {x_max:.1f} m")
        print(f"  Y: {y_min:.1f} -> {y_max:.1f} m")
        print(f"  Z: {z_min:.1f} -> {z_max:.1f} m (AGL)")
        print(f"  Rozdzielczość: {res:.1f} m")

        grid = np.zeros((nx, ny, nz), dtype=np.uint8)

        if self.buildings_gdf is None or len(self.buildings_gdf) == 0:
            print("  ⚠️ Brak budynków - grid pusty")
            return grid, (x_min, x_max, y_min, y_max, z_min, z_max), (xa, ya, xb, yb), res

        # Filtr budynków w obszarze
        try:
            lat_min, lon_min = local_to_wgs84(x_min - 50, y_min - 50, self.transformer)
            lat_max, lon_max = local_to_wgs84(x_max + 50, y_max + 50, self.transformer)

            buildings_in = self.buildings_gdf.cx[
                min(lon_min, lon_max):max(lon_min, lon_max),
                min(lat_min, lat_max):max(lat_min, lat_max)
            ]
        except:
            buildings_in = self.buildings_gdf

        self.buildings_in_area = buildings_in
        print(f"  Budynki w obszarze: {len(buildings_in)}")

        # Użyj podanego avoid_distance lub domyślnego
        avoid = avoid_distance if avoid_distance is not None else self.config.avoid_distance
        print(f"  Avoid distance: {avoid:.1f} m")

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

                ix_min = max(0, int((bx_min - x_min) / res))
                ix_max = min(nx - 1, int((bx_max - x_min) / res))
                iy_min = max(0, int((by_min - y_min) / res))
                iy_max = min(ny - 1, int((by_max - y_min) / res))
                iz_max = min(nz - 1, int(bz_max / res))

                if ix_min > ix_max or iy_min > iy_max or iz_max < 0:
                    continue

                grid[ix_min:ix_max + 1, iy_min:iy_max + 1, 0:iz_max + 1] = 1

            except:
                continue

        occupied = np.sum(grid)
        total = grid.size
        print(f"  Komórki zajęte: {occupied:,} / {total:,} ({100 * occupied / total:.1f}%)")

        return grid, (x_min, x_max, y_min, y_max, z_min, z_max), (xa, ya, xb, yb), res

    def _check_line_collision(self, p1, p2, grid, bounds, resolution):
        """Sprawdza czy prosta linia między p1 i p2 koliduje z przeszkodami - Bresenham 3D."""
        x_min, x_max, y_min, y_max, z_min, z_max = bounds
        nx, ny, nz = grid.shape

        x1, y1, z1 = p1
        x2, y2, z2 = p2

        # Konwersja do indeksów
        ix1 = int((x1 - x_min) / resolution)
        iy1 = int((y1 - y_min) / resolution)
        iz1 = int(z1 / resolution)

        ix2 = int((x2 - x_min) / resolution)
        iy2 = int((y2 - y_min) / resolution)
        iz2 = int(z2 / resolution)

        # Bresenham 3D
        dx = abs(ix2 - ix1)
        dy = abs(iy2 - iy1)
        dz = abs(iz2 - iz1)

        sx = 1 if ix2 > ix1 else -1
        sy = 1 if iy2 > iy1 else -1
        sz = 1 if iz2 > iz1 else -1

        # Dominująca oś
        if dx >= dy and dx >= dz:
            err_y = 2 * dy - dx
            err_z = 2 * dz - dx
            ix, iy, iz = ix1, iy1, iz1

            for _ in range(dx + 1):
                if ix < 0 or ix >= nx or iy < 0 or iy >= ny or iz < 0 or iz >= nz:
                    return True
                if grid[ix, iy, iz] == 1:
                    return True

                if err_y > 0:
                    iy += sy
                    err_y -= 2 * dx
                if err_z > 0:
                    iz += sz
                    err_z -= 2 * dx

                err_y += 2 * dy
                err_z += 2 * dz
                ix += sx

        elif dy >= dx and dy >= dz:
            err_x = 2 * dx - dy
            err_z = 2 * dz - dy
            ix, iy, iz = ix1, iy1, iz1

            for _ in range(dy + 1):
                if ix < 0 or ix >= nx or iy < 0 or iy >= ny or iz < 0 or iz >= nz:
                    return True
                if grid[ix, iy, iz] == 1:
                    return True

                if err_x > 0:
                    ix += sx
                    err_x -= 2 * dy
                if err_z > 0:
                    iz += sz
                    err_z -= 2 * dy

                err_x += 2 * dx
                err_z += 2 * dz
                iy += sy
        else:
            err_x = 2 * dx - dz
            err_y = 2 * dy - dz
            ix, iy, iz = ix1, iy1, iz1

            for _ in range(dz + 1):
                if ix < 0 or ix >= nx or iy < 0 or iy >= ny or iz < 0 or iz >= nz:
                    return True
                if grid[ix, iy, iz] == 1:
                    return True

                if err_x > 0:
                    ix += sx
                    err_x -= 2 * dz
                if err_y > 0:
                    iy += sy
                    err_y -= 2 * dz

                err_x += 2 * dx
                err_y += 2 * dy
                iz += sz

        return False

    def _astar_3d_enhanced(self, grid, start_idx, goal_idx, bounds):
        """
        Ulepszony A* 3D:
        - 18 kierunków (6 głównych + 12 ukośnych)
        - Dynamiczny cylinder
        - Adaptacyjne koszty
        """
        t_start = time.time()

        nx, ny, nz = grid.shape
        x_min, x_max, y_min, y_max, z_min, z_max = bounds

        def heuristic(a, b):
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            dz = abs(a[2] - b[2])
            return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # Dynamiczny cylinder - szerszy dla trudniejszych przypadków
        base_dist = heuristic(start_idx, goal_idx)
        cylinder_radius = max(100, base_dist * 0.5)  # 50% szerokości

        def in_search_space(node):
            """Sprawdź czy węzeł jest w cylindrze"""
            sx, sy, sz = start_idx
            gx, gy, gz = goal_idx
            nx_node, ny_node, nz_node = node

            dx_line = gx - sx
            dy_line = gy - sy
            dz_line = gz - sz
            line_len_sq = dx_line ** 2 + dy_line ** 2 + dz_line ** 2

            if line_len_sq < 1:
                return True

            dx_node = nx_node - sx
            dy_node = ny_node - sy
            dz_node = nz_node - sz

            t = max(0, min(1, (dx_node * dx_line + dy_node * dy_line + dz_node * dz_line) / line_len_sq))

            closest_x = sx + t * dx_line
            closest_y = sy + t * dy_line
            closest_z = sz + t * dz_line

            dist = np.sqrt((nx_node - closest_x) ** 2 +
                           (ny_node - closest_y) ** 2 +
                           (nz_node - closest_z) ** 2)

            return dist <= cylinder_radius

        def get_neighbors(node):
            """18 kierunków: 6 głównych + 12 ukośnych"""
            x, y, z = node
            neighbors = []

            # 6 głównych kierunków (koszt 1.0)
            for dx, dy, dz in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]:
                nx_new = x + dx
                ny_new = y + dy
                nz_new = z + dz

                if 0 <= nx_new < nx and 0 <= ny_new < ny and 0 <= nz_new < nz:
                    if grid[nx_new, ny_new, nz_new] == 0:
                        neighbor = (nx_new, ny_new, nz_new)
                        if in_search_space(neighbor):
                            neighbors.append((neighbor, 1.0))

            # 12 ukośnych kierunków (koszt sqrt(2) ≈ 1.414)
            diagonal_dirs = [
                (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),  # poziome ukośne
                (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),  # XZ ukośne
                (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1)  # YZ ukośne
            ]

            for dx, dy, dz in diagonal_dirs:
                nx_new = x + dx
                ny_new = y + dy
                nz_new = z + dz

                if 0 <= nx_new < nx and 0 <= ny_new < ny and 0 <= nz_new < nz:
                    if grid[nx_new, ny_new, nz_new] == 0:
                        neighbor = (nx_new, ny_new, nz_new)
                        if in_search_space(neighbor):
                            # Preferuj ruch poziomy (mniejszy koszt dla ruchu w XY)
                            if dz == 0:
                                cost = 1.414
                            else:
                                cost = 1.732  # sqrt(3) dla ruchu z pionową składową
                            neighbors.append((neighbor, cost))

            return neighbors

        print(f"\n[A* 3D Enhanced]")
        print(f"  Przestrzeń: cylinder r={cylinder_radius:.1f} komórek")
        print(f"  Sąsiedzi: 18 kierunków")

        open_set = []
        heappush(open_set, (0, start_idx))
        came_from = {}
        g_score = {start_idx: 0}
        f_score = {start_idx: heuristic(start_idx, goal_idx)}

        nodes_explored = 0
        max_open_size = 0

        while open_set:
            max_open_size = max(max_open_size, len(open_set))
            _, current = heappop(open_set)
            nodes_explored += 1

            if nodes_explored % 2000 == 0:
                print(f"  Przeszukano: {nodes_explored:,} węzłów...")

            if current == goal_idx:
                # Rekonstrukcja ścieżki
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_idx)
                path.reverse()

                t_elapsed = time.time() - t_start
                print(f"\n✓ Znaleziono ścieżkę!")
                print(f"  Czas: {t_elapsed:.2f}s")
                print(f"  Węzły przeszukane: {nodes_explored:,}")
                print(f"  Długość ścieżki: {len(path)} punktów")

                return path

            for neighbor, cost in get_neighbors(current):
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal_idx)
                    heappush(open_set, (f_score[neighbor], neighbor))

        t_elapsed = time.time() - t_start
        print(f"\n✗ Nie znaleziono ścieżki")
        print(f"  Czas: {t_elapsed:.2f}s")
        print(f"  Węzły przeszukane: {nodes_explored:,}")

        return None

    def _smooth_path_line_of_sight(self, path_indices, grid, bounds, resolution):
        """Wygładza ścieżkę używając line-of-sight shortcutting."""
        if len(path_indices) <= 2:
            return path_indices

        t_start = time.time()
        x_min, x_max, y_min, y_max, z_min, z_max = bounds

        def idx_to_meters(idx):
            ix, iy, iz = idx
            x = x_min + ix * resolution
            y = y_min + iy * resolution
            z = iz * resolution
            return (x, y, z)

        def meters_to_idx(coord):
            x, y, z = coord
            ix = int((x - x_min) / resolution)
            iy = int((y - y_min) / resolution)
            iz = int(z / resolution)
            return (ix, iy, iz)

        path_meters = [idx_to_meters(idx) for idx in path_indices]

        smoothed = [path_meters[0]]
        current_idx = 0

        while current_idx < len(path_meters) - 1:
            farthest_idx = current_idx + 1

            for test_idx in range(len(path_meters) - 1, current_idx, -1):
                if not self._check_line_collision(
                        path_meters[current_idx],
                        path_meters[test_idx],
                        grid, bounds, resolution
                ):
                    farthest_idx = test_idx
                    break

            smoothed.append(path_meters[farthest_idx])
            current_idx = farthest_idx

        smoothed_indices = [meters_to_idx(coord) for coord in smoothed]

        t_elapsed = time.time() - t_start
        print(f"\n[Path Smoothing]")
        print(f"  Czas: {t_elapsed:.2f}s")
        print(f"  Punkty przed: {len(path_indices)}, po: {len(smoothed_indices)}")
        print(f"  Redukcja: {100 * (1 - len(smoothed_indices) / len(path_indices)):.1f}%")

        return smoothed_indices

    def _add_vertical_segments(self, smoothed_path, start_alt, cruise_min, cruise_max, end_alt,
                               grid, bounds, resolution):
        """Dodaje pionowe segmenty: wzlot, lot, lądowanie."""
        x_min, x_max, y_min, y_max, z_min, z_max = bounds

        def idx_to_meters(idx):
            ix, iy, iz = idx
            x = x_min + ix * resolution
            y = y_min + iy * resolution
            z = iz * resolution
            return (x, y, z)

        start_pos = idx_to_meters(smoothed_path[0])
        end_pos = idx_to_meters(smoothed_path[-1])

        # WZLOT
        takeoff_waypoints = []
        if start_alt < cruise_min:
            x_s, y_s, _ = start_pos
            num_steps = max(int((cruise_min - start_alt) / resolution) + 1, 3)

            for i in range(num_steps + 1):
                t = i / num_steps
                z = start_alt * (1 - t) + cruise_min * t
                ix = int((x_s - x_min) / resolution)
                iy = int((y_s - y_min) / resolution)
                iz = int(z / resolution)

                if 0 <= ix < grid.shape[0] and 0 <= iy < grid.shape[1] and 0 <= iz < grid.shape[2]:
                    takeoff_waypoints.append((ix, iy, iz))

        # LĄDOWANIE
        landing_waypoints = []
        if end_alt < cruise_min:
            x_e, y_e, _ = end_pos
            num_steps = max(int((cruise_min - end_alt) / resolution) + 1, 3)

            for i in range(num_steps + 1):
                t = i / num_steps
                z = cruise_min * (1 - t) + end_alt * t
                ix = int((x_e - x_min) / resolution)
                iy = int((y_e - y_min) / resolution)
                iz = int(z / resolution)

                if 0 <= ix < grid.shape[0] and 0 <= iy < grid.shape[1] and 0 <= iz < grid.shape[2]:
                    landing_waypoints.append((ix, iy, iz))

        full_path = takeoff_waypoints + smoothed_path + landing_waypoints

        return full_path

    def _is_point_in_real_building(self, x, y, z):
        """Sprawdza czy punkt (x,y,z) jest w RZECZYWISTYM budynku (bez avoid_distance)."""
        if self.buildings_in_area is None:
            return False

        from shapely.geometry import Point
        point_2d = Point(x, y)

        for idx, row in self.buildings_in_area.iterrows():
            try:
                geom = row.geometry
                if geom is None:
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

                from shapely.geometry import Polygon as ShapelyPolygon
                building_poly = ShapelyPolygon(zip(bx, by))

                if building_poly.contains(point_2d):
                    h = estimate_building_height(row)
                    if z <= h:
                        return True

            except:
                continue

        return False

    def plan(self, point_a=None, point_b=None, show_plots=False):
        """Główna funkcja planowania trasy - ULEPSZONA"""
        print("\n" + "=" * 70)
        print("PLANOWANIE TRASY DRONA - WERSJA ULEPSZONA")
        print("=" * 70)

        t_total_start = time.time()

        # Załaduj budynki
        t_start = time.time()
        self._load_buildings()
        print(f"⏱️  Czas ładowania budynków: {time.time() - t_start:.2f}s")

        if point_a is None:
            point_a = (self.config.lat_start, self.config.lon_start)
        if point_b is None:
            point_b = (self.config.lat_end, self.config.lon_end)

        lat_a, lon_a = point_a
        lat_b, lon_b = point_b

        self.ref_lat = (lat_a + lat_b) / 2
        self.ref_lon = (lon_a + lon_b) / 2
        self.transformer = create_transformer(self.ref_lat, self.ref_lon)

        xa, ya = wgs84_to_local(lat_a, lon_a, self.transformer)
        xb, yb = wgs84_to_local(lat_b, lon_b, self.transformer)

        # Sprawdź punkty PRZED tworzeniem gridu
        margin_check = 100.0
        x_min_check = min(xa, xb) - margin_check
        x_max_check = max(xa, xb) + margin_check
        y_min_check = min(ya, yb) - margin_check
        y_max_check = max(ya, yb) + margin_check

        try:
            lat_min, lon_min = local_to_wgs84(x_min_check, y_min_check, self.transformer)
            lat_max, lon_max = local_to_wgs84(x_max_check, y_max_check, self.transformer)
            self.buildings_in_area = self.buildings_gdf.cx[
                min(lon_min, lon_max):max(lon_min, lon_max),
                min(lat_min, lat_max):max(lat_min, lat_max)
            ]
        except:
            self.buildings_in_area = self.buildings_gdf

        cruise_alt = (self.config.altitude_cruise_min + self.config.altitude_cruise_max) / 2.0

        if self._is_point_in_real_building(xa, ya, cruise_alt):
            print(f"✗ Punkt A ({lat_a:.6f}, {lon_a:.6f}) jest WEWNĄTRZ budynku!")
            print(f"  Wybierz punkt na ulicy, placu lub w parku.")
            return None

        if self._is_point_in_real_building(xb, yb, cruise_alt):
            print(f"✗ Punkt B ({lat_b:.6f}, {lon_b:.6f}) jest WEWNĄTRZ budynku!")
            print(f"  Wybierz punkt na ulicy, placu lub w parku.")
            return None

        print("✓ Oba punkty poza budynkami - kontynuuję planowanie...")


        # STRATEGIA: Próbuj z różnymi parametrami
        strategies = [
            {"margin": 500, "altitude_offset": 0},  # Standardowa wysokość
            {"margin": 500, "altitude_offset": 10},  # Leć 10m wyżej
            {"margin": 500, "altitude_offset": 20},  # Leć 20m wyżej
            {"margin": 1000, "altitude_offset": 0},  # Większy obszar
            {"margin": 1000, "altitude_offset": 10},  # Większy obszar + wyżej
            {"margin": 1500, "altitude_offset": 0},  # Jeszcze większy obszar
            {"margin": 1500, "altitude_offset": 20},  # Maksymalnie
        ]

        path_raw = None

        for strategy_idx, strategy in enumerate(strategies):
            print(f"\n{'=' * 70}")
            print(f"STRATEGIA {strategy_idx + 1}/{len(strategies)}")
            print(f"  Margin: {strategy['margin']}m")
            print(f"  Avoid distance: {self.config.avoid_distance:.1f}m (stały)")
            print(f"  Altitude offset: +{strategy['altitude_offset']}m")
            if strategy['altitude_offset'] > 0:
                print(f"  → Wysokość lotu: {cruise_alt:.1f}m → {cruise_alt + strategy['altitude_offset']:.1f}m")
            print(f"{'=' * 70}")

            # Twórz grid - BEZ zmiany avoid_distance!
            t_start = time.time()
            grid, bounds, (xa, ya, xb, yb), res = self._create_3d_grid(
                point_a, point_b,
                margin=strategy['margin']
                # NIE przekazujemy avoid_distance - używa domyślnego z config
            )
            print(f"⏱️  Czas tworzenia gridu: {time.time() - t_start:.2f}s")

            x_min, x_max, y_min, y_max, z_min, z_max = bounds

            # Wysokości z offsetem
            start_alt = self.config.altitude_start
            cruise_min = self.config.altitude_cruise_min + strategy['altitude_offset']
            cruise_max = self.config.altitude_cruise_max + strategy['altitude_offset']
            end_alt = self.config.altitude_end

            cruise_alt = (cruise_min + cruise_max) / 2.0

            start_idx = (
                int((xa - x_min) / res),
                int((ya - y_min) / res),
                int(cruise_alt / res)
            )

            goal_idx = (
                int((xb - x_min) / res),
                int((yb - y_min) / res),
                int(cruise_alt / res)
            )

            print(f"\n[Planowanie ścieżki]")
            print(f"  Start: {start_idx} @ {cruise_alt:.1f}m AGL")
            print(f"  Cel:   {goal_idx} @ {cruise_alt:.1f}m AGL")

            nx, ny, nz = grid.shape

            # Sprawdź czy punkty w gridzie
            if not (0 <= start_idx[0] < nx and 0 <= start_idx[1] < ny and 0 <= start_idx[2] < nz):
                print("✗ Punkt startowy poza gridem!")
                continue

            if not (0 <= goal_idx[0] < nx and 0 <= goal_idx[1] < ny and 0 <= goal_idx[2] < nz):
                print("✗ Punkt docelowy poza gridem!")
                continue

            # NIE sprawdzamy czy punkty są w przeszkodach!
            # A* sam znajdzie drogę dookoła jeśli trzeba

            # A* ulepszony
            path_raw = self._astar_3d_enhanced(grid, start_idx, goal_idx, bounds)

            if path_raw is not None:
                print(f"\n✓ Znaleziono ścieżkę ze strategią {strategy_idx + 1}!")

                # Zapisz parametry użyte do znalezienia ścieżki
                self.used_cruise_min = cruise_min
                self.used_cruise_max = cruise_max
                break
            else:
                print(f"\n✗ Nie znaleziono ścieżki ze strategią {strategy_idx + 1}")

        if path_raw is None:
            print("\n✗ Nie znaleziono ścieżki mimo prób z różnymi strategiami!")
            print("  Spróbuj wybrać inne punkty lub zwiększyć wysokość lotu w konfiguracji.")
            return None

        # Wygładź ścieżkę
        path_smoothed = self._smooth_path_line_of_sight(path_raw, grid, bounds, res)

        # Dodaj pionowe segmenty (używamy zapisane cruise_min/max)
        path_final = self._add_vertical_segments(
            path_smoothed, start_alt,
            self.used_cruise_min, self.used_cruise_max,
            end_alt, grid, bounds, res
        )

        # Konwertuj na współrzędne
        X, Y, Z = [], [], []
        for ix, iy, iz in path_final:
            x = x_min + ix * res
            y = y_min + iy * res
            z = iz * res
            X.append(x)
            Y.append(y)
            Z.append(z)

        X = np.array(X)
        Y = np.array(Y)
        Z = np.array(Z)

        segments = np.sqrt(np.diff(X) ** 2 + np.diff(Y) ** 2 + np.diff(Z) ** 2)
        total_dist = np.sum(segments)

        # Oblicz dystans wzdłuż trasy
        dx_total = xb - xa
        dy_total = yb - ya
        angle = -np.arctan2(dy_total, dx_total)
        c_rot, s_rot = np.cos(angle), np.sin(angle)
        X_r = c_rot * X - s_rot * Y
        xa_r = c_rot * xa - s_rot * ya
        s = X_r - xa_r

        t_total = time.time() - t_total_start

        print(f"\n" + "=" * 70)
        print(f"[WYNIK]")
        print(f"  ⏱️  Całkowity czas: {t_total:.2f}s")
        print(f"  📏 Długość trasy: {total_dist:.1f} m")
        print(f"  📍 Waypoints: {len(path_final)}")
        print(f"  ➡️  Odcinki: {len(path_final) - 1}")
        print("=" * 70)

        if show_plots:
            self._plot_3d_path_straight(X, Y, Z, s, xa, ya, xb, yb, bounds, grid)

        result = {
            "X": X,
            "Y": Y,
            "Z": Z,
            "s": s,
            "distance": total_dist,
            "waypoints": path_final,
            "num_segments": len(path_final) - 1,
            "computation_time": t_total,
            "xa": xa,
            "ya": ya,
            "xb": xb,
            "yb": yb,
            "bounds": bounds,
            "buildings_gdf": self.buildings_gdf,
            "transformer": self.transformer,
            "buildings_in_area": self.buildings_in_area
        }

        output_file = Path("planned_path.npz")
        np.savez(
            output_file,
            X_ref=X,
            Y_ref=Y,
            Z_ref=-Z,  # Konwersja AGL -> NED
            s_ref=s
        )
        print(f"✓ Zapisano trasę do: {output_file}")

        return result

    def _plot_3d_path_straight(self, X, Y, Z_agl, s, xa, ya, xb, yb, bounds, grid):
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
        xa_r = c * xa - s_rot * ya

        dist_total = s[-1]

        fig = plt.figure(figsize=(16, 8))
        gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], width_ratios=[1.2, 1.0])
        ax1 = fig.add_subplot(gs[:, 0], projection='3d')
        ax2 = fig.add_subplot(gs[0, 1])
        ax3 = fig.add_subplot(gs[1, 1])

        # ===================================================================
        # Wykres 1: 3D z dystansem na osi X
        # ===================================================================

        ax1.plot(X, Y, Z_agl, 'r-', linewidth=3, label='Trasa')

        ax1.scatter([xa], [ya], [self.config.altitude_start], c='g', s=150, marker='o', label='Start',
                    edgecolor='black', linewidth=2)
        ax1.scatter([xb], [yb], [self.config.altitude_end], c='r', s=150, marker='o', label='Cel',
                    edgecolor='black', linewidth=2)
        ax1.text(xa, ya, self.config.altitude_start, "  A", fontsize=13, fontweight="bold")
        ax1.text(xb, yb, self.config.altitude_start, "  B", fontsize=13, fontweight="bold")

        # Dodaj budynki 3D
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        if buildings_in is not None and len(buildings_in) > 0:
            try:
                from shapely.geometry import LineString
                route_coords = []
                for x_loc, y_loc in zip(X, Y):
                    lat, lon = local_to_wgs84(x_loc, y_loc, self.transformer)
                    route_coords.append((lon, lat))
                route_line = LineString(route_coords)
                buf = 30.0 / 111000.0
                rbuf = route_line.buffer(buf)

                for _, row in buildings_in.iterrows():
                    geom = row.geometry
                    if geom is None or not geom.intersects(rbuf):
                        continue

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

                        if not (bx[0] == bx[-1] and by[0] == by[-1]):
                            bx = np.append(bx, bx[0])
                            by = np.append(by, by[0])

                        walls = []
                        for i in range(len(bx) - 1):
                            x0, x1 = bx[i], bx[i + 1]
                            y0, y1 = by[i], by[i + 1]
                            wall = [
                                [x0, y0, 0],
                                [x1, y1, 0],
                                [x1, y1, h],
                                [x0, y0, h]
                            ]
                            walls.append(wall)

                        roof = [[bx[i], by[i], h] for i in range(len(bx))]

                        ax1.add_collection3d(Poly3DCollection(walls, facecolors='orange', edgecolors='gray',
                                                              linewidths=0.3, alpha=0.5, zorder=10))
                        ax1.add_collection3d(Poly3DCollection([roof], facecolors='orange', edgecolors='gray',
                                                              linewidths=0.5, alpha=0.6, zorder=15))
                    except:
                        continue
            except Exception as e:
                print(f"[3D Budynki] ⚠️ Błąd: {e}")

        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Wysokość [m]')
        ax1.set_title('Widok na zaplanowaną trasę')
        ax1.legend(loc='upper left')
        ax1.view_init(elev=90, azim=270)
        ax1.grid(True, alpha=0.3)

        # ===================================================================
        # Wykres 2: widok z góry w układzie lokalnym X-Y
        # ===================================================================
        ax2.set_facecolor("#f9f9f9")

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
                    except:
                        continue
            except:
                pass

        ax2.plot(X, Y, 'r-', linewidth=3,
                 label=f'Trasa ({dist_total:.0f} m)',
                 zorder=50)

        ax2.scatter([xa], [ya],
                    c='green', s=150, marker='o',
                    edgecolor="black", linewidth=2,
                    zorder=100)

        ax2.scatter([xb], [yb],
                    c='red', s=150, marker='o',
                    edgecolor="black", linewidth=2,
                    zorder=100)
        ax2.text(xa, ya, "  A", fontsize=13, fontweight="bold")
        ax2.text(xb, yb, "  B", fontsize=13, fontweight="bold")

        ax2.set_xlabel("X [m]")
        ax2.set_ylabel("Y [m]")
        ax2.set_title("Widok z góry (XY)")
        ax2.grid(True, alpha=0.3, linestyle="--")

        margin_xy = 100.0
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

        if buildings_in is not None and len(buildings_in) > 0:
            try:
                from shapely.geometry import LineString
                route_coords = []
                for x_loc, y_loc in zip(X, Y):
                    lat, lon = local_to_wgs84(x_loc, y_loc, self.transformer)
                    route_coords.append((lon, lat))
                route_line = LineString(route_coords)
                buf = 15.0 / 111000.0
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
                        bx_r = c * bx - s_rot * by
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

        ax3.plot(s, Z_agl, 'r-', linewidth=3, label='Wysokość trasy', zorder=100)
        ax3.scatter([0], [self.config.altitude_start],
                    c='green', s=80, marker='o',
                    edgecolor='black', linewidth=2,
                    zorder=110)
        ax3.text(0, self.config.altitude_start,
                 "  A", fontsize=12, fontweight="bold")

        ax3.scatter([dist_total], [self.config.altitude_end],
                    c='red', s=80, marker='o',
                    edgecolor='black', linewidth=2,
                    zorder=110)
        ax3.text(dist_total, self.config.altitude_end,
                 "  B", fontsize=12, fontweight="bold")

        ax3.set_xlabel('Dystans [m]')
        ax3.set_ylabel('Wysokość [m]')
        ax3.set_title('Profil wysokości')
        ax3.grid(True, alpha=0.3, linestyle="--")
        ax3.set_xlim(-50, dist_total + 50)
        ax3.set_ylim(bottom=-2)

        plt.tight_layout()
        plt.show()