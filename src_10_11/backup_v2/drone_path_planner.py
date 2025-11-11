"""
3D A* Path Planner for Drone Navigation with OSM Buildings
===========================================================
Planuje optymalną 3D ścieżkę od punktu A do B omijając budynki.

Zmiany vs. poprzednia wersja:
- Occupancy grid znakowany w UKŁADZIE METRYCZNYM (lokalne XY) z buforem avoid_distance.
- Szybka rasteryzacja: przygotowane geometrie (shapely.prepared) + bbox w gridzie.
- Wygładzanie B-spline z walidacją kolizji (fallback do ścieżki surowej / mniejsze wygładzenie).
"""

import heapq
import numpy as np
from scipy.interpolate import splprep, splev
from shapely.geometry import Point, Polygon, MultiPolygon, LinearRing
from shapely.prepared import prep
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - keep for 3D projection


class Node:
    """Węzeł w grafie A*."""

    def __init__(self, position, parent=None):
        self.position = position  # (i, j, k) w gridzie
        self.parent = parent
        self.g = 0.0  # koszt od startu
        self.h = 0.0  # heurystyka do celu
        self.f = 0.0  # f = g + h

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

    def __hash__(self):
        return hash(self.position)


class PathPlanner3D:
    """Planner 3D ścieżki z A* i omijaniem budynków."""

    def __init__(self, buildings, config):
        """
        Parameters
        ----------
        buildings : GeoDataFrame
            Budynki z OSM (geometry + wysokości/poziomy).
        config : DroneConfig
            Konfiguracja drona (grid_resolution, altitude_* , avoid_distance, itp.)
        """
        self.buildings = buildings
        self.config = config

        # Parametry gridu
        self.grid_resolution = config.grid_resolution
        self.altitude_min = config.altitude_cruise_min
        self.altitude_max = config.altitude_cruise_max
        self.avoid_distance = config.avoid_distance

        # Będą ustawione w create_occupancy_grid()
        self.occupancy_grid = None
        self.grid_origin = None  # (x_min, y_min, z_min)
        self.grid_shape = None   # (nx, ny, nz)

    # --- Konwersje WGS84 <-> lokalne metry ---

    def latlon_to_meters(self, lat, lon, alt, ref_lat, ref_lon):
        """Lat/lon -> metry względem punktu odniesienia."""
        y = (lat - ref_lat) * 111000.0
        x = (lon - ref_lon) * 111000.0 * np.cos(np.radians(ref_lat))
        z = alt
        return x, y, z

    def meters_to_latlon(self, x, y, z, ref_lat, ref_lon):
        """Metry -> lat/lon."""
        lat = ref_lat + y / 111000.0
        lon = ref_lon + x / (111000.0 * np.cos(np.radians(ref_lat)))
        alt = z
        return lat, lon, alt

    # --- Wysokość budynku ---

    def estimate_building_height(self, building):
        """Szacowanie wysokości budynku (m) z kolumn OSM."""
        height = None

        # 1) height
        try:
            if 'height' in building.index:
                val = building['height']
                if val is not None and str(val).strip() != '':
                    h = str(val).replace('m', '').replace('M', '').strip()
                    height = float(h)
        except Exception:
            pass

        # 2) building:levels
        if height is None and 'building:levels' in building.index:
            try:
                levels = float(building['building:levels'])
                btype = building.get('building', 'yes')
                if btype in ['apartments', 'residential']:
                    height = levels * 3.0
                elif btype == 'office':
                    height = levels * 4.0
                elif btype == 'commercial':
                    height = levels * 4.5
                else:
                    height = levels * 3.5
            except Exception:
                pass

        # 3) domyślne wg typu
        if height is None:
            btype = building.get('building', 'yes')
            defaults = {
                'house': 6.0, 'garage': 3.0, 'shed': 2.5,
                'commercial': 12.0, 'industrial': 8.0,
                'apartments': 15.0, 'office': 20.0, 'yes': 10.0
            }
            height = defaults.get(btype, 10.0)

        return float(height)

    # --- Occupancy grid ---

    def _geom_to_local_polys(self, geom, ref_lat, ref_lon):
        """Zamień geometrię budynku (WGS84) na listę poligonów w metrach (x,y)."""
        polys = []
        if geom is None:
            return polys
        geoms = []
        if isinstance(geom, Polygon):
            geoms = [geom]
        elif isinstance(geom, MultiPolygon):
            geoms = list(geom.geoms)
        else:
            return polys

        for g in geoms:
            try:
                # exterior
                ex = list(g.exterior.coords)
                ex_xy = []
                for lon, lat in ex:
                    xx, yy, _ = self.latlon_to_meters(lat, lon, 0.0, ref_lat, ref_lon)
                    ex_xy.append((xx, yy))
                holes = []
                for hole in g.interiors:
                    h_xy = []
                    for lon, lat in hole.coords:
                        xx, yy, _ = self.latlon_to_meters(lat, lon, 0.0, ref_lat, ref_lon)
                        h_xy.append((xx, yy))
                    holes.append(h_xy)

                poly = Polygon(ex_xy, holes) if holes else Polygon(ex_xy)
                if not poly.is_valid and len(ex_xy) >= 3:
                    poly = Polygon(LinearRing(ex_xy))
                if poly.is_valid and not poly.is_empty:
                    polys.append(poly)
            except Exception:
                continue
        return polys

    def create_occupancy_grid(self, bbox, ref_lat, ref_lon):
        """
        Stwórz 3D occupancy grid w METRACH i oznacz budynki z buforem avoid_distance.

        Parameters
        ----------
        bbox : (x_min, x_max, y_min, y_max) [m]
        ref_lat, ref_lon : float

        Returns
        -------
        grid : ndarray (nx, ny, nz) bool
            True = occupied
        """
        x_min, x_max, y_min, y_max = bbox

        # Rozmiar gridu
        nx = int((x_max - x_min) / self.grid_resolution) + 1
        ny = int((y_max - y_min) / self.grid_resolution) + 1
        nz = int((self.altitude_max - self.altitude_min) / self.grid_resolution) + 1

        print(f"\n[Grid] Wymiary: {nx} x {ny} x {nz} = {nx*ny*nz:,} komórek")
        print(f"[Grid] Rozdzielczość: {self.grid_resolution} m")
        print(f"[Grid] Wysokości: {self.altitude_min}–{self.altitude_max} m")

        grid = np.zeros((nx, ny, nz), dtype=bool)
        self.grid_origin = (x_min, y_min, self.altitude_min)
        self.grid_shape = (nx, ny, nz)

        # Filtrowanie budynków bbox (w WGS)
        lat_min, lon_min, _ = self.meters_to_latlon(x_min, y_min, 0, ref_lat, ref_lon)
        lat_max, lon_max, _ = self.meters_to_latlon(x_max, y_max, 0, ref_lat, ref_lon)
        try:
            buildings_in_bbox = self.buildings.cx[lon_min:lon_max, lat_min:lat_max]
            print(f"[Grid] Budynków w obszarze: {len(buildings_in_bbox)} / {len(self.buildings)}")
        except Exception:
            # fallback na .bounds
            b = self.buildings
            bounds = b.geometry.bounds
            mask = (
                (bounds['minx'] <= lon_max) &
                (bounds['maxx'] >= lon_min) &
                (bounds['miny'] <= lat_max) &
                (bounds['maxy'] >= lat_min)
            )
            buildings_in_bbox = b[mask]
            print(f"[Grid] Budynków w obszarze: {len(buildings_in_bbox)} / {len(self.buildings)}")

        # 1) Przekształć do metrycznego XY i zbuforuj w METRACH (avoid_distance)
        buffered_buildings = []
        for _, b in buildings_in_bbox.iterrows():
            try:
                h_top = self.estimate_building_height(b) + float(self.avoid_distance)
                polys = self._geom_to_local_polys(b.geometry, ref_lat, ref_lon)
                if not polys:
                    continue
                for p in polys:
                    pb = p.buffer(float(self.avoid_distance))  # bufor w metrach
                    if pb.is_empty:
                        continue
                    buffered_buildings.append((prep(pb), pb.bounds, h_top))
            except Exception:
                continue

        print(f"[Grid] Zbuforowanych brył: {len(buffered_buildings)}")

        # 2) Rasteryzacja do gridu (XY), oznacz Z < h_top jako occupied
        xs = x_min + np.arange(nx) * self.grid_resolution
        ys = y_min + np.arange(ny) * self.grid_resolution

        def to_idx_x(x):
            return int(np.clip(np.floor((x - x_min) / self.grid_resolution), 0, nx - 1))

        def to_idx_y(y):
            return int(np.clip(np.floor((y - y_min) / self.grid_resolution), 0, ny - 1))

        z_vals = self.grid_origin[2] + np.arange(nz) * self.grid_resolution
        buildings_marked = 0

        for pb_prep, (bx0, by0, bx1, by1), h_top in buffered_buildings:
            i0 = to_idx_x(bx0); i1 = to_idx_x(bx1)
            j0 = to_idx_y(by0); j1 = to_idx_y(by1)
            if i1 < 0 or j1 < 0 or i0 >= nx or j0 >= ny:
                continue

            k_mask = z_vals < h_top
            if not np.any(k_mask):
                continue
            ks = np.where(k_mask)[0]

            for i in range(i0, i1 + 1):
                cx = xs[i]
                for j in range(j0, j1 + 1):
                    cy = ys[j]
                    if pb_prep.contains(Point(cx, cy)):
                        grid[i, j, ks] = True

            buildings_marked += 1
            if buildings_marked % 200 == 0:
                print(f"[Grid] Raster: {buildings_marked}/{len(buffered_buildings)}")

        occupied_cells = int(np.sum(grid))
        occupancy_ratio = occupied_cells / (nx * ny * nz) * 100.0
        print(f"[Grid] Oznaczono brył: {buildings_marked}")
        print(f"[Grid] Zajętych komórek: {occupied_cells:,} ({occupancy_ratio:.1f}%)")

        return grid

    # --- Narzędzia gridowe ---

    def world_to_grid(self, x, y, z):
        """Świat (metry) -> indeksy gridu (i, j, k)."""
        i = int((x - self.grid_origin[0]) / self.grid_resolution)
        j = int((y - self.grid_origin[1]) / self.grid_resolution)
        k = int((z - self.grid_origin[2]) / self.grid_resolution)
        return (i, j, k)

    def grid_to_world(self, i, j, k):
        """Indeksy gridu -> metry świata (x, y, z)."""
        x = self.grid_origin[0] + i * self.grid_resolution
        y = self.grid_origin[1] + j * self.grid_resolution
        z = self.grid_origin[2] + k * self.grid_resolution
        return (x, y, z)

    def is_valid_node(self, position):
        """Czy węzeł w granicach i niezajęty?"""
        i, j, k = position
        nx, ny, nz = self.grid_shape
        if i < 0 or i >= nx or j < 0 or j >= ny or k < 0 or k >= nz:
            return False
        return not self.occupancy_grid[i, j, k]

    # --- A* ---

    def get_neighbors(self, node):
        """26-sąsiedztwo w 3D."""
        neighbors = []
        i, j, k = node.position
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for dk in (-1, 0, 1):
                    if di == 0 and dj == 0 and dk == 0:
                        continue
                    pos = (i + di, j + dj, k + dk)
                    if self.is_valid_node(pos):
                        neighbors.append(Node(pos, node))
        return neighbors

    def heuristic(self, pos1, pos2):
        """Heurystyka euklidesowa w gridzie."""
        return np.sqrt(
            (pos1[0] - pos2[0]) ** 2 +
            (pos1[1] - pos2[1]) ** 2 +
            (pos1[2] - pos2[2]) ** 2
        )

    def movement_cost(self, from_pos, to_pos):
        """Koszt ruchu (dystans + kary wysokości)."""
        dx = abs(to_pos[0] - from_pos[0])
        dy = abs(to_pos[1] - from_pos[1])
        dz = abs(to_pos[2] - from_pos[2])

        # bazowy koszt (ortho / diag 2D / diag 3D)
        if dx + dy + dz == 1:
            base = 1.0
        elif dx + dy + dz == 2:
            base = 1.414
        else:
            base = 1.732

        # kara za zmianę wysokości i latanie wyżej
        altitude_penalty = abs(dz) * 0.5
        _, _, z = self.grid_to_world(*to_pos)
        height_penalty = (z - self.altitude_min) * 0.01

        return base + altitude_penalty + height_penalty

    def astar_3d(self, start_pos, goal_pos):
        """A* w 3D. Zwraca listę indeksów gridu [(i,j,k), ...] lub None."""
        print("\n[A*] Rozpoczęcie planowania...")

        start_node = Node(start_pos)
        goal_node = Node(goal_pos)

        open_list = []
        heapq.heappush(open_list, (0.0, start_node))
        open_dict = {start_pos: start_node}
        closed = set()

        iterations = 0
        max_iterations = 200000

        while open_list and iterations < max_iterations:
            iterations += 1
            if iterations % 5000 == 0:
                print(f"[A*] Iteracja {iterations}, open={len(open_list)}")

            _, current = heapq.heappop(open_list)
            open_dict.pop(current.position, None)

            if current.position == goal_pos:
                print(f"[A*] ✓ Znaleziono ścieżkę po {iterations} iteracjach")
                path = []
                p = current
                while p is not None:
                    path.append(p.position)
                    p = p.parent
                return list(reversed(path))

            closed.add(current.position)

            for nb in self.get_neighbors(current):
                if nb.position in closed:
                    continue

                g_new = current.g + self.movement_cost(current.position, nb.position)
                h_new = self.heuristic(nb.position, goal_pos)
                f_new = g_new + h_new

                if nb.position in open_dict and g_new >= open_dict[nb.position].g:
                    continue

                nb.g, nb.h, nb.f = g_new, h_new, f_new
                nb.parent = current
                heapq.heappush(open_list, (nb.f, nb))
                open_dict[nb.position] = nb

        print(f"[A*] ✗ Nie znaleziono ścieżki (iteracje={iterations})")
        return None

    # --- Wygładzanie z ochroną przed kolizją ---

    def smooth_path(self, path_grid):
        """
        Wygładź ścieżkę B-spline, ale utrzymaj bezkolizyjność.
        Przy kolizji zmniejsz smoothing; jeśli nadal kolizja — zwróć ścieżkę surową.
        """
        print("\n[Smoothing] Wygładzanie ścieżki...")
        path_world = np.array([self.grid_to_world(i, j, k) for i, j, k in path_grid])

        if len(path_world) < 4:
            print("[Smoothing] Ścieżka zbyt krótka — bez wygładzania")
            return path_world

        def is_collision_free(points_m):
            nx, ny, nz = self.grid_shape
            for x, y, z in points_m:
                i, j, k = self.world_to_grid(x, y, z)
                if i < 0 or j < 0 or k < 0 or i >= nx or j >= ny or k >= nz:
                    return False
                if self.occupancy_grid[i, j, k]:
                    return False
            return True

        s_factor = len(path_world) * self.grid_resolution
        for attempt in range(6):
            try:
                tck, u = splprep(
                    [path_world[:, 0], path_world[:, 1], path_world[:, 2]],
                    s=s_factor,
                    k=min(3, len(path_world) - 1)
                )
                u_fine = np.linspace(0, 1, len(path_world) * 5)
                sx, sy, sz = splev(u_fine, tck)
                smooth = np.column_stack([sx, sy, sz])

                if is_collision_free(smooth):
                    print(f"[Smoothing] ✓ OK (attempt {attempt+1}, s={s_factor:.2f}) | {len(smooth)} pkt")
                    return smooth
                else:
                    print(f"[Smoothing] ⚠ Kolizja po wygładzeniu (attempt {attempt+1}) — zmniejszam s")
                    s_factor *= 0.4
            except Exception as e:
                print(f"[Smoothing] ⚠ Błąd wygładzania: {e} — zwracam ścieżkę surową")
                return path_world

        print("[Smoothing] ✗ Nie udało się wygładzić bez kolizji — używam ścieżki surowej")
        return path_world

    # --- Główne planowanie ---

    def plan_path(self, lat_A, lon_A, lat_B, lon_B):
        """
        Główna funkcja planowania ścieżki.

        Returns
        -------
        dict lub None:
            {
                'path_latlon': [(lat, lon, alt), ...],
                'path_meters': np.ndarray(N,3),
                'distance': float,
                'ref_point': (ref_lat, ref_lon)
            }
        """
        print("\n" + "=" * 70)
        print("3D A* PATH PLANNING")
        print("=" * 70)

        ref_lat, ref_lon = lat_A, lon_A

        # A i B -> metry
        start_x, start_y, _ = self.latlon_to_meters(lat_A, lon_A, 0.0, ref_lat, ref_lon)
        goal_x, goal_y, _ = self.latlon_to_meters(lat_B, lon_B, 0.0, ref_lat, ref_lon)

        start_z = float(self.config.altitude_start)
        goal_z = float(self.config.altitude_end)

        print(f"\n[Setup] Start: ({start_x:.1f}, {start_y:.1f}, {start_z:.1f}) m")
        print(f"[Setup] Goal:  ({goal_x:.1f}, {goal_y:.1f}, {goal_z:.1f}) m")

        # BBox z marginesem (wystarczający na omijanie)
        margin = max(50.0, float(3.0 * self.avoid_distance))
        x_min = min(start_x, goal_x) - margin
        x_max = max(start_x, goal_x) + margin
        y_min = min(start_y, goal_y) - margin
        y_max = max(start_y, goal_y) + margin
        bbox = (x_min, x_max, y_min, y_max)

        print(f"[Setup] BBox: X=[{x_min:.1f},{x_max:.1f}] Y=[{y_min:.1f},{y_max:.1f}]")

        # Tworzenie occupancy grid
        self.occupancy_grid = self.create_occupancy_grid(bbox, ref_lat, ref_lon)

        # Start/Goal w gridzie
        start_grid = self.world_to_grid(start_x, start_y, start_z)
        goal_grid = self.world_to_grid(goal_x, goal_y, goal_z)
        print(f"[Setup] Start grid: {start_grid}")
        print(f"[Setup] Goal  grid: {goal_grid}")

        # jeśli punkty w przeszkodzie — znajdź najbliższy wolny
        if not self.is_valid_node(start_grid):
            print("[Fix] Start w przeszkodzie — szukam najbliższego wolnego...")
            start_grid = self.find_nearest_free_node(start_grid)
            print(f"[Fix] Nowy start: {start_grid}")

        if not self.is_valid_node(goal_grid):
            print("[Fix] Cel w przeszkodzie — szukam najbliższego wolnego...")
            goal_grid = self.find_nearest_free_node(goal_grid)
            print(f"[Fix] Nowy cel: {goal_grid}")

        # A*
        path_grid = self.astar_3d(start_grid, goal_grid)
        if path_grid is None:
            print("\n[ERROR] ✗ Nie znaleziono ścieżki")
            return None

        # Wygładzenie bez kolizji
        path_smooth = self.smooth_path(path_grid)

        # Metry -> LatLon
        path_latlon = []
        for x, y, z in path_smooth:
            lat, lon, alt = self.meters_to_latlon(x, y, z, ref_lat, ref_lon)
            path_latlon.append((lat, lon, alt))

        # Długość całkowita
        dist = 0.0
        for i in range(len(path_smooth) - 1):
            d = np.linalg.norm(path_smooth[i + 1] - path_smooth[i])
            dist += float(d)

        print(f"\n[Result] ✓ Długość ścieżki: {dist:.1f} m | punkty: {len(path_smooth)}")
        print("=" * 70)

        return {
            'path_latlon': path_latlon,
            'path_meters': path_smooth,
            'distance': dist,
            'ref_point': (ref_lat, ref_lon),
        }

    def find_nearest_free_node(self, position):
        """Najbliższy wolny węzeł wokół zadanej pozycji (prosta „spirala”)."""
        i, j, k = position
        nx, ny, nz = self.grid_shape
        max_r = max(nx, ny, nz)

        for radius in range(1, max(5, min(40, max_r))):
            for di in range(-radius, radius + 1):
                for dj in range(-radius, radius + 1):
                    for dk in range(-radius, radius + 1):
                        test = (i + di, j + dj, k + dk)
                        if self.is_valid_node(test):
                            return test
        return position

    # --- Wizualizacja 3D (opcjonalnie) ---

    def visualize_path_3d(self, path_result):
        """Wizualizuj zaplanowaną ścieżkę 3D (metry)."""
        if path_result is None:
            return

        path = path_result['path_meters']
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b-', linewidth=2, label='Ścieżka drona')
        ax.scatter(path[0, 0], path[0, 1], path[0, 2], c='green', s=120, marker='o', label='Start', zorder=5)
        ax.scatter(path[-1, 0], path[-1, 1], path[-1, 2], c='red', s=120, marker='o', label='Koniec', zorder=5)

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Wysokość [m]')
        ax.set_title('Zaplanowana ścieżka 3D', fontweight='bold', fontsize=14)
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
