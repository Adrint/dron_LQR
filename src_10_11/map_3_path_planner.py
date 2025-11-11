
"""
route_planner_integrated.py
===========================
Jedna klasa zintegrowana do SEKCJI 4: wybór/analiza obiektów + A* 3D + wygładzanie + wykresy.

Najważniejsze: wszystkie parametry pobierane są z `config` w __init__:
    self.grid_resolution = config.grid_resolution
    self.altitude_start  = config.altitude_start
    self.altitude_end    = config.altitude_end
    self.altitude_min    = config.altitude_cruise_min
    self.altitude_max    = config.altitude_cruise_max
    self.avoid_distance  = config.avoid_distance

Publiczne użycie:
-----------------
from route_planner_integrated import DroneRoutePlanner
planner = DroneRoutePlanner(config)
result = planner.plan(point_a, point_b, show_plots=True)
# Zapisze planned_path.npz (X_ref, Y_ref, Z_ref w NED)

Pliki danych:
-------------
data/warsaw_buildings.pkl  (GeoDataFrame budynków OSM, opcjonalne)
"""

import heapq
import json
from pathlib import Path
from typing import Optional, Tuple, Dict, Any, List

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.interpolate import splprep, splev
from shapely.geometry import Point, Polygon, MultiPolygon, LinearRing
from shapely.prepared import prep

# ---- Ścieżki plików
DATA_DIR = Path("data")
IN_POINTS = Path("selected_points.json")
OUT_PATH = Path("planned_path.npz")


class Node:
    """Węzeł grafu A*."""
    def __init__(self, position, parent=None):
        self.position = position  # (i, j, k) w gridzie
        self.parent = parent
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0

    def __lt__(self, other):
        return self.f < other.f


class DroneRoutePlanner:
    """Zintegrowany moduł planowania trasy 3D z omijaniem obiektów (jedna klasa)."""

    def __init__(self, config):
        # --- PARAMETRY KLUCZOWE Z CONFIG ---
        self.grid_resolution = float(config.grid_resolution)
        self.altitude_start  = float(config.altitude_start)
        self.altitude_end    = float(config.altitude_end)
        self.altitude_min    = float(config.altitude_cruise_min)
        self.altitude_max    = float(config.altitude_cruise_max)
        self.avoid_distance  = float(config.avoid_distance)

        # Dane wejściowe (opcjonalne)
        self.buildings_gdf: Optional[pd.DataFrame] = None

        # Stan gridu
        self.occupancy_grid = None
        self.grid_origin = None      # (x_min, y_min, z_min)
        self.grid_shape = None       # (nx, ny, nz)

    # ====== Konwersje geo ======
    @staticmethod
    def latlon_to_meters(lat, lon, alt, ref_lat, ref_lon):
        y = (lat - ref_lat) * 111000.0
        x = (lon - ref_lon) * 111000.0 * np.cos(np.radians(ref_lat))
        z = alt
        return x, y, z

    @staticmethod
    def meters_to_latlon(x, y, z, ref_lat, ref_lon):
        lat = ref_lat + y / 111000.0
        lon = ref_lon + x / (111000.0 * np.cos(np.radians(ref_lat)))
        return lat, lon, z

    # ====== Budynki / analiza ======
    @staticmethod
    def _estimate_height(row) -> float:
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

    def _geom_to_local_polys(self, geom, ref_lat, ref_lon) -> List[Polygon]:
        """Zamień geometrię WGS84 -> poligony w metrach lokalnych."""
        polys: List[Polygon] = []
        if geom is None:
            return polys
        if isinstance(geom, Polygon):
            geoms = [geom]
        elif isinstance(geom, MultiPolygon):
            geoms = list(geom.geoms)
        else:
            return polys

        for g in geoms:
            try:
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

    @staticmethod
    def _load_buildings_file() -> Optional[pd.DataFrame]:
        pkl = DATA_DIR / "warsaw_buildings.pkl"
        if not pkl.exists():
            print("⚠ Brak pliku z budynkami — planowanie bez kolizji 3D względem zabudowy.")
            return None
        try:
            return pd.read_pickle(pkl)
        except Exception as e:
            print(f"⚠ Nie udało się wczytać budynków: {e}")
            return None

    @staticmethod
    def _cut_bbox(buildings_gdf, lats, lons, margin_m=500.0):
        if buildings_gdf is None or len(buildings_gdf) == 0:
            return buildings_gdf
        try:
            margin_deg = margin_m / 111000.0
            bbox = {
                "north": max(lats) + margin_deg,
                "south": min(lats) - margin_deg,
                "east": max(lons) + margin_deg,
                "west": min(lons) - margin_deg,
            }
            return buildings_gdf.cx[bbox["west"]:bbox["east"], bbox["south"]:bbox["north"]]
        except Exception:
            return buildings_gdf

    # ====== Grid / zajętość ======
    def _create_occupancy_grid(self, bbox, ref_lat, ref_lon):
        """Tworzy 3D occupancy grid i znakowanie budynków z buforem avoid_distance."""
        x_min, x_max, y_min, y_max = bbox

        nx = int((x_max - x_min) / self.grid_resolution) + 1
        ny = int((y_max - y_min) / self.grid_resolution) + 1
        nz = int((self.altitude_max - self.altitude_min) / self.grid_resolution) + 1

        print(f"\n[Grid] {nx}x{ny}x{nz} komórek | res={self.grid_resolution} m | Z={self.altitude_min}–{self.altitude_max} m")

        grid = np.zeros((nx, ny, nz), dtype=bool)
        self.occupancy_grid = grid
        self.grid_origin = (x_min, y_min, self.altitude_min)
        self.grid_shape = (nx, ny, nz)

        if self.buildings_gdf is None or len(self.buildings_gdf) == 0:
            print("[Grid] Brak budynków — siatka pusta")
            return grid

        # bbox w WGS
        lat_min, lon_min, _ = self.meters_to_latlon(x_min, y_min, 0.0, ref_lat, ref_lon)
        lat_max, lon_max, _ = self.meters_to_latlon(x_max, y_max, 0.0, ref_lat, ref_lon)
        try:
            buildings_in_bbox = self.buildings_gdf.cx[lon_min:lon_max, lat_min:lat_max]
        except Exception:
            buildings_in_bbox = self.buildings_gdf

        # przygotowanie brył
        buffered = []
        for _, b in buildings_in_bbox.iterrows():
            try:
                h_top = self._estimate_height(b) + self.avoid_distance
                for p in self._geom_to_local_polys(b.geometry, ref_lat, ref_lon):
                    pb = p.buffer(self.avoid_distance)
                    if not pb.is_empty:
                        buffered.append((prep(pb), pb.bounds, h_top))
            except Exception:
                continue

        xs = x_min + np.arange(nx) * self.grid_resolution
        ys = y_min + np.arange(ny) * self.grid_resolution
        z_vals = self.grid_origin[2] + np.arange(nz) * self.grid_resolution

        def to_idx_x(x):
            return int(np.clip(np.floor((x - x_min) / self.grid_resolution), 0, nx - 1))
        def to_idx_y(y):
            return int(np.clip(np.floor((y - y_min) / self.grid_resolution), 0, ny - 1))

        for pb_prep, (bx0, by0, bx1, by1), h_top in buffered:
            i0, i1 = to_idx_x(bx0), to_idx_x(bx1)
            j0, j1 = to_idx_y(by0), to_idx_y(by1)
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

        occupied = int(np.sum(grid))
        total = int(nx * ny * nz)
        print(f"[Grid] occupied={occupied:,} ({occupied/total*100:.1f}%)")
        return grid

    # ====== Narzędzia gridowe ======
    def _world_to_grid(self, x, y, z):
        i = int((x - self.grid_origin[0]) / self.grid_resolution)
        j = int((y - self.grid_origin[1]) / self.grid_resolution)
        k = int((z - self.grid_origin[2]) / self.grid_resolution)
        return (i, j, k)

    def _grid_to_world(self, i, j, k):
        x = self.grid_origin[0] + i * self.grid_resolution
        y = self.grid_origin[1] + j * self.grid_resolution
        z = self.grid_origin[2] + k * self.grid_resolution
        return (x, y, z)

    def _is_valid_node(self, pos):
        i, j, k = pos
        nx, ny, nz = self.grid_shape
        if i < 0 or i >= nx or j < 0 or j >= ny or k < 0 or k >= nz:
            return False
        return not self.occupancy_grid[i, j, k]

    # ====== A* ======
    def _heuristic(self, a, b):
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _move_cost(self, a, b):
        dx = abs(b[0]-a[0]); dy = abs(b[1]-a[1]); dz = abs(b[2]-a[2])
        if dx + dy + dz == 1:
            base = 1.0
        elif dx + dy + dz == 2:
            base = 1.414
        else:
            base = 1.732
        # kara wysokości
        _, _, z = self._grid_to_world(*b)
        height_penalty = (z - self.altitude_min) * 0.01
        altitude_penalty = abs(dz) * 0.5
        return base + height_penalty + altitude_penalty

    def _neighbors(self, node: Node):
        i, j, k = node.position
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for dk in (-1, 0, 1):
                    if di == 0 and dj == 0 and dk == 0:
                        continue
                    pos = (i + di, j + dj, k + dk)
                    if self._is_valid_node(pos):
                        yield Node(pos, node)

    def _astar(self, start_pos, goal_pos):
        open_heap = []
        start = Node(start_pos)
        start.g = 0.0
        start.h = self._heuristic(start_pos, goal_pos)
        start.f = start.g + start.h
        heapq.heappush(open_heap, (start.f, start))
        open_dict = {start_pos: start}
        closed = set()

        iterations = 0
        max_iter = 200000

        while open_heap and iterations < max_iter:
            iterations += 1
            _, current = heapq.heappop(open_heap)
            if current.position == goal_pos:
                path = []
                p = current
                while p is not None:
                    path.append(p.position)
                    p = p.parent
                print(f"[A*] ✓ iter={iterations} | path_len={len(path)}")
                return list(reversed(path))

            closed.add(current.position)

            for nb in self._neighbors(current):
                if nb.position in closed:
                    continue
                g_new = current.g + self._move_cost(current.position, nb.position)
                h_new = self._heuristic(nb.position, goal_pos)
                f_new = g_new + h_new
                if nb.position in open_dict and g_new >= open_dict[nb.position].g:
                    continue
                nb.g, nb.h, nb.f = g_new, h_new, f_new
                nb.parent = current
                heapq.heappush(open_heap, (nb.f, nb))
                open_dict[nb.position] = nb

        print("[A*] ✗ nie znaleziono ścieżki")
        return None

    # ====== Wygładzanie ======
    def _smooth_path(self, path_grid):
        pts = np.array([self._grid_to_world(i, j, k) for (i, j, k) in path_grid])
        if len(pts) < 4:
            return pts

        def is_free(P):
            nx, ny, nz = self.grid_shape
            for x, y, z in P:
                i, j, k = self._world_to_grid(x, y, z)
                if i < 0 or j < 0 or k < 0 or i >= nx or j >= ny or k >= nz:
                    return False
                if self.occupancy_grid[i, j, k]:
                    return False
            return True

        s_factor = len(pts) * self.grid_resolution
        for _ in range(6):
            try:
                tck, u = splprep([pts[:,0], pts[:,1], pts[:,2]], s=s_factor, k=min(3, len(pts)-1))
                u_f = np.linspace(0, 1, len(pts) * 5)
                sx, sy, sz = splev(u_f, tck)
                smooth = np.column_stack([sx, sy, sz])
                if is_free(smooth):
                    return smooth
                s_factor *= 0.4
            except Exception:
                return pts
        return pts

    # ====== Wizualizacja Y(X) + Z(s) ======
    @staticmethod
    def _rotate_points(x, y, angle_rad):
        c, s = np.cos(angle_rad), np.sin(angle_rad)
        xr = c * x - s * y
        yr = s * x + c * y
        return xr, yr

    def _plot_overview_and_profile(self, path_latlon, latA, lonA, latB, lonB, buildings):
        lats = [p[0] for p in path_latlon]
        lons = [p[1] for p in path_latlon]
        alts = [p[2] for p in path_latlon]
        ref_lat = float(np.mean(lats)); ref_lon = float(np.mean(lons))

        px, py = [], []
        for lat, lon in zip(lats, lons):
            x, y, _ = self.latlon_to_meters(lat, lon, 0, ref_lat, ref_lon)
            px.append(x); py.append(y)
        px = np.array(px); py = np.array(py)

        xa, ya, _ = self.latlon_to_meters(latA, lonA, 0, ref_lat, ref_lon)
        xb, yb, _ = self.latlon_to_meters(latB, lonB, 0, ref_lat, ref_lon)

        angle = -np.arctan2((yb - ya), (xb - xa))
        r = lambda X, Y: self._rotate_points(X, Y, angle)
        px_r, py_r = r(px, py)
        xa_r, ya_r = r(np.array([xa]), np.array([ya]))
        xb_r, yb_r = r(np.array([xb]), np.array([yb]))
        xa_r, ya_r = xa_r[0], ya_r[0]; xb_r, yb_r = xb_r[0], yb_r[0]

        x_min = min(px_r.min(), xa_r, xb_r) - 20
        x_max = max(px_r.max(), xa_r, xb_r) + 20
        y_center = 0.5 * (ya_r + yb_r)
        y_span = max(100.0, min(300.0, max(abs(py_r - y_center)) + 50.0))
        y_min = y_center - y_span; y_max = y_center + y_span

        # dystans s
        dx = np.diff(px); dy = np.diff(py)
        ds = np.hypot(dx, dy)
        s = np.concatenate([[0.0], np.cumsum(ds)])
        alts_arr = np.array(alts, dtype=float)

        # przybliżone h zabudowy pod trasą
        bldg_h = np.zeros_like(s)
        b_in = self._cut_bbox(buildings, lats, lons, margin_m=80.0) if buildings is not None else None
        if b_in is not None and len(b_in) > 0:
            tol_m = 1.5; tol_deg = tol_m / 111000.0
            for i, (lat_i, lon_i) in enumerate(zip(lats, lons)):
                try:
                    around = b_in.cx[lon_i - tol_deg: lon_i + tol_deg, lat_i - tol_deg: lat_i + tol_deg]
                except Exception:
                    around = b_in
                if len(around) == 0:
                    continue
                p = Point(lon_i, lat_i)
                hmax = 0.0
                for _, b in around.iterrows():
                    try:
                        geom = b.geometry
                        if geom is None:
                            continue
                        if geom.contains(p) or geom.distance(p) <= tol_deg:
                            hmax = max(hmax, self._estimate_height(b))
                    except Exception:
                        continue
                bldg_h[i] = hmax

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
        ax1.set_aspect("equal")
        ax1.set_facecolor("#f9f9f9")

        if b_in is not None and len(b_in) > 0:
            for _, b in b_in.iterrows():
                try:
                    geom = b.geometry
                    if geom is None:
                        continue
                    if geom.geom_type == "Polygon":
                        coords = list(geom.exterior.coords)
                    elif geom.geom_type == "MultiPolygon":
                        coords = list(geom.geoms[0].exterior.coords)
                    else:
                        continue
                    lons_b, lats_b = zip(*coords)
                    bx, by = [], []
                    for latv, lonv in zip(lats_b, lons_b):
                        x, y, _ = self.latlon_to_meters(latv, lonv, 0, ref_lat, ref_lon)
                        bx.append(x); by.append(y)
                    bx = np.array(bx); by = np.array(by)
                    bxr, byr = self._rotate_points(bx, by, angle)
                    ax1.fill(bxr, byr, facecolor="orange", edgecolor="darkgray", alpha=0.4, linewidth=0.4, zorder=10)
                except Exception:
                    continue

        ax1.plot([xa_r, xb_r], [ya_r, yb_r], "b--", lw=2, alpha=0.4, label="Linia prosta")
        ax1.plot(px_r, py_r, "r-", lw=3, label="Trasa drona")
        ax1.set_xlim(x_min, x_max); ax1.set_ylim(y_min, y_max)
        ax1.set_ylabel("Y [m]"); ax1.set_title("Widok z góry Y(X) – zaplanowana trasa"); ax1.grid(True, alpha=0.3, linestyle="--")
        ax1.legend(loc="upper right")

        ax2.set_facecolor("#f9f9f9")
        ax2.fill_between(s, 0.0, bldg_h, alpha=0.35, label="Zabudowa pod trasą", zorder=20)
        ax2.plot(s, alts_arr, "r-", lw=3, label="Trasa drona", zorder=100)
        ax2.set_xlabel("Dystans wzdłuż trasy s [m]"); ax2.set_ylabel("Wysokość [m]")
        ax2.grid(True, alpha=0.3, linestyle="--")
        ax2.set_xlim(0, float(s[-1] if s.size else 1.0))
        ax2.set_ylim(bottom=0, top=max(float(alts_arr.max()) if alts_arr.size else 0.0,
                                       float(bldg_h.max()) if bldg_h.size else 0.0) * 1.15 + 1e-6)
        ax2.legend(loc="upper right")
        fig.tight_layout()
        plt.show()

    # ====== Naprawy pozycji start/cel ======
    def _nearest_free(self, pos):
        i, j, k = pos
        nx, ny, nz = self.grid_shape
        for radius in range(1, max(nx, ny, nz)):
            for di in range(-radius, radius+1):
                for dj in range(-radius, radius+1):
                    for dk in range(-radius, radius+1):
                        test = (i+di, j+dj, k+dk)
                        if 0 <= test[0] < nx and 0 <= test[1] < ny and 0 <= test[2] < nz:
                            if not self.occupancy_grid[test]:
                                return test
        return pos

    # ====== Główny interfejs ======
    def plan(self, point_a: Optional[Tuple[float, float]],
                   point_b: Optional[Tuple[float, float]],
                   show_plots: bool = True) -> Optional[Dict[str, Any]]:
        """
        Planowanie trasy z pełnym użyciem parametrów z `config`.
        Zwraca słownik z trasą i zapisuje planned_path.npz.

        point_a/b: (lat, lon)
        """
        if point_a is None or point_b is None:
            # spróbuj wczytać interaktywnie wybrane punkty (jeśli są)
            if IN_POINTS.exists():
                try:
                    data = json.loads(IN_POINTS.read_text(encoding="utf-8"))
                    point_a = (float(data["A"]["lat"]), float(data["A"]["lon"]))
                    point_b = (float(data["B"]["lat"]), float(data["B"]["lon"]))
                    print(f"✓ Wczytano punkty z {IN_POINTS}: A={point_a}, B={point_b}")
                except Exception as e:
                    print(f"✗ Brak punktów A/B i błąd wczytywania {IN_POINTS}: {e}")
                    return None
            else:
                print("✗ Brak punktów A/B")
                return None

        latA, lonA = float(point_a[0]), float(point_a[1])
        latB, lonB = float(point_b[0]), float(point_b[1])

        # Wczytaj budynki i utnij do bbox wokół A-B (ANALIZA W KORYTARZU)
        self.buildings_gdf = self._load_buildings_file()
        if self.buildings_gdf is not None:
            self.buildings_gdf = self._cut_bbox(self.buildings_gdf, [latA, latB], [lonA, lonB], margin_m=500.0)
            print(f"✓ Budynki w analizie: {len(self.buildings_gdf)}")

        # Odniesienie metryczne
        ref_lat, ref_lon = latA, lonA
        start_x, start_y, _ = self.latlon_to_meters(latA, lonA, 0, ref_lat, ref_lon)
        goal_x,  goal_y,  _ = self.latlon_to_meters(latB, lonB, 0, ref_lat, ref_lon)
        start_z = self.altitude_start
        goal_z  = self.altitude_end

        # BBOX z marginesem zależnym od avoid_distance
        margin = max(50.0, 3.0 * self.avoid_distance)
        x_min = min(start_x, goal_x) - margin
        x_max = max(start_x, goal_x) + margin
        y_min = min(start_y, goal_y) - margin
        y_max = max(start_y, goal_y) + margin

        # Siatka zajętości 3D
        self._create_occupancy_grid((x_min, x_max, y_min, y_max), ref_lat, ref_lon)

        # Start / cel -> grid
        start_g = self._world_to_grid(start_x, start_y, start_z)
        goal_g  = self._world_to_grid(goal_x,  goal_y,  goal_z)
        if not self._is_valid_node(start_g):
            print("[Fix] Start w przeszkodzie — szukam wolnego...")
            start_g = self._nearest_free(start_g)
        if not self._is_valid_node(goal_g):
            print("[Fix] Cel w przeszkodzie — szukam wolnego...")
            goal_g = self._nearest_free(goal_g)

        # A*
        path_grid = self._astar(start_g, goal_g)
        if path_grid is None:
            print("✗ Nie udało się wyznaczyć ścieżki")
            return None

        # Wygładzenie
        path_m = self._smooth_path(path_grid)

        # Metry -> WGS84
        path_latlon = [self.meters_to_latlon(x, y, z, ref_lat, ref_lon) for (x, y, z) in path_m]

        # Długość
        dist = 0.0
        for i in range(len(path_m)-1):
            dist += float(np.linalg.norm(path_m[i+1]-path_m[i]))

        # Zapis NED (Z_ref ujemny dla „up”)
        X_ref = path_m[:, 0]
        Y_ref = path_m[:, 1]
        Z_ref = -path_m[:, 2]
        np.savez(OUT_PATH, X_ref=X_ref, Y_ref=Y_ref, Z_ref=Z_ref)
        print(f"✓ Zapisano {OUT_PATH} | długość={dist:.1f} m | punkty={len(path_m)}")

        result = {
            "path_latlon": path_latlon,
            "path_meters": path_m,
            "distance": dist,
            "ref_point": (ref_lat, ref_lon),
        }

        if show_plots:
            self._plot_overview_and_profile(path_latlon, latA, lonA, latB, lonB, self.buildings_gdf)

        return result
