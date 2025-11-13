import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, MultiPolygon

COLORS = {
    "building_face": "#FFA500",
    "building_edge": "darkgray",
    "track": "red",
    "drone": "blue",
}


class PlannerStyleAnimator:
    """
    Dwa wykresy jak w map_3_path_planner:
      - ax1 (Y(X)): budynki + trasa referencyjna + ruch drona (ślad + marker)
      - ax2 (Z(s)): wysokość zabudowy na zaplanowanej trasie + profil trasy + AGL drona
    """

    # ---------- pomocnicze ----------
    @staticmethod
    def _cumdist(x, y):
        if len(x) < 2:
            return np.array([0.0])
        d = np.hypot(np.diff(x), np.diff(y))
        return np.concatenate([[0.0], np.cumsum(d)])

    def __init__(self, path_latlon, point_a, point_b, buildings_gdf,
                 latlon_to_meters_fn, estimate_height_fn,
                 sim_ref_s=None):
        """
        path_latlon : list[(lat,lon,alt)] – zaplanowana trasa (WGS84 + alt)
        point_a/b   : (lat,lon)
        buildings_gdf : GeoDataFrame budynków
        latlon_to_meters_fn(lat,lon,alt,ref_lat,ref_lon) -> (x,y,z)
        estimate_height_fn(row)->float
        sim_ref_s : array - dystans referencyjny z symulacji
        """
        self.latlon_to_meters = latlon_to_meters_fn
        self.estimate_height = estimate_height_fn
        self.point_a = point_a
        self.point_b = point_b
        self.buildings_gdf = buildings_gdf

        # --- dane trasy (WGS84) ---
        self.lats = np.array([p[0] for p in path_latlon], dtype=float)
        self.lons = np.array([p[1] for p in path_latlon], dtype=float)
        self.alts = np.array([p[2] for p in path_latlon], dtype=float)
        self.ref_lat = float(np.mean(self.lats))
        self.ref_lon = float(np.mean(self.lons))

        # --- rzut do metrów (lokalnych) ---
        px, py = [], []
        for lat, lon in zip(self.lats, self.lons):
            x, y, _ = self.latlon_to_meters(lat, lon, 0.0, self.ref_lat, self.ref_lon)
            px.append(x);
            py.append(y)
        self.px = np.asarray(px);
        self.py = np.asarray(py)

        # --- obrót jak w map_3 (wg A–B) -> OSTATECZNA baza Y(X) ---
        xa, ya, _ = self.latlon_to_meters(point_a[0], point_a[1], 0.0, self.ref_lat, self.ref_lon)
        xb, yb, _ = self.latlon_to_meters(point_b[0], point_b[1], 0.0, self.ref_lat, self.ref_lon)
        self.angle = -np.arctan2((yb - ya), (xb - xa))

        c, s = np.cos(self.angle), np.sin(self.angle)
        self.px_r = c * self.px - s * self.py
        self.py_r = s * self.px + c * self.py

        # PROBLEM 4: Nie używamy transformacji Kabsch - używamy dystansu bezpośrednio
        # Dystans referencyjny z symulacji (jeśli podany)
        self.sim_ref_s = sim_ref_s if sim_ref_s is not None else np.zeros(1)

        # --- zakresy jak w map_3 ---
        self.xa_r = float(c * xa - s * ya)
        self.ya_r = float(s * xa + c * ya)
        self.xb_r = float(c * xb - s * yb)
        self.yb_r = float(s * xb + c * yb)

        # Przesuń współrzędne tak aby A było w 0 (dystans od 0)
        self.px_r_dist = self.px_r - self.xa_r

        # Dystans wzdłuż trasy
        self.s = self._cumdist(self.px, self.py)
        self.smax = float(self.s[-1]) if self.s.size else 1.0

        # Zakresy dla wykresów - dystans od 0
        self.x_min = -50.0
        self.x_max = self.smax + 50.0
        y_center = 0.5 * (self.ya_r + self.yb_r)
        y_span = max(100.0, min(300.0, float(np.max(np.abs(self.py_r - y_center))) + 50.0))
        self.y_min = y_center - y_span
        self.y_max = y_center + y_span

        # --- profil wysokości zabudowy wzdłuż trasy ---
        self.bldg_h = self._compute_building_profile()

        # --- wysokość trasy (zakładamy, że alts jest docelowym profilem) ---
        self.alts_arr = np.asarray(self.alts, dtype=float)

        # --- rysunek bazowy ---
        self._init_figure()

    def _compute_building_profile(self):
        """Maks. wysokość budynku pod każdą próbką trasy (jak w map_3)."""
        if self.buildings_gdf is None or len(self.buildings_gdf) == 0:
            return np.zeros_like(self.s)

        bldg_h = np.zeros_like(self.s)
        tol_m = 1.5
        tol_deg = tol_m / 111000.0

        for i, (lat_i, lon_i) in enumerate(zip(self.lats, self.lons)):
            try:
                around = self.buildings_gdf.cx[lon_i - tol_deg: lon_i + tol_deg,
                         lat_i - tol_deg: lat_i + tol_deg]
            except Exception:
                around = self.buildings_gdf
            if len(around) == 0:
                continue

            import shapely.geometry as sg
            p = sg.Point(lon_i, lat_i)
            hmax = 0.0
            for _, row in around.iterrows():
                geom = row.geometry
                if geom is None:
                    continue
                try:
                    if geom.contains(p) or geom.distance(p) <= tol_deg:
                        hmax = max(hmax, float(self.estimate_height(row)))
                except Exception:
                    continue
            bldg_h[i] = hmax
        return bldg_h

    def _geom_to_local_polys(self, geom):
        """WGS84 -> poligony w metrach, po obrocie (jak w map_3)."""
        polys_xy = []
        if geom is None:
            return polys_xy
        geoms = [geom] if isinstance(geom, Polygon) else (list(geom.geoms) if isinstance(geom, MultiPolygon) else [])
        for g in geoms:
            try:
                coords = list(g.exterior.coords)
                bx, by = [], []
                for lonv, latv in coords:
                    x, y, _ = self.latlon_to_meters(latv, lonv, 0.0, self.ref_lat, self.ref_lon)
                    bx.append(x);
                    by.append(y)
                bx = np.array(bx);
                by = np.array(by)
                c, s = np.cos(self.angle), np.sin(self.angle)
                bxr = c * bx - s * by
                byr = s * bx + c * by
                # Przesuń na dystans od 0
                bxr_dist = bxr - self.xa_r
                polys_xy.append((bxr_dist, byr))
            except Exception:
                continue
        return polys_xy

    def _init_figure(self):
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(14, 10))
        for ax in (self.ax1, self.ax2):
            ax.set_facecolor("#f9f9f9")
            ax.grid(True, alpha=0.3, linestyle="--")

        # ===== ax1: Y(dystans) =====
        self.ax1.set_aspect("equal", adjustable="datalim")
        self.ax1.set_xlim(self.x_min, self.x_max)
        self.ax1.set_ylim(self.y_min, self.y_max)
        self.ax1.set_xlabel("Dystans [m]")
        self.ax1.set_ylabel("Y [m]")
        self.ax1.set_title("Widok z góry Y(dystans) – zaplanowana trasa")

        if self.buildings_gdf is not None and len(self.buildings_gdf) > 0:
            for _, b in self.buildings_gdf.iterrows():
                for bxr, byr in self._geom_to_local_polys(b.geometry):
                    self.ax1.fill(
                        bxr, byr,
                        facecolor=COLORS["building_face"],
                        edgecolor=COLORS["building_edge"],
                        alpha=0.5, linewidth=0.5, zorder=10
                    )

        # trasa referencyjna (na dystansie od 0)
        self.ax1.plot(self.px_r_dist, self.py_r, color=COLORS["track"], lw=3, label="Trasa referencyjna", zorder=50)

        # ślad drona
        self.path_xy_line, = self.ax1.plot([], [], "b-", lw=2, alpha=0.9, label="Ślad drona", zorder=60)
        self.drone_xy_dot, = self.ax1.plot([], [], "o", ms=9, color=COLORS["drone"], zorder=70)

        self.ax1.legend(loc="upper right")

        # ===== ax2: Z(s) =====
        self.ax2.set_xlabel("Dystans wzdłuż trasy s [m]")
        self.ax2.set_ylabel("Wysokość [m]")
        self.ax2.set_title("Wysokość zabudowy na zaplanowanej trasie")

        self.ax2.fill_between(self.s, 0.0, self.bldg_h, facecolor=COLORS["building_face"], alpha=0.6,
                              label="Wysokość zabudowy", zorder=5)
        self.ax2.plot(self.s, self.bldg_h, linewidth=0.5, color=COLORS["building_edge"], zorder=6)

        self.ax2.plot(self.s, self.alts_arr, "r-", lw=3, label="Wysokość trasy", zorder=100)

        self.path_alt_line, = self.ax2.plot([], [], "b-", lw=2, alpha=0.9, label="AGL drona", zorder=120)
        self.drone_alt_dot, = self.ax2.plot([], [], "o", ms=9, color=COLORS["drone"], zorder=130)

        top = max(float(self.alts_arr.max(initial=0.0)), float(self.bldg_h.max(initial=0.0))) * 1.15 + 1e-6
        self.ax2.set_xlim(0.0, self.smax)
        self.ax2.set_ylim(0.0, top)
        self.ax2.legend(loc="upper right")

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # ---------- aktualizacja ruchu drona ----------
    def update_drone(self, states_array):
        """
        states_array[:,6] = X, [:,7] = Y, [:,8] = Z (NED, czyli AGL = -Z)
        PROBLEM 4: Obliczamy dystans bezpośrednio z współrzędnych symulacji
        """
        if states_array is None or len(states_array) == 0:
            return

        Xp = states_array[:, 6]
        Yp = states_array[:, 7]
        Zp = states_array[:, 8]
        AGLp = -Zp

        # Oblicz dystans przebity w symulacji
        Sp = self._cumdist(Xp, Yp)

        # Dla widoku z góry - obrót współrzędnych jak w planie
        c, s = np.cos(self.angle), np.sin(self.angle)
        Xpr = c * Xp - s * Yp
        Ypr = s * Xp + c * Yp

        # Przesuń na dystans od 0
        Xpr_dist = Xpr - self.xa_r

        # Uaktualnij ślady
        self.path_xy_line.set_data(Xpr_dist, Ypr)
        self.drone_xy_dot.set_data([Xpr_dist[-1]], [Ypr[-1]])

        self.path_alt_line.set_data(Sp, AGLp)
        self.drone_alt_dot.set_data([Sp[-1]], [AGLp[-1]])

        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def close(self):
        plt.ioff()
        plt.show()