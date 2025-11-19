"""
Wizualizacja lotu drona w czasie rzeczywistym.
Używa DOKŁADNIE tych samych wykresów co map_3_path_planner._plot_3d_path_straight()
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import LineString
from map_geo_utils import wgs84_to_local, local_to_wgs84, estimate_building_height
from config import config


class DroneLiveAnimator:
    """Animator drona - DOKŁADNIE te same wykresy co w map_3_path_planner"""

    def __init__(self):
        self.fig = None
        self.ax1 = None  # 3D
        self.ax2 = None  # Top view
        self.ax3 = None  # Profile

        # Markery i linie drona
        self.drone_3d = None
        self.traj_3d = None
        self.drone_2d = None
        self.traj_2d = None
        self.drone_prof = None
        self.traj_prof = None

        # Historia
        self.hist_x = []
        self.hist_y = []
        self.hist_z = []
        self.hist_s = []

        # Dane rotacji dla profilu
        self.cos_rot = 1.0
        self.sin_rot = 0.0
        self.xa_rot = 0.0

        # Licznik aktualizacji
        self.counter = 0
        self.interval = 3  # Aktualizuj co N kroków

        # Rzeczywiste wysokości z trasy
        self.altitude_start = 0.0
        self.altitude_end = 0.0

    def initialize(self, path_result, fig=None, gridspec=None):
        """
        Inicjalizacja - DOKŁADNIE TEN SAM KOD CO _plot_3d_path_straight()

        Args:
            path_result: Dict z map_3_path_planner.plan() zawierający:
                - X, Y, Z, s: współrzędne trasy
                - xa, ya, xb, yb: punkty start/cel
                - bounds: granice
                - buildings_gdf: wszystkie budynki
                - transformer: konwerter współrzędnych
                - buildings_in_area: budynki w pobliżu
        """

        # Wyciągnij dane
        X = path_result['X']
        Y = path_result['Y']
        Z_agl = path_result['Z']
        s = path_result['s']
        xa = path_result['xa']
        ya = path_result['ya']
        xb = path_result['xb']
        yb = path_result['yb']
        bounds = path_result['bounds']
        buildings_gdf = path_result['buildings_gdf']
        transformer = path_result['transformer']

        dist_total = path_result['distance']

        # Zapisz rzeczywiste wysokości startu i lądowania z trasy
        self.altitude_start = Z_agl[0]  # Pierwsza wysokość na trasie
        self.altitude_end = Z_agl[-1]   # Ostatnia wysokość na trasie

        print(f"\n[Animator] Inicjalizacja wykresow")
        print(f"Punkty: {len(X)}")
        print(f"Dystans: {dist_total:.1f} m")
        print(f"Wysokosc startowa: {self.altitude_start:.2f} m")
        print(f"Wysokosc ladowania: {self.altitude_end:.2f} m")

        # Wczytaj budynki w obszarze (DOKŁADNIE JAK W MAP_3)
        buildings_in = None
        if buildings_gdf is not None:
            x_min, x_max, y_min, y_max, z_min, z_max = bounds
            margin = 50.0
            try:
                lat_min, lon_min = local_to_wgs84(x_min - margin, y_min - margin, transformer)
                lat_max, lon_max = local_to_wgs84(x_max + margin, y_max + margin, transformer)

                buildings_in = buildings_gdf.cx[
                    min(lon_min, lon_max):max(lon_min, lon_max),
                    min(lat_min, lat_max):max(lat_min, lat_max)
                ]
                print(f"Budynki w obszarze: {len(buildings_in)}")
            except Exception as e:
                print(f"Blad ladowania budynkow: {e}")
                buildings_in = None

        # Oblicz kąt rotacji dla widoku z góry (DOKŁADNIE JAK W MAP_3)
        angle = -np.arctan2((yb - ya), (xb - xa))
        c = np.cos(angle)
        s_rot = np.sin(angle)

        # Rotacja dla profilu wysokości
        self.cos_rot = c
        self.sin_rot = s_rot
        self.xa_rot = c * xa - s_rot * ya

        # UTWORZENIE FIGURY (3 SUBPLOTY)
        self.fig = plt.figure(figsize=(16, 8))
        self.gs = self.fig.add_gridspec(2, 2, height_ratios=[1, 1], width_ratios=[1.2, 1.0])
        self.ax1 = self.fig.add_subplot(self.gs[:, 0], projection='3d')
        self.ax2 = self.fig.add_subplot(self.gs[0, 1])  # prawa górna
        self.ax3 = self.fig.add_subplot(self.gs[1, 1])  # prawa dolna

        # WYKRES 1: 3D - SKOPIOWANE 1:1
        self.ax1.set_facecolor("#f0f0f0")

        # Trasa planowana
        self.ax1.plot(X, Y, Z_agl, 'r-', linewidth=2.5,
                     label=f'Trasa ({dist_total:.0f} m)', zorder=50)

        # Start i cel z rzeczywistymi wysokościami
        self.ax1.scatter(
            [xa], [ya], [self.altitude_start],
            c='green', s=150, marker='o',
            edgecolor='black', linewidth=2,
            label='Start'
        )
        self.ax1.text(
            xa, ya, self.altitude_start,
            "  A", fontsize=13, fontweight="bold"
        )

        self.ax1.scatter(
            [xb], [yb], [self.altitude_end],
            c='red', s=150, marker='o',
            edgecolor='black', linewidth=2,
            label='Cel'
        )
        self.ax1.text(
            xb, yb, self.altitude_end,
            "  B", fontsize=13, fontweight="bold"
        )

        # Budynki 3D (DOKŁADNIE JAK W MAP_3)
        if buildings_in is not None and len(buildings_in) > 0:
            try:
                # Buffer trasy (JAK W MAP_3)
                route_coords = []
                for x_loc, y_loc in zip(X, Y):
                    lat, lon = local_to_wgs84(x_loc, y_loc, transformer)
                    route_coords.append((lon, lat))
                route_line = LineString(route_coords)
                buf = 30.0 / 111000.0  # 30m buffer
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
                            x_loc, y_loc = wgs84_to_local(lat_v, lon_v, transformer)
                            bx.append(x_loc)
                            by.append(y_loc)

                        bx = np.array(bx)
                        by = np.array(by)

                        if not (bx[0] == bx[-1] and by[0] == by[-1]):
                            bx = np.append(bx, bx[0])
                            by = np.append(by, by[0])

                        # Ściany
                        walls = []
                        for i in range(len(bx) - 1):
                            x0, x1 = bx[i], bx[i + 1]
                            y0, y1 = by[i], by[i + 1]
                            wall = [[x0, y0, 0], [x1, y1, 0], [x1, y1, h], [x0, y0, h]]
                            walls.append(wall)

                        # Dach
                        roof = [[bx[i], by[i], h] for i in range(len(bx))]

                        self.ax1.add_collection3d(Poly3DCollection(
                            walls, facecolors='orange', edgecolors='gray',
                            linewidths=0.3, alpha=0.5, zorder=10))
                        self.ax1.add_collection3d(Poly3DCollection(
                            [roof], facecolors='orange', edgecolors='gray',
                            linewidths=0.5, alpha=0.6, zorder=15))
                    except:
                        continue
            except Exception as e:
                print(f"[3D Budynki] {e}")

        self.ax1.set_xlabel('X [m]')
        self.ax1.set_ylabel('Y [m]')
        self.ax1.set_zlabel('Wysokosc [m]')
        self.ax1.set_title('Trasa 3D (X, Y, wysokosc)')
        self.ax1.legend(loc='upper left')
        self.ax1.view_init(elev=20, azim=250)
        self.ax1.grid(True, alpha=0.3)

        # WYKRES 2: WIDOK Z GÓRY - SKOPIOWANE 1:1
        self.ax2.set_facecolor("#f9f9f9")

        # Budynki (JAK W MAP_3)
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
                            x_loc, y_loc = wgs84_to_local(lat_v, lon_v, transformer)
                            bx.append(x_loc)
                            by.append(y_loc)

                        bx = np.array(bx)
                        by = np.array(by)

                        self.ax2.fill(bx, by, facecolor="orange", edgecolor="darkgray",
                                     alpha=0.5, linewidth=0.5, zorder=10)
                    except:
                        continue
            except:
                pass

        # Trasa
        self.ax2.plot(X, Y, 'r-', linewidth=3,
                 label=f'Trasa ({dist_total:.0f} m)',
                 zorder=50)
        self.ax2.scatter(
            [xa], [ya],
            c='green', s=150, marker='o',
            edgecolor="black", linewidth=2,
            zorder=100
        )
        self.ax2.scatter(
            [xb], [yb],
            c='red', s=150, marker='o',
            edgecolor="black", linewidth=2,
            zorder=100
        )
        self.ax2.text(xa, ya, "  A", fontsize=13, fontweight="bold")
        self.ax2.text(xb, yb, "  B", fontsize=13, fontweight="bold")

        self.ax2.set_xlabel("X [m]")
        self.ax2.set_ylabel("Y [m]")
        self.ax2.set_title("Widok z gory: uklad lokalny X-Y")
        self.ax2.grid(True, alpha=0.3, linestyle="--")

        # Granice (JAK W MAP_3)
        margin_xy = 20.0
        self.ax2.set_xlim(min(X.min(), xa, xb) - margin_xy, max(X.max(), xa, xb) + margin_xy)
        self.ax2.set_ylim(min(Y.min(), ya, yb) - margin_xy, max(Y.max(), ya, yb) + margin_xy)
        self.ax2.set_aspect('equal', adjustable='box')

        # WYKRES 3: PROFIL WYSOKOŚCI - SKOPIOWANE 1:1
        self.ax3.set_facecolor("#f9f9f9")

        # Budynki pod trasą (JAK W MAP_3)
        if buildings_in is not None and len(buildings_in) > 0:
            try:
                route_coords = []
                for x_loc, y_loc in zip(X, Y):
                    lat, lon = local_to_wgs84(x_loc, y_loc, transformer)
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
                            x_loc, y_loc = wgs84_to_local(lat_v, lon_v, transformer)
                            bx.append(x_loc)
                            by.append(y_loc)

                        bx = np.array(bx)
                        by = np.array(by)
                        # Rotacja (JAK W MAP_3)
                        bx_r = c * bx - s_rot * by
                        bx_r_dist = bx_r - self.xa_rot

                        if len(bx_r_dist) > 0:
                            w = max(float(np.max(bx_r_dist) - np.min(bx_r_dist)), 3.0)
                            xc = 0.5 * (float(np.max(bx_r_dist)) + float(np.min(bx_r_dist)))
                            rect = plt.Rectangle((xc - w / 2, 0), w, h,
                                                facecolor="orange", edgecolor="darkgray",
                                                alpha=0.6, linewidth=1, zorder=30)
                            self.ax3.add_patch(rect)
                    except:
                        continue
            except Exception as e:
                print(f"[Profil wysokosci] {e}")

        # Trasa z rzeczywistymi wysokościami
        self.ax3.plot(s, Z_agl, 'r-', linewidth=3, label='Wysokosc trasy', zorder=100)
        self.ax3.scatter([0], [self.altitude_start],
                         c='green', s=80, marker='o',
                         edgecolor='black', linewidth=2,
                         zorder=110)
        self.ax3.text(0, self.altitude_start,
                      "  A", fontsize=12, fontweight="bold")

        # Używamy końca osi s, a nie dist_total (długości 3D)
        s_end = float(s[-1])

        self.ax3.scatter([s_end], [self.altitude_end],
                         c='red', s=80, marker='o',
                         edgecolor='black', linewidth=2,
                         zorder=110)
        self.ax3.text(s_end, self.altitude_end,
                      "  B", fontsize=12, fontweight="bold")

        self.ax3.set_xlabel('Dystans [m]')
        self.ax3.set_ylabel('Wysokosc [m]')
        self.ax3.set_title('Profil wysokosci')
        self.ax3.grid(True, alpha=0.3, linestyle="--")
        self.ax3.set_xlim(-50, s_end + 50)
        self.ax3.set_ylim(bottom=-2)

        # DODAJ MARKERY DRONA (na początku niewidoczne)
        # 3D
        self.drone_3d, = self.ax1.plot([], [], [], 'bo', markersize=12,
                                       markeredgecolor='black', markeredgewidth=2.5,
                                       label='Dron', zorder=300)
        self.traj_3d, = self.ax1.plot([], [], [], 'b-', linewidth=2,
                                      alpha=0.7, label='Tor rzeczywisty', zorder=250)

        # 2D
        self.drone_2d, = self.ax2.plot([], [], 'bo', markersize=12,
                                       markeredgecolor='black', markeredgewidth=2.5,
                                       zorder=300)
        self.traj_2d, = self.ax2.plot([], [], 'b-', linewidth=2,
                                      alpha=0.7, label='Tor rzeczywisty', zorder=250)

        # Profile
        self.drone_prof, = self.ax3.plot([], [], 'bo', markersize=12,
                                         markeredgecolor='black', markeredgewidth=2.5,
                                         zorder=300)
        self.traj_prof, = self.ax3.plot([], [], 'b-', linewidth=2,
                                        alpha=0.7, label='Tor rzeczywisty', zorder=250)

        # Aktualizuj legendy
        self.ax1.legend(loc='upper left')

        plt.tight_layout()
        plt.ion()
        plt.show(block=False)
        plt.pause(0.001)

        #print("Wykresy gotowe")

    def update(self, x_state, t, step):
        """Aktualizuj pozycję drona na wykresach."""

        self.counter += 1
        if self.counter % self.interval != 0:
            return

        # Pozycja drona
        x = x_state[6]
        y = x_state[7]
        z_ned = x_state[8]
        z_agl = -z_ned

        # Dodaj do historii
        self.hist_x.append(x)
        self.hist_y.append(y)
        self.hist_z.append(z_agl)

        # Dystans (z rotacją jak w map_3)
        x_rot = self.cos_rot * x - self.sin_rot * y
        s = x_rot - self.xa_rot
        self.hist_s.append(s)

        # Konwertuj do numpy
        hx = np.array(self.hist_x)
        hy = np.array(self.hist_y)
        hz = np.array(self.hist_z)
        hs = np.array(self.hist_s)

        try:
            # Aktualizuj 3D
            self.drone_3d.set_data([x], [y])
            self.drone_3d.set_3d_properties([z_agl])
            self.traj_3d.set_data(hx, hy)
            self.traj_3d.set_3d_properties(hz)

            # Aktualizuj 2D
            self.drone_2d.set_data([x], [y])
            self.traj_2d.set_data(hx, hy)

            # Aktualizuj profil
            self.drone_prof.set_data([s], [z_agl])
            self.traj_prof.set_data(hs, hz)

            # Odśwież
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception as e:
            print(f"[Animator] {e}")

    def close(self):
        """Zamknij animator."""
        print(f"\n[Animator] Koniec - zapisano {len(self.hist_x)} punktow")
        if len(self.hist_s) > 0:
            print(f"Dystans: {self.hist_s[-1]:.1f} m")
        plt.ioff()
        plt.show()