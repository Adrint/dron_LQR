import osmnx as ox
import pickle
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
from pathlib import Path
from geopy.distance import geodesic
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec
from shapely.geometry import Point, LineString
import warnings
warnings.filterwarnings('ignore')

import sys
sys.path.append(str(Path(__file__).parent))
from drone_path_planner import PathPlanner3D
from config import config
from drone_parameters import input_parameters


def rotate_points(x, y, angle_rad):
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    xr = c * x - s * y
    yr = s * x + c * y
    return xr, yr

# Konfiguracja
ox.settings.use_cache = True
DATA_DIR = Path("data")

COLORS = {
    'road': '#999999',
    'building': '#FFA500',
    'point_a': 'green',
    'point_b': 'red',
    'route': 'blue',
    'bbox': 'orange'
}

print("=" * 70)
print("INTERAKTYWNA MAPA WARSZAWY - WYBÓR PUNKTÓW A i B")
print("=" * 70)


class WarsawMapSelector:
    """Klasa do interaktywnej selekcji punktów na mapie Warszawy."""

    def __init__(self):
        self.G = None
        self.buildings = None
        self.fig = None
        self.ax = None
        self.points = []
        self.selection_mode = False
        self.drone_path = None
        self.drone_path_distance = None
        self.path_planner = None

    def load_data(self):
        """Załaduj dane z plików."""
        print("\n[1/2] Ładowanie danych...")

        # Sieć drogowa
        street_network_path = DATA_DIR / "warsaw_street_network.pkl"
        if not street_network_path.exists():
            print(f"✗ Brak pliku: {street_network_path}")
            print(f"→ Uruchom najpierw: 01_download_warsaw_data.py")
            return False

        with open(street_network_path, "rb") as f:
            self.G = pickle.load(f)
        print(f"  ✓ Sieć drogowa: {len(self.G.nodes):,} węzłów")

        # Budynki
        buildings_path = DATA_DIR / "warsaw_buildings.pkl"
        if buildings_path.exists():
            self.buildings = pd.read_pickle(buildings_path)
            print(f"  ✓ Budynki: {len(self.buildings):,}")
        else:
            print(f"  ⚠ Brak budynków")

        return True

    def create_base_map(self):
        """Stwórz podstawową mapę."""
        print("\n[2/2] Tworzenie mapy...")

        self.fig, self.ax = ox.plot_graph(
            self.G,
            node_size=0,
            edge_linewidth=0.5,
            edge_color=COLORS['road'],
            bgcolor='white',
            show=False,
            close=False
        )

        # Dodaj budynki z kolorowaniem według wysokości
        if self.buildings is not None:
            buildings_plot = self.buildings.copy()
            buildings_plot['estimated_height'] = buildings_plot.apply(
                self._estimate_building_height, axis=1
            )

            buildings_plot.plot(
                ax=self.ax,
                column='estimated_height',
                cmap='YlOrRd',
                alpha=0.6,
                edgecolor='darkgray',
                linewidth=0.3,
                legend=True,
                legend_kwds={'label': 'Wysokość [m]', 'shrink': 0.8}
            )
            print(f"  ✓ Wyrenderowano {len(buildings_plot):,} budynków")

        return self.fig, self.ax

    @staticmethod
    def _estimate_building_height(building):
        """Szacuj wysokość budynku."""
        height = None

        if 'height' in building.index and not pd.isna(building['height']):
            try:
                h = str(building['height']).replace('m', '').replace('M', '').strip()
                height = float(h)
            except:
                pass

        if height is None and 'building:levels' in building.index:
            try:
                levels = int(building['building:levels'])
                height = levels * 3.0
            except:
                pass

        return height if height is not None else 10.0

    def setup_interactive_mode(self):
        """Skonfiguruj tryb interaktywny."""
        print("\n" + "=" * 70)
        print("INSTRUKCJA:")
        print("1. Użyj ZOOM/PAN aby znaleźć obszar")
        print("2. Wciśnij SPACJĘ aby włączyć tryb zaznaczania")
        print("3. Kliknij punkt A (zielony), potem B (czerwony)")
        print("=" * 70)

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.ax.set_title(
            "TRYB ZOOM: Użyj zoom/pan, potem SPACJA",
            color='blue', fontweight='bold', fontsize=12
        )

    def on_click(self, event):
        """Obsługa kliknięcia."""
        if self.fig.canvas.toolbar.mode != '' or not self.selection_mode:
            return

        if event.xdata is not None and event.ydata is not None:
            lon, lat = event.xdata, event.ydata
            self.points.append((lat, lon))

            color = COLORS['point_a'] if len(self.points) == 1 else COLORS['point_b']
            label = 'A' if len(self.points) == 1 else 'B'

            self.ax.plot(lon, lat, 'o', color=color, markersize=15,
                        markeredgecolor='black', markeredgewidth=2, zorder=1000)
            self.ax.text(lon, lat, f'  {label}', fontsize=14,
                        fontweight='bold', zorder=1001)

            print(f"✓ Punkt {label}: lat={lat:.6f}, lon={lon:.6f}")
            plt.draw()

            if len(self.points) == 2:
                print("\n✓ Wybrano oba punkty!")
                plt.close()

    def on_key(self, event):
        """Obsługa klawisza."""
        if event.key == ' ':
            self.selection_mode = not self.selection_mode

            if self.selection_mode:
                print("\n🟢 TRYB ZAZNACZANIA")
                self.ax.set_title("TRYB ZAZNACZANIA: Kliknij punkty A i B",
                                color='green', fontweight='bold', fontsize=12)
            else:
                print("\n🔵 TRYB ZOOM")
                self.ax.set_title("TRYB ZOOM: Użyj zoom/pan, potem SPACJA",
                                color='blue', fontweight='bold', fontsize=12)
            plt.draw()

    @staticmethod
    def latlon_to_meters(lat, lon, alt, ref_lat, ref_lon):
        """Konwertuj lat/lon na metry względem punktu odniesienia."""
        y = (lat - ref_lat) * 111000
        x = (lon - ref_lon) * 111000 * np.cos(np.radians(ref_lat))
        z = alt
        return x, y, z

    def visualize_route_and_bbox(self):
        """Wizualizuj trasę prostą - rzut y(x) + profil z(x) z obrotem i wspólną legendą."""
        if len(self.points) != 2:
            return

        print("\n" + "=" * 70)
        print("WIZUALIZACJA TRASY PROSTEJ")
        print("=" * 70)

        point_a, point_b = self.points
        distance_ab = geodesic(point_a, point_b).meters
        print(f"\n📏 Odległość A-B: {distance_ab:.2f} m")

        # Bounding box
        margines = 100
        margines_deg = margines / 111000
        lats = [point_a[0], point_b[0]]
        lons = [point_a[1], point_b[1]]

        bbox = {
            'north': max(lats) + margines_deg,
            'south': min(lats) - margines_deg,
            'east': max(lons) + margines_deg,
            'west': min(lons) - margines_deg
        }

        # Filtruj budynki
        buildings_in_bbox = None
        if self.buildings is not None:
            try:
                buildings_in_bbox = self.buildings.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]
                print(f"📦 Budynków w obszarze: {len(buildings_in_bbox)}")
            except Exception as e:
                print(f"⚠ Błąd filtrowania: {e}")

        # Konwersja do metrów (lokalny układ)
        ref_lat = np.mean(lats)
        ref_lon = np.mean(lons)

        # Punkty A i B w metrach
        xa, ya, _ = self.latlon_to_meters(point_a[0], point_a[1], 0, ref_lat, ref_lon)
        xb, yb, _ = self.latlon_to_meters(point_b[0], point_b[1], 0, ref_lat, ref_lon)

        # === OBRÓT: ustaw linię A–B poziomo ===
        dx = xb - xa
        dy = yb - ya
        angle = -np.arctan2(dy, dx)

        xa_rot_arr, ya_rot_arr = rotate_points(np.array([xa]), np.array([ya]), angle)
        xb_rot_arr, yb_rot_arr = rotate_points(np.array([xb]), np.array([yb]), angle)
        xa_rot, ya_rot = xa_rot_arr[0], ya_rot_arr[0]
        xb_rot, yb_rot = xb_rot_arr[0], yb_rot_arr[0]

        # Wspólne xlim
        x_min = min(xa_rot, xb_rot) - 50
        x_max = max(xa_rot, xb_rot) + 50

        # Uporządkowany zakres Y wokół linii; jak mapa (bez rozciągania)
        y_center = 0.5 * (ya_rot + yb_rot)
        # okno ±100…300 m, wg rozrzutu
        y_span = max(100.0, min(300.0, abs(yb_rot - ya_rot) * 0.25 + 100.0))
        y_min = y_center - y_span
        y_max = y_center + y_span

        # Figura – 2 wykresy pionowo, wspólna oś X
        fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)

        # === GÓRNY: Widok z góry Y(X) (po obrocie) ===
        ax_top.set_aspect('equal', adjustable='datalim')  # normalna mapa, bez deformacji
        ax_top.set_facecolor('#f9f9f9')

        # Rysuj budynki (po obrocie do układu XY)
        if buildings_in_bbox is not None and len(buildings_in_bbox) > 0:
            for _, building in buildings_in_bbox.iterrows():
                try:
                    geom = building.geometry
                    if geom.geom_type == 'Polygon':
                        coords = list(geom.exterior.coords)
                    elif geom.geom_type == 'MultiPolygon':
                        coords = list(geom.geoms[0].exterior.coords)
                    else:
                        continue

                    bldg_lons, bldg_lats = zip(*coords)
                    bx, by = [], []
                    for lat, lon in zip(bldg_lats, bldg_lons):
                        x, y, _ = self.latlon_to_meters(lat, lon, 0, ref_lat, ref_lon)
                        bx.append(x);
                        by.append(y)
                    bx = np.array(bx);
                    by = np.array(by)
                    bxr, byr = rotate_points(bx, by, angle)

                    ax_top.fill(bxr, byr, facecolor='orange',
                                edgecolor='darkgray', alpha=0.5, linewidth=0.5, zorder=10)
                except:
                    continue

        # Linia prosta A-B
        ax_top.plot([xa_rot, xb_rot], [ya_rot, yb_rot], 'b-', linewidth=3,
                    label=f'Linia prosta ({distance_ab:.0f}m)', zorder=100)

        # Punkty A i B
        ax_top.plot(xa_rot, ya_rot, 'o', color=COLORS['point_a'], markersize=15,
                    markeredgecolor='black', markeredgewidth=2, label='Punkt A', zorder=200)
        ax_top.plot(xb_rot, yb_rot, 'o', color=COLORS['point_b'], markersize=15,
                    markeredgecolor='black', markeredgewidth=2, label='Punkt B', zorder=200)
        ax_top.text(xa_rot, ya_rot, '  A', fontsize=14, fontweight='bold', zorder=201)
        ax_top.text(xb_rot, yb_rot, '  B', fontsize=14, fontweight='bold', zorder=201)

        ax_top.set_ylabel('Y [m]', fontsize=11)
        ax_top.set_title(f'Widok z góry Y(X) | Linia prosta: {distance_ab:.0f}m',
                         fontsize=13, fontweight='bold')
        ax_top.grid(True, alpha=0.3, linestyle='--')
        ax_top.set_xlim(x_min, x_max)
        ax_top.set_ylim(y_min, y_max)

        # === DOLNY: Profil Z(X) – budynki na linii prostej ===
        ax_bottom.set_facecolor('#f9f9f9')

        # Linia prosta na poziomie gruntu (0)
        ax_bottom.plot([xa_rot, xb_rot], [0, 0], 'b-', linewidth=3,
                       label='Linia prosta (poziom gruntu)', zorder=100)

        # Punkty A i B
        ax_bottom.scatter([xa_rot], [0], color=COLORS['point_a'],
                          s=150, marker='o', edgecolor='black', linewidth=2,
                          label='Punkt A', zorder=200)
        ax_bottom.scatter([xb_rot], [0], color=COLORS['point_b'],
                          s=150, marker='o', edgecolor='black', linewidth=2,
                          label='Punkt B', zorder=200)

        # Budynki przecinające linię A–B (profil)
        max_building_height = 10.0
        if buildings_in_bbox is not None and len(buildings_in_bbox) > 0:
            route_line = LineString([(point_a[1], point_a[0]), (point_b[1], point_b[0])])
            buffer_deg = 10.0 / 111000
            route_buffer = route_line.buffer(buffer_deg)

            buildings_drawn = 0
            for _, building in buildings_in_bbox.iterrows():
                if building.geometry.intersects(route_buffer):
                    try:
                        height = self._estimate_building_height(building)
                        max_building_height = max(max_building_height, height)

                        geom = building.geometry
                        if geom.geom_type == 'Polygon':
                            coords = list(geom.exterior.coords)
                        elif geom.geom_type == 'MultiPolygon':
                            coords = list(geom.geoms[0].exterior.coords)
                        else:
                            continue

                        bldg_lons, bldg_lats = zip(*coords)
                        bx, by = [], []
                        for lat, lon in zip(bldg_lats, bldg_lons):
                            x, y, _ = self.latlon_to_meters(lat, lon, 0, ref_lat, ref_lon)
                            bx.append(x);
                            by.append(y)
                        bx = np.array(bx);
                        by = np.array(by)
                        bxr, byr = rotate_points(bx, by, angle)

                        if len(bxr):
                            x_min_b = float(np.min(bxr))
                            x_max_b = float(np.max(bxr))
                            width = max(x_max_b - x_min_b, 5.0)
                            x_c = 0.5 * (x_min_b + x_max_b)

                            ax_bottom.add_patch(patches.Rectangle(
                                (x_c - width / 2, 0), width, height,
                                facecolor='orange', edgecolor='darkgray',
                                alpha=0.6, linewidth=1, zorder=50
                            ))
                            buildings_drawn += 1
                    except:
                        continue

            if buildings_drawn > 0:
                print(f"  - Budynków na linii prostej: {buildings_drawn}")

        # Linia gruntu
        ax_bottom.axhline(y=0, color='brown', linestyle='-', linewidth=2,
                          alpha=0.5, label='Poziom gruntu', zorder=10)

        # Oś X – podpis po prawej
        ax_bottom.set_xlabel('X [m]', fontsize=11, loc='right', labelpad=6)
        ax_bottom.set_ylabel('Wysokość [m]', fontsize=11)
        ax_bottom.set_title('Profil Z(X) - budynki na linii prostej', fontsize=13, fontweight='bold')
        ax_bottom.grid(True, alpha=0.3, linestyle='--')
        ax_bottom.set_ylim(bottom=-2, top=max_building_height * 1.15)
        ax_bottom.set_xlim(x_min, x_max)

        # ✅ Wspólna legenda pod spodem
        handles = [
            plt.Line2D([0], [0], color='blue', lw=3, label='Linia prosta'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['point_a'],
                       markeredgecolor='black', markersize=12, label='Punkt A'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['point_b'],
                       markeredgecolor='black', markersize=12, label='Punkt B'),
        ]
        fig.legend(handles=handles, loc='lower center', ncol=3, fontsize=11, frameon=False)
        fig.subplots_adjust(bottom=0.16)

        plt.tight_layout()
        plt.show()

        return {
            'point_a': point_a,
            'point_b': point_b,
            'distance': distance_ab,
            'bbox': bbox,
            'buildings_in_area': len(buildings_in_bbox) if buildings_in_bbox is not None else 0
        }

    def plan_drone_path(self):
        """Zaplanuj ścieżkę drona używając 3D A*."""
        if len(self.points) != 2:
            return None

        print("\n" + "=" * 70)
        print("PARAMETRY LOTU")
        print("=" * 70)

        altitude_start = config.altitude_start
        altitude_cruise_min = config.altitude_cruise_min
        altitude_cruise_max = config.altitude_cruise_max
        altitude_end = config.altitude_end
        avoid_distance = config.avoid_distance

        print(f"\n✓ Start: {altitude_start:.1f}m | Przelot: {altitude_cruise_min:.1f}-{altitude_cruise_max:.1f}m | Koniec: {altitude_end:.1f}m")

        # Filtruj budynki w obszarze
        point_a, point_b = self.points
        margines_deg = 500 / 111000  # 500m margines
        lats = [point_a[0], point_b[0]]
        lons = [point_a[1], point_b[1]]

        buildings_filtered = None
        if self.buildings is not None:
            try:
                buildings_filtered = self.buildings.cx[
                    min(lons) - margines_deg:max(lons) + margines_deg,
                    min(lats) - margines_deg:max(lats) + margines_deg
                ]
                print(f"📦 Budynków do sprawdzenia: {len(buildings_filtered)}")
            except Exception as e:
                print(f"⚠ Błąd filtrowania budynków: {e}")
                buildings_filtered = self.buildings

        # Stwórz planner i zaplanuj ścieżkę
        self.path_planner = PathPlanner3D(buildings_filtered, config)

        result = self.path_planner.plan_path(
            point_a[0], point_a[1],  # lat_A, lon_A
            point_b[0], point_b[1]   # lat_B, lon_B
        )

        if result is None:
            print("\n✗ Nie udało się zaplanować ścieżki!")
            return None

        # Konwertuj wynik do formatu (lat, lon, alt)
        self.drone_path = result['path_latlon']
        self.drone_path_distance = result['distance']

        # Eksportuj do pliku dla trajectory.py
        self.export_path_for_simulation(result)

        return self.drone_path, self.drone_path_distance

    def export_path_for_simulation(self, path_result):
        """Eksportuj zaplanowaną ścieżkę do pliku dla symulacji LQR."""
        if path_result is None:
            return

        path_meters = path_result['path_meters']

        # Wyciągnij X, Y, Z
        X_ref = path_meters[:, 0]
        Y_ref = path_meters[:, 1]
        Z_ref = -path_meters[:, 2]  # Konwersja: altituda dodatnia → NED negative

        # Zapisz do pliku
        np.savez('planned_path.npz',
                 X_ref=X_ref,
                 Y_ref=Y_ref,
                 Z_ref=Z_ref)

        print(f"\n✓ Eksportowano trajektorię do 'planned_path.npz'")
        print(f"  - Punktów: {len(X_ref)}")
        print(f"  - Zakres X: [{X_ref.min():.1f}, {X_ref.max():.1f}] m")
        print(f"  - Zakres Y: [{Y_ref.min():.1f}, {Y_ref.max():.1f}] m")
        print(f"  - Zakres Z (NED): [{Z_ref.min():.1f}, {Z_ref.max():.1f}] m")
        print(f"  - Wysokości AGL: [{-Z_ref.min():.1f}, {-Z_ref.max():.1f}] m")

    def visualize_drone_path(self):
        """Wizualizuj zaplanowaną ścieżkę - rzut y(x) + profil z(s) (obwiednia zabudowy pod krzywą), wspólna legenda i normalna mapa."""
        if not self.drone_path:
            print("⚠ Brak ścieżki")
            return

        print("\n" + "=" * 70)
        print("WIZUALIZACJA TRASY DRONA")
        print("=" * 70)

        lats = [point[0] for point in self.drone_path]
        lons = [point[1] for point in self.drone_path]
        alts = [point[2] for point in self.drone_path]
        point_a, point_b = self.points

        # Bbox tylko obszar trasy
        margines = 50
        margines_deg = margines / 111000
        bbox = {
            'north': max(lats) + margines_deg,
            'south': min(lats) - margines_deg,
            'east': max(lons) + margines_deg,
            'west': min(lons) - margines_deg
        }

        # Budynki do rysunku
        buildings_in_bbox = None
        if self.buildings is not None:
            try:
                buildings_in_bbox = self.buildings.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]
                print(f"📦 Budynków w obszarze: {len(buildings_in_bbox)}")
            except:
                pass

        # Lokalne metry
        ref_lat = np.mean(lats)
        ref_lon = np.mean(lons)

        path_x, path_y = [], []
        for lat, lon in zip(lats, lons):
            x, y, _ = self.latlon_to_meters(lat, lon, 0, ref_lat, ref_lon)
            path_x.append(x)
            path_y.append(y)
        path_x = np.array(path_x)
        path_y = np.array(path_y)

        xa, ya, _ = self.latlon_to_meters(point_a[0], point_a[1], 0, ref_lat, ref_lon)
        xb, yb, _ = self.latlon_to_meters(point_b[0], point_b[1], 0, ref_lat, ref_lon)

        # Obrót aby A–B było poziomo (dla górnego wykresu)
        dx = xb - xa
        dy = yb - ya
        angle = -np.arctan2(dy, dx)
        path_x_rot, path_y_rot = rotate_points(path_x, path_y, angle)
        xa_rot, ya_rot = rotate_points(np.array([xa]), np.array([ya]), angle)
        xb_rot, yb_rot = rotate_points(np.array([xb]), np.array([yb]), angle)
        xa_rot, ya_rot = xa_rot[0], ya_rot[0]
        xb_rot, yb_rot = xb_rot[0], yb_rot[0]

        # Wspólne xlim (dla górnego wykresu)
        x_min = min(path_x_rot.min(), xa_rot, xb_rot) - 20
        x_max = max(path_x_rot.max(), xa_rot, xb_rot) + 20

        # „Mapa” – okno Y ±100…300 m wokół linii (dla górnego wykresu)
        y_center = 0.5 * (ya_rot + yb_rot)
        raw_span = max(abs(path_y_rot.min() - y_center), abs(path_y_rot.max() - y_center))
        y_span = max(100.0, min(300.0, raw_span + 50.0))  # [m] – użyjemy tego też jako szerokości korytarza profilu
        y_min = y_center - y_span
        y_max = y_center + y_span

        fig, (ax_top, ax_bottom) = plt.subplots(2, 1, figsize=(14, 10), sharex=False)

        # =========================
        # Górny: widok z góry (po obrocie) – BEZ ZMIAN
        # =========================
        ax_top.set_aspect('equal', adjustable='datalim')
        ax_top.set_facecolor('#f9f9f9')

        if buildings_in_bbox is not None:
            for _, building in buildings_in_bbox.iterrows():
                try:
                    geom = building.geometry
                    if geom.geom_type == 'Polygon':
                        coords = list(geom.exterior.coords)
                    elif geom.geom_type == 'MultiPolygon':
                        coords = list(geom.geoms[0].exterior.coords)
                    else:
                        continue
                    bldg_lons, bldg_lats = zip(*coords)
                    bx, by = [], []
                    for lat, lon in zip(bldg_lats, bldg_lons):
                        x, y, _ = self.latlon_to_meters(lat, lon, 0, ref_lat, ref_lon)
                        bx.append(x)
                        by.append(y)
                    bx = np.array(bx)
                    by = np.array(by)
                    bxr, byr = rotate_points(bx, by, angle)
                    ax_top.fill(bxr, byr, facecolor='orange',
                                edgecolor='darkgray', alpha=0.5, linewidth=0.5, zorder=10)
                except:
                    continue

        ax_top.plot([xa_rot, xb_rot], [ya_rot, yb_rot], 'b--', linewidth=2, alpha=0.3,
                    label='Linia prosta', zorder=50)
        ax_top.plot(path_x_rot, path_y_rot, 'r-', linewidth=3, label='Trasa drona', zorder=100)
        ax_top.plot(xa_rot, ya_rot, 'o', color=COLORS['point_a'], markersize=15,
                    markeredgecolor='black', markeredgewidth=2, label='Start (A)', zorder=200)
        ax_top.plot(xb_rot, yb_rot, 'o', color=COLORS['point_b'], markersize=15,
                    markeredgecolor='black', markeredgewidth=2, label='Cel (B)', zorder=200)
        ax_top.text(xa_rot, ya_rot, '  A', fontsize=14, fontweight='bold', zorder=201)
        ax_top.text(xb_rot, yb_rot, '  B', fontsize=14, fontweight='bold', zorder=201)

        ax_top.set_ylabel('Y [m]', fontsize=11)
        ax_top.set_title(f'Widok z góry Y(X) | {len(self.drone_path)} waypoints',
                         fontsize=13, fontweight='bold')
        ax_top.grid(True, alpha=0.3, linestyle='--')
        ax_top.set_xlim(x_min, x_max)
        ax_top.set_ylim(y_min, y_max)

        # ==========================================
        # Dolny: NOWY PROFIL Z(s) + zabudowa pod krzywą
        # ==========================================
        ax_bottom.set_facecolor('#f9f9f9')

        # --- [NOWE] Profil po długości łuku s (dokładnie pod trasą) ---
        dxs = np.diff(path_x)
        dys = np.diff(path_y)
        ds = np.hypot(dxs, dys)
        s = np.concatenate([[0.0], np.cumsum(ds)])  # s[i] – dystans od startu
        alts_arr = np.array(alts, dtype=float)

        # Wysokość budynków DOKŁADNIE pod punktem trasy
        bldg_h = np.zeros_like(s)
        if buildings_in_bbox is not None and len(buildings_in_bbox) > 0:
            from shapely.geometry import Point
            # bardzo mały promień tolerancji (na numerykę/brzeg poligonu)
            tol_m = 1.5
            tol_deg = tol_m / 111000.0

            for i, (lat_i, lon_i) in enumerate(zip(lats, lons)):
                # lokalne okno, by nie iterować po wszystkich budynkach
                try:
                    around = buildings_in_bbox.cx[
                        lon_i - tol_deg: lon_i + tol_deg,
                        lat_i - tol_deg: lat_i + tol_deg
                    ]
                except Exception:
                    around = buildings_in_bbox

                if len(around) == 0:
                    continue

                p = Point(lon_i, lat_i)
                hmax = 0.0
                for _, b in around.iterrows():
                    try:
                        geom = b.geometry
                        if geom is None:
                            continue
                        # uznaj punkt za "pod budynkiem", jeśli jest wewnątrz
                        # albo bardzo blisko krawędzi (<= tol_deg)
                        if geom.contains(p) or geom.distance(p) <= tol_deg:
                            hmax = max(hmax, self._estimate_building_height(b))
                    except Exception:
                        continue
                bldg_h[i] = hmax
        else:
            bldg_h[:] = 0.0

        # --- [RYSOWANIE DOLNEGO WYKRESU] ---
        ax_bottom.clear()
        ax_bottom.set_facecolor('#f9f9f9')

        # Budynki jako "podkład" od 0 do bldg_h(s)
        ax_bottom.fill_between(s, 0.0, bldg_h, alpha=0.35, label='Zabudowa pod trasą', zorder=20)

        # Trasa Z(s)
        ax_bottom.plot(s, alts_arr, 'r-', linewidth=3, label='Trasa drona', zorder=100)

        # Znaczniki A/B
        ax_bottom.scatter([s[0]], [alts_arr[0]], color=COLORS['point_a'],
                          s=150, marker='o', edgecolor='black', linewidth=2,
                          label='Start (A)', zorder=200)
        ax_bottom.scatter([s[-1]], [alts_arr[-1]], color=COLORS['point_b'],
                          s=150, marker='o', edgecolor='black', linewidth=2,
                          label='Cel (B)', zorder=200)

        ax_bottom.set_xlabel('Dystans wzdłuż trasy s [m]', fontsize=11, loc='right', labelpad=6)
        ax_bottom.set_ylabel('Wysokość [m]', fontsize=11)
        ax_bottom.grid(True, alpha=0.3, linestyle='--')
        ax_bottom.set_ylim(
            bottom=0,
            top=max(float(alts_arr.max()) if alts_arr.size else 0.0,
                    float(bldg_h.max()) if bldg_h.size else 0.0) * 1.15 + 1e-6
        )
        ax_bottom.set_xlim(0, float(s[-1]) if s.size else 1.0)

        # Adnotacja max wysokości lotu
        if alts_arr.size:
            max_alt = float(alts_arr.max())
            max_idx = int(alts_arr.argmax())
            ax_bottom.annotate(f'Max: {max_alt:.1f}m',
                               xy=(s[max_idx], max_alt),
                               xytext=(10, 10), textcoords='offset points',
                               fontsize=10, fontweight='bold',
                               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7),
                               arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

        # 3) Rysowanie: budynki jako tło (0 -> bldg_h), a nad tym krzywa lotu Z(s)
        ax_bottom.fill_between(s, 0.0, bldg_h, alpha=0.35, label='Zabudowa (obwiednia)', zorder=20)
        ax_bottom.plot(s, alts_arr, 'r-', linewidth=3, label='Trasa drona', zorder=100)

        # Znaczniki A/B
        ax_bottom.scatter([s[0]], [alts_arr[0]], color=COLORS['point_a'],
                          s=150, marker='o', edgecolor='black', linewidth=2,
                          label='Start (A)', zorder=200)
        ax_bottom.scatter([s[-1]], [alts_arr[-1]], color=COLORS['point_b'],
                          s=150, marker='o', edgecolor='black', linewidth=2,
                          label='Cel (B)', zorder=200)

        # Opisy osi, siatka, zakresy
        ax_bottom.set_xlabel('Dystans wzdłuż trasy s [m]', fontsize=11, loc='right', labelpad=6)
        ax_bottom.set_ylabel('Wysokość [m]', fontsize=11)
        ax_bottom.grid(True, alpha=0.3, linestyle='--')
        ax_bottom.set_ylim(bottom=0, top=max(alts_arr.max(), bldg_h.max()) * 1.15 if len(s) > 0 else 10)
        ax_bottom.set_xlim(0, s[-1] if len(s) > 0 else 1)

        # Adnotacja max wysokości lotu
        if len(alts_arr) > 0:
            max_alt = float(alts_arr.max())
            max_idx = int(alts_arr.argmax())
            ax_bottom.annotate(f'Max: {max_alt:.1f}m',
                               xy=(s[max_idx], max_alt),
                               xytext=(10, 10), textcoords='offset points',
                               fontsize=10, fontweight='bold',
                               bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7),
                               arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

        # Statystyki
        print(f"\n📊 Statystyki:")
        print(f"  - Waypoints: {len(self.drone_path)}")
        print(f"  - Długość trasy: {self.drone_path_distance:.2f} m")
        print(f"  - Wysokość: {min(alts):.1f}-{max(alts):.1f}m (śr. {np.mean(alts):.1f}m)")
        straight_dist = np.hypot(xb - xa, yb - ya)
        print(f"  - Linia prosta: {straight_dist:.2f} m")
        print(f"  - Wydłużenie trasy: {(self.drone_path_distance / straight_dist - 1) * 100:.1f}%")

        # ✅ Wspólna legenda
        handles = [
            plt.Line2D([0], [0], color='red', lw=3, label='Trasa drona'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['point_a'],
                       markeredgecolor='black', markersize=12, label='Start (A)'),
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['point_b'],
                       markeredgecolor='black', markersize=12, label='Cel (B)'),
            plt.Line2D([0], [0], color='orange', lw=8, alpha=0.35, label='Zabudowa (obwiednia)'),
        ]
        fig.legend(handles=handles, loc='lower center', ncol=4, fontsize=11, frameon=False)
        fig.subplots_adjust(bottom=0.16)

        plt.tight_layout()
        plt.show()

    def animate_combined_view(self):
        """Animacja: widok z góry (XY) + profil (ZX)."""
        if not self.drone_path:
            print("⚠ Brak ścieżki")
            return

        print("\n" + "=" * 70)
        print("ANIMACJA 2D")
        print("=" * 70)

        lats = [point[0] for point in self.drone_path]
        lons = [point[1] for point in self.drone_path]
        alts = [point[2] for point in self.drone_path]

        ref_lat = np.mean(lats)
        ref_lon = np.mean(lons)

        # Konwertuj trasę
        path_x, path_y, path_z = [], [], []
        for lat, lon, alt in zip(lats, lons, alts):
            x, y, z = self.latlon_to_meters(lat, lon, alt, ref_lat, ref_lon)
            path_x.append(x)
            path_y.append(y)
            path_z.append(z)

        path_x = np.array(path_x)
        path_y = np.array(path_y)
        path_z = np.array(path_z)

        # Dystans
        dist = np.zeros(len(path_x))
        for i in range(1, len(path_x)):
            dist[i] = dist[i - 1] + np.hypot(path_x[i] - path_x[i - 1], path_y[i] - path_y[i - 1])

        # Budynki
        building_shapes = []
        buildings_in_bbox = None  # Zachowaj referencję dla profilu

        if self.buildings is not None:
            try:
                margines = 200
                margines_deg = margines / 111000
                bbox = {
                    'north': max(lats) + margines_deg,
                    'south': min(lats) - margines_deg,
                    'east': max(lons) + margines_deg,
                    'west': min(lons) - margines_deg
                }
                buildings_in_bbox = self.buildings.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]

                for _, building in buildings_in_bbox.iterrows():
                    try:
                        geom = building.geometry
                        if geom.geom_type == 'Polygon':
                            coords = list(geom.exterior.coords)
                        elif geom.geom_type == 'MultiPolygon':
                            coords = list(geom.geoms[0].exterior.coords)
                        else:
                            continue

                        bldg_lons, bldg_lats = zip(*coords)
                        bldg_x, bldg_y = [], []
                        for lat, lon in zip(bldg_lats, bldg_lons):
                            x, y, _ = self.latlon_to_meters(lat, lon, 0, ref_lat, ref_lon)
                            bldg_x.append(x)
                            bldg_y.append(y)

                        building_shapes.append({'x': bldg_x, 'y': bldg_y})
                    except:
                        continue

                print(f"  - Budynków: {len(building_shapes)}")
            except Exception as e:
                print(f"  ⚠ Błąd: {e}")

        # Animacja
        fps = 30
        speed_multiplier = 5.0
        total_time = (dist[-1] / 5.0) / speed_multiplier
        total_frames = int(total_time * fps)

        print(f"  - Czas: {total_time:.1f}s, klatek: {total_frames}")

        # Interpolacja
        t = np.linspace(0, 1, total_frames)
        drone_x = np.interp(t, np.linspace(0, 1, len(path_x)), path_x)
        drone_y = np.interp(t, np.linspace(0, 1, len(path_y)), path_y)
        drone_s = np.interp(t, np.linspace(0, 1, len(dist)), dist)
        drone_z = np.interp(t, np.linspace(0, 1, len(path_z)), path_z)

        # Figura
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

        # AX1: XY
        ax1.set_aspect('equal')
        ax1.set_facecolor('#f0f0f0')
        ax1.grid(True, alpha=0.3, linestyle='--')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title('Widok z góry (XY)', fontweight='bold')

        drone_pt1, = ax1.plot([], [], 'ro', markersize=12, markeredgecolor='black',
                              markeredgewidth=2, zorder=1000)
        path_line1, = ax1.plot([], [], 'b-', linewidth=2, alpha=0.5, zorder=100)
        traveled1, = ax1.plot([], [], 'g-', linewidth=3, alpha=0.7, zorder=200)

        # AX2: ZX
        ax2.set_facecolor('#f9f9f9')
        ax2.grid(True, alpha=0.3, linestyle='--')
        ax2.set_xlabel('Dystans [m]')
        ax2.set_ylabel('Wysokość [m]')
        ax2.set_title('Profil (ZX)', fontweight='bold')

        ax2.plot(dist, path_z, 'b-', linewidth=2, alpha=0.6, zorder=10)
        drone_pt2, = ax2.plot([], [], 'ro', markersize=10, markeredgecolor='black',
                              markeredgewidth=1.5, zorder=20)

        # === DODAJ BUDYNKI NA PROFILU ZX ===
        if buildings_in_bbox is not None and len(buildings_in_bbox) > 0:
            print("  - Rysowanie budynków na profilu...")

            # Linia trasy do sprawdzania kolizji
            route_line = LineString([(lon, lat) for lat, lon in zip(lats, lons)])
            buffer_deg = 10.0 / 111000  # 10m bufor
            route_buffer = route_line.buffer(buffer_deg)

            buildings_drawn = 0
            for _, building in buildings_in_bbox.iterrows():
                if building.geometry.intersects(route_buffer):
                    try:
                        height = self._estimate_building_height(building)

                        # Znajdź które punkty trasy są nad budynkiem
                        x_ranges = []
                        for i in range(len(lats)):
                            point = Point(lons[i], lats[i])
                            if building.geometry.contains(point) or building.geometry.distance(point) < buffer_deg:
                                x_ranges.append(dist[i])

                        if x_ranges:
                            x_min, x_max = min(x_ranges), max(x_ranges)
                            width = max(x_max - x_min, 5.0)  # Min 5m szerokości
                            x_center = (x_min + x_max) / 2

                            # Rysuj budynek jako prostokąt
                            rect = patches.Rectangle(
                                (x_center - width / 2, 0),
                                width, height,
                                facecolor='orange',
                                edgecolor='darkgray',
                                alpha=0.6,
                                linewidth=1,
                                zorder=50
                            )
                            ax2.add_patch(rect)
                            buildings_drawn += 1
                    except:
                        continue

            print(f"    ✓ Narysowano {buildings_drawn} budynków na profilu")

        ax2.set_xlim(0, dist[-1] * 1.02)
        ax2.set_ylim(bottom=0, top=max(path_z) * 1.15)

        info_text = fig.text(0.5, 0.02, '', ha='center', fontsize=11, family='monospace',
                             bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

        def init():
            drone_pt1.set_data([], [])
            path_line1.set_data([], [])
            traveled1.set_data([], [])
            drone_pt2.set_data([], [])
            return drone_pt1, path_line1, traveled1, drone_pt2, info_text

        def update(frame):
            dx, dy = drone_x[frame], drone_y[frame]
            ds, dz = drone_s[frame], drone_z[frame]

            # XY
            drone_pt1.set_data([dx], [dy])

            look_ahead = 80
            look_ahead_frames = min(int(look_ahead / 5), total_frames - frame - 1)
            if look_ahead_frames > 0:
                path_line1.set_data(drone_x[frame:frame + look_ahead_frames],
                                    drone_y[frame:frame + look_ahead_frames])
            else:
                path_line1.set_data([], [])

            if frame > 0:
                traveled1.set_data(drone_x[0:frame], drone_y[0:frame])

            for patch in ax1.patches[:]:
                patch.remove()

            view_range = 100
            for bldg in building_shapes:
                bldg_x = np.array(bldg['x'])
                bldg_y = np.array(bldg['y'])
                dist_to_drone = np.hypot(np.mean(bldg_x) - dx, np.mean(bldg_y) - dy)

                if dist_to_drone < view_range * 1.5:
                    poly = plt.Polygon(list(zip(bldg_x, bldg_y)),
                                       facecolor='#808080', edgecolor='#404040',
                                       alpha=0.7, linewidth=1, zorder=50)
                    ax1.add_patch(poly)

            ax1.set_xlim(dx - view_range, dx + view_range)
            ax1.set_ylim(dy - view_range, dy + view_range)

            # ZX
            drone_pt2.set_data([ds], [dz])

            view_half_width = 80
            ax2.set_xlim(max(0, ds - view_half_width), min(dist[-1], ds + view_half_width))

            progress = (frame / total_frames) * 100
            info_text.set_text(
                f'Postęp: {progress:.1f}% | Dystans: {ds:.1f}m | Wysokość: {dz:.1f}m'
            )

            return drone_pt1, path_line1, traveled1, drone_pt2, info_text

        print("\n🎬 Animacja...")
        anim = animation.FuncAnimation(
            fig, update, init_func=init,
            frames=total_frames, interval=1000 / fps,
            blit=False, repeat=True
        )

        plt.tight_layout()
        plt.show()

    def run(self):
        """Uruchom pełny workflow."""
        if not self.load_data():
            return None

        self.create_base_map()
        self.setup_interactive_mode()

        plt.tight_layout()
        plt.show()

        if len(self.points) == 2:
            result = self.visualize_route_and_bbox()

            plan = input("\nZaplanować ścieżkę? (t/n): ").strip().lower()
            if plan in ['t', 'tak', 'yes', 'y']:
                self.plan_drone_path()

                if self.drone_path:
                    self.visualize_drone_path()

                    animate = input("\nAnimacja? (t/n): ").strip().lower()
                    if animate in ['t', 'tak', 'yes', 'y']:
                        self.animate_combined_view()

            print("\n" + "=" * 70)
            print("GOTOWE!")
            print(f"  A = {self.points[0]}")
            print(f"  B = {self.points[1]}")
            print(f"  Odległość = {result['distance']:.2f} m")
            print(f"  Budynków = {result['buildings_in_area']}")
            print("=" * 70)

            return result
        else:
            print("\n⚠ Nie wybrano punktów")
            return None


def drone_pathing(point_a=None, point_b=None, headless=False):
    """
    Jeśli point_a i point_b są podane, użyj ich bez interaktywnego wyboru.
    headless=True -> bez otwierania okien z wizualizacjami (np. w trybie batch).
    """
    selector = WarsawMapSelector()

    # 1) Ładowanie danych (jak w .run())
    if not selector.load_data():
        return None

    if point_a is not None and point_b is not None:
        # Użyj już wybranych punktów
        selector.points = [tuple(point_a), tuple(point_b)]

        # Opcjonalnie pokaż podgląd linii prostej i bbox
        if headless:
            selector.create_base_map()
            selector.visualize_route_and_bbox()

        # Planowanie + eksport
        selector.plan_drone_path()

        # Opcjonalne wizualizacje trasy
        if not headless and selector.drone_path:
            selector.visualize_drone_path()
            selector.animate_combined_view()  # jeśli chcesz animację od razu

        # Zwracamy to samo API co wcześniej (wynik .run()), ale bez wymuszania klikania
        return {
            'point_a': selector.points[0],
            'point_b': selector.points[1],
            'distance': selector.drone_path_distance if selector.drone_path_distance else None
        }

    # --- STARY, interaktywny workflow, gdy punktów nie podano ---
    return selector.run()


if __name__ == "__main__":
    input_parameters()
    drone_pathing()