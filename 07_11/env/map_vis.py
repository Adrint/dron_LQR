"""
Interaktywna wizualizacja Warszawy z wyborem punktów A i B
===========================================================
Autor: Warsaw Drone Simulation Project
Inspiracja: https://github.com/gboeing/osmnx-examples

Ten skrypt:
- Ładuje dane Warszawy (drogi, budynki, drzewa, mosty)
- Wyświetla mapę 2D z obiektami i wysokościami
- Umożliwia interaktywny wybór punktów A i B
- Wyznacza trasę między punktami
- Wizualizuje obszar analizy

Użycie:
    1. Użyj przycisków zoom/pan do nawigacji po mapie
    2. Wciśnij SPACJĘ aby włączyć tryb zaznaczania
    3. Kliknij punkt A (zielony), potem punkt B (czerwony)
    4. Mapa pokaże trasę i obszar analizy
"""

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
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from typing import Optional, List
import warnings
warnings.filterwarnings('ignore')

# Import planera ścieżki
import sys
sys.path.append(str(Path(__file__).parent))
try:
    from path_planning_module import DronePathPlanner, Waypoint3D
except ImportError:
    DronePathPlanner = None
    Waypoint3D = None
    print("⚠ Moduł planowania ścieżki niedostępny")

# Import planera ścieżki
try:
    from drone_path_planner import DronePathPlanner, PathPlannerConfig, visualize_path_3d
except ImportError:
    # Jeśli nie ma modułu, spróbuj z prefixem
    import sys
    sys.path.insert(0, str(Path(__file__).parent))
    from drone_path_planner import DronePathPlanner, PathPlannerConfig, visualize_path_3d

# Konfiguracja
ox.settings.use_cache = True
DATA_DIR = Path("data")

# Kolory wizualizacji
COLORS = {
    'road': '#999999',
    'building': '#FFA500',
    'tree': '#228B22',
    'bridge': '#4169E1',
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
        """Inicjalizacja selektora mapy."""
        self.G = None
        self.buildings = None
        self.trees = None
        self.bridges = None

        self.fig = None
        self.ax = None

        self.points = []
        self.selection_mode = False

        self.route_nodes = None
        self.route = None

        # Planowanie ścieżki drona
        self.drone_path = None
        self.drone_path_distance = None
        self.path_planner = None

    def load_data(self):
        """Załaduj wszystkie dane z plików."""
        print("\n[1/4] Ładowanie danych z plików...")

        # Sieć drogowa
        street_network_path = DATA_DIR / "warsaw_street_network.pkl"
        if street_network_path.exists():
            with open(street_network_path, "rb") as f:
                self.G = pickle.load(f)
            print(f"      ✓ Sieć drogowa: {len(self.G.nodes):,} węzłów")
        else:
            print(f"      ✗ Brak pliku: {street_network_path}")
            print(f"      → Uruchom najpierw: 01_download_warsaw_data.py")
            return False

        # Budynki
        buildings_path = DATA_DIR / "warsaw_buildings.pkl"
        if buildings_path.exists():
            self.buildings = pd.read_pickle(buildings_path)
            print(f"      ✓ Budynki: {len(self.buildings):,}")
        else:
            print(f"      ⚠ Brak budynków (opcjonalne)")

        # Drzewa
        trees_path = DATA_DIR / "warsaw_trees.pkl"
        if trees_path.exists():
            self.trees = pd.read_pickle(trees_path)
            print(f"      ✓ Drzewa: {len(self.trees):,}")
        else:
            print(f"      ⚠ Brak drzew (opcjonalne)")

        # Mosty
        bridges_path = DATA_DIR / "warsaw_bridges.pkl"
        if bridges_path.exists():
            self.bridges = pd.read_pickle(bridges_path)
            print(f"      ✓ Mosty: {len(self.bridges):,}")
        else:
            print(f"      ⚠ Brak mostów (opcjonalne)")

        return True

    def create_base_map(self):
        """Stwórz podstawową mapę z siecią drogową."""
        print("\n[2/4] Tworzenie podstawowej mapy...")

        # Rysuj sieć drogową
        self.fig, self.ax = ox.plot_graph(
            self.G,
            node_size=0,
            edge_linewidth=0.5,
            edge_color=COLORS['road'],
            bgcolor='white',
            show=False,
            close=False
        )

        print(f"      ✓ Sieć drogowa wyrenderowana")

        return self.fig, self.ax

    def add_buildings_layer(self):
        """Dodaj warstwę budynków z kolorowaniem według wysokości."""
        if self.buildings is None:
            return

        print("\n[3/4] Dodawanie warstwy budynków...")

        # Przygotuj dane o wysokościach
        buildings_plot = self.buildings.copy()

        # Spróbuj wyekstrahować wysokości z różnych kolumn
        heights = []
        for idx, building in buildings_plot.iterrows():
            height = None

            # Próbuj pobrać z kolumny 'height'
            if 'height' in building.index and not pd.isna(building['height']):
                try:
                    h = str(building['height'])
                    # Usuń jednostki i konwertuj
                    h = h.replace('m', '').replace('M', '').strip()
                    height = float(h)
                except:
                    pass

            # Jeśli brak, szacuj z liczby pięter
            if height is None and 'building:levels' in building.index:
                try:
                    levels = int(building['building:levels'])
                    height = levels * 3.0  # Szacunek: 3m na piętro
                except:
                    pass

            # Domyślna wysokość
            if height is None:
                height = 10.0  # Domyślnie 10m

            heights.append(height)

        buildings_plot['estimated_height'] = heights

        # Rysuj budynki z kolorowaniem
        buildings_plot.plot(
            ax=self.ax,
            column='estimated_height',
            cmap='YlOrRd',
            alpha=0.6,
            edgecolor='darkgray',
            linewidth=0.3,
            legend=True,
            legend_kwds={'label': 'Wysokość budynku [m]', 'shrink': 0.8}
        )

        print(f"      ✓ Wyrenderowano {len(buildings_plot):,} budynków")
        print(f"      ℹ Zakres wysokości: {min(heights):.1f}m - {max(heights):.1f}m")

    def add_trees_layer(self):
        """Dodaj warstwę drzew."""
        if self.trees is None:
            return

        print("\n[4/4] Dodawanie warstwy drzew i mostów...")

        # Rysuj drzewa jako małe punkty
        self.trees.plot(
            ax=self.ax,
            color=COLORS['tree'],
            markersize=2,
            alpha=0.4,
            label='Drzewa'
        )

        print(f"      ✓ Wyrenderowano {len(self.trees):,} drzew")

    def add_bridges_layer(self):
        """Dodaj warstwę mostów."""
        if self.bridges is None:
            return

        # Rysuj mosty
        self.bridges.plot(
            ax=self.ax,
            color=COLORS['bridge'],
            linewidth=2,
            alpha=0.5,
            label='Mosty'
        )

        print(f"      ✓ Wyrenderowano {len(self.bridges):,} mostów")

    def setup_interactive_mode(self):
        """Skonfiguruj tryb interaktywny."""
        print("\n" + "=" * 70)
        print("INSTRUKCJA OBSŁUGI")
        print("=" * 70)
        print("1. Użyj przycisków ZOOM/PAN aby znaleźć interesujący obszar")
        print("2. Wciśnij SPACJĘ aby włączyć tryb zaznaczania punktów")
        print("3. Kliknij na mapie aby oznaczyć punkt A (zielony)")
        print("4. Kliknij ponownie aby oznaczyć punkt B (czerwony)")
        print("5. Program automatycznie wyznaczy trasę i zamknie okno")
        print("=" * 70)

        # Podłącz obsługę zdarzeń
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        # Ustaw tytuł
        self.ax.set_title(
            "TRYB ZOOM: Użyj przycisków zoom/pan, potem wciśnij SPACJĘ",
            color='blue',
            fontweight='bold',
            fontsize=12
        )

        print("\n🔵 TRYB ZOOM aktywny - używaj przycisków zoom/pan")
        print("   Wciśnij SPACJĘ aby przełączyć na tryb zaznaczania\n")

    def on_click(self, event):
        """Obsługa kliknięcia myszką."""
        # Sprawdź czy toolbar jest aktywny (zoom/pan)
        if self.fig.canvas.toolbar.mode != '':
            return

        # Sprawdź czy tryb zaznaczania jest włączony
        if not self.selection_mode:
            return

        if event.xdata is not None and event.ydata is not None:
            lon = event.xdata
            lat = event.ydata

            self.points.append((lat, lon))

            color = COLORS['point_a'] if len(self.points) == 1 else COLORS['point_b']
            label = 'A' if len(self.points) == 1 else 'B'

            # Rysuj punkt
            self.ax.plot(
                lon, lat, 'o',
                color=color,
                markersize=15,
                markeredgecolor='black',
                markeredgewidth=2,
                zorder=1000
            )

            # Dodaj etykietę
            self.ax.text(
                lon, lat, f'  {label}',
                fontsize=14,
                fontweight='bold',
                zorder=1001
            )

            print(f"✓ Zaznaczono punkt {label}: lat={lat:.6f}, lon={lon:.6f}")

            plt.draw()

            if len(self.points) == 2:
                print("\n✓ Wybrano oba punkty! Wyznaczanie trasy...")
                plt.close()

    def on_key(self, event):
        """Obsługa naciśnięcia klawisza."""
        if event.key == ' ':  # Spacja
            self.selection_mode = not self.selection_mode

            if self.selection_mode:
                print("\n🟢 TRYB ZAZNACZANIA - klikaj punkty A i B")
                self.ax.set_title(
                    "TRYB ZAZNACZANIA: Wybierz punkty A (zielony) i B (czerwony)",
                    color='green',
                    fontweight='bold',
                    fontsize=12
                )
            else:
                print("\n🔵 TRYB ZOOM - używaj przycisków zoom/pan")
                self.ax.set_title(
                    "TRYB ZOOM: Użyj przycisków zoom/pan, potem wciśnij SPACJĘ",
                    color='blue',
                    fontweight='bold',
                    fontsize=12
                )

            plt.draw()

    def calculate_route(self):
        """Wyznacz trasę między punktami A i B."""
        if len(self.points) != 2:
            return None

        point_a, point_b = self.points

        print("\n" + "=" * 70)
        print("WYZNACZANIE TRASY")
        print("=" * 70)

        # Znajdź najbliższe węzły w grafie
        node_a = ox.distance.nearest_nodes(self.G, point_a[1], point_a[0])
        node_b = ox.distance.nearest_nodes(self.G, point_b[1], point_b[0])

        print(f"\nPunkt A: {point_a}")
        print(f"  → Najbliższy węzeł: {node_a}")
        print(f"\nPunkt B: {point_b}")
        print(f"  → Najbliższy węzeł: {node_b}")

        # Oblicz najkrótszą ścieżkę
        try:
            route = ox.routing.shortest_path(self.G, node_a, node_b, weight='length')

            if route is None:
                print("\n✗ Nie znaleziono trasy między punktami!")
                return None

            # Oblicz długość trasy
            route_length = sum(
                ox.routing.route_to_gdf(self.G, route)['length']
            )

            print(f"\n✓ Znaleziono trasę:")
            print(f"  - Liczba węzłów: {len(route)}")
            print(f"  - Długość trasy: {route_length:.2f} metrów")

            self.route = route
            self.route_nodes = (node_a, node_b)

            return route

        except Exception as e:
            print(f"\n✗ Błąd podczas wyznaczania trasy: {e}")
            return None

    def visualize_route_and_bbox(self):
        """Wizualizuj trasę i obszar analizy - 2D + 3D."""
        if len(self.points) != 2:
            return

        print("\n" + "=" * 70)
        print("WIZUALIZACJA TRASY I OBSZARU ANALIZY")
        print("=" * 70)

        point_a, point_b = self.points

        # Oblicz bounding box z marginesem
        margines = 100  # metry

        # Odległość między punktami
        distance_ab = geodesic(point_a, point_b).meters
        print(f"\n📏 Odległość w linii prostej A-B: {distance_ab:.2f} m")

        # Rozszerz bbox o margines
        lats = [point_a[0], point_b[0]]
        lons = [point_a[1], point_b[1]]

        margines_deg = margines / 111000  # przybliżone przeliczenie

        bbox = {
            'north': max(lats) + margines_deg,
            'south': min(lats) - margines_deg,
            'east': max(lons) + margines_deg,
            'west': min(lons) - margines_deg
        }

        print(f"\n📦 Obszar analizy:")
        print(f"   North: {bbox['north']:.6f}")
        print(f"   South: {bbox['south']:.6f}")
        print(f"   East: {bbox['east']:.6f}")
        print(f"   West: {bbox['west']:.6f}")

        # ====================================================================
        # FIGURA Z DWOMA PANELAMI: 2D + 3D
        # ====================================================================
        from mpl_toolkits.mplot3d import Axes3D

        fig = plt.figure(figsize=(18, 8))

        # ====================================================================
        # LEWY PANEL: Widok 2D (jak dotychczas)
        # ====================================================================
        ax_2d = fig.add_subplot(121)

        print("\nTworzenie widoku 2D...")

        # Narysuj sieć drogową
        ox.plot_graph(
            self.G,
            ax=ax_2d,
            node_size=0,
            edge_linewidth=0.5,
            edge_color=COLORS['road'],
            bgcolor='white',
            show=False,
            close=False
        )

        # Dodaj obiekty do mapy zbliżenia
        print("  - Dodawanie obiektów do widoku 2D...")

        # Filtruj obiekty w bbox
        buildings_in_bbox = None
        trees_in_bbox = None
        bridges_in_bbox = None

        if self.buildings is not None:
            try:
                buildings_in_bbox = self.buildings.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]
                if len(buildings_in_bbox) > 0:
                    buildings_in_bbox.plot(
                        ax=ax_2d,
                        facecolor='orange',
                        alpha=0.6,
                        edgecolor='darkgray',
                        linewidth=0.3
                    )
                    print(f"    ✓ Budynki w obszarze: {len(buildings_in_bbox)}")
            except Exception as e:
                print(f"    ⚠ Błąd budynków: {e}")

        if self.trees is not None:
            try:
                trees_in_bbox = self.trees.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]
                if len(trees_in_bbox) > 0:
                    trees_in_bbox.plot(
                        ax=ax_2d,
                        color=COLORS['tree'],
                        markersize=3,
                        alpha=0.4
                    )
                    print(f"    ✓ Drzewa w obszarze: {len(trees_in_bbox)}")
            except Exception as e:
                print(f"    ⚠ Błąd drzew: {e}")

        if self.bridges is not None:
            try:
                bridges_in_bbox = self.bridges.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]
                if len(bridges_in_bbox) > 0:
                    bridges_in_bbox.plot(
                        ax=ax_2d,
                        color=COLORS['bridge'],
                        linewidth=2,
                        alpha=0.5
                    )
                    print(f"    ✓ Mosty w obszarze: {len(bridges_in_bbox)}")
            except Exception as e:
                print(f"    ⚠ Błąd mostów: {e}")

        # Narysuj LINIĘ PROSTĄ między A i B
        ax_2d.plot(
            [point_a[1], point_b[1]],
            [point_a[0], point_b[0]],
            'b-',
            linewidth=3,
            label=f'Linia prosta ({distance_ab:.0f}m)',
            zorder=500
        )

        # Dodaj punkty A i B
        ax_2d.plot(
            point_a[1], point_a[0], 'o',
            color=COLORS['point_a'],
            markersize=15,
            markeredgecolor='black',
            markeredgewidth=2,
            label='Punkt A',
            zorder=1000
        )

        ax_2d.plot(
            point_b[1], point_b[0], 'o',
            color=COLORS['point_b'],
            markersize=15,
            markeredgecolor='black',
            markeredgewidth=2,
            label='Punkt B',
            zorder=1000
        )

        # Dodaj etykiety
        ax_2d.text(point_a[1], point_a[0], '  A', fontsize=14, fontweight='bold', zorder=1001)
        ax_2d.text(point_b[1], point_b[0], '  B', fontsize=14, fontweight='bold', zorder=1001)

        # Narysuj prostokąt obszaru analizy
        width = bbox['east'] - bbox['west']
        height = bbox['north'] - bbox['south']
        rect = patches.Rectangle(
            (bbox['west'], bbox['south']),
            width,
            height,
            linewidth=3,
            edgecolor=COLORS['bbox'],
            facecolor='yellow',
            alpha=0.15,
            label=f'Obszar ±{margines}m',
            zorder=100
        )
        ax_2d.add_patch(rect)

        # Ustaw widok na obszar
        ax_2d.set_xlim(bbox['west'], bbox['east'])
        ax_2d.set_ylim(bbox['south'], bbox['north'])

        # Legenda i tytuł
        ax_2d.legend(loc='upper right', fontsize=9)
        ax_2d.set_title(
            f"Widok 2D | Linia prosta: {distance_ab:.0f}m",
            fontsize=12,
            fontweight='bold'
        )
        ax_2d.set_xlabel('Longitude')
        ax_2d.set_ylabel('Latitude')

        # ====================================================================
        # PRAWY PANEL: Widok 3D z wysokościami budynków
        # ====================================================================
        ax_3d = fig.add_subplot(122, projection='3d')

        print("\nTworzenie widoku 3D z wysokościami...")

        if buildings_in_bbox is not None and len(buildings_in_bbox) > 0:
            # Przygotuj dane o wysokościach budynków
            heights_3d = []

            for idx, building in buildings_in_bbox.iterrows():
                height = None

                # Próbuj pobrać wysokość
                if 'height' in building.index and not pd.isna(building['height']):
                    try:
                        h = str(building['height'])
                        h = h.replace('m', '').replace('M', '').strip()
                        height = float(h)
                    except:
                        pass

                # Szacuj z liczby pięter
                if height is None and 'building:levels' in building.index:
                    try:
                        levels = int(building['building:levels'])
                        height = levels * 3.0
                    except:
                        pass

                # Domyślna wysokość
                if height is None:
                    height = 10.0

                heights_3d.append(height)

            # Rysuj budynki jako słupki 3D
            print("  - Renderowanie budynków 3D...")

            for idx, (building_idx, building) in enumerate(buildings_in_bbox.iterrows()):
                geom = building.geometry
                height = heights_3d[idx]

                try:
                    # Pobierz współrzędne konturu budynku
                    if geom.geom_type == 'Polygon':
                        coords = list(geom.exterior.coords)
                    elif geom.geom_type == 'MultiPolygon':
                        # Weź pierwszy polygon
                        coords = list(geom.geoms[0].exterior.coords)
                    else:
                        continue

                    # Rysuj budynek jako "ściany"
                    xs = [c[0] for c in coords]
                    ys = [c[1] for c in coords]

                    # Podstawa (z=0)
                    ax_3d.plot(xs, ys, 0, 'k-', alpha=0.2, linewidth=0.5)

                    # Góra (z=height)
                    ax_3d.plot(xs, ys, height, 'orange', alpha=0.8, linewidth=1)

                    # Ściany pionowe (próbkuj co 4 punkt dla wydajności)
                    for i in range(0, len(coords)-1, 4):
                        x_wall = [coords[i][0], coords[i][0]]
                        y_wall = [coords[i][1], coords[i][1]]
                        z_wall = [0, height]
                        ax_3d.plot(x_wall, y_wall, z_wall, 'orange', alpha=0.3, linewidth=0.5)

                except Exception as e:
                    continue

            print(f"    ✓ Wyrenderowano {len(buildings_in_bbox)} budynków w 3D")
            print(f"    ℹ Zakres wysokości: {min(heights_3d):.1f}m - {max(heights_3d):.1f}m")

        # Narysuj linię prostą A-B w 3D (na wysokości 30m)
        flight_height = 30.0
        ax_3d.plot(
            [point_a[1], point_b[1]],
            [point_a[0], point_b[0]],
            [flight_height, flight_height],
            'b-',
            linewidth=3,
            label=f'Trasa drona (h={flight_height}m)',
            zorder=1000
        )

        # Punkty A i B w 3D
        ax_3d.scatter(
            [point_a[1]], [point_a[0]], [0],
            color=COLORS['point_a'],
            s=200,
            edgecolors='black',
            linewidths=2,
            label='Start A',
            zorder=1001
        )

        ax_3d.scatter(
            [point_b[1]], [point_b[0]], [0],
            color=COLORS['point_b'],
            s=200,
            edgecolors='black',
            linewidths=2,
            label='Cel B',
            zorder=1001
        )

        # Linie pionowe od punktów do wysokości lotu
        ax_3d.plot(
            [point_a[1], point_a[1]],
            [point_a[0], point_a[0]],
            [0, flight_height],
            'g--',
            linewidth=2,
            alpha=0.6,
            zorder=999
        )

        ax_3d.plot(
            [point_b[1], point_b[1]],
            [point_b[0], point_b[0]],
            [0, flight_height],
            'r--',
            linewidth=2,
            alpha=0.6,
            zorder=999
        )

        # Ustawienia osi 3D
        ax_3d.set_xlim(bbox['west'], bbox['east'])
        ax_3d.set_ylim(bbox['south'], bbox['north'])
        ax_3d.set_zlim(0, max(heights_3d) * 1.2 if buildings_in_bbox is not None and len(buildings_in_bbox) > 0 else 50)

        ax_3d.set_xlabel('Longitude', fontsize=10)
        ax_3d.set_ylabel('Latitude', fontsize=10)
        ax_3d.set_zlabel('Wysokość [m]', fontsize=10)

        ax_3d.set_title(
            f"Widok 3D | Wysokość lotu: {flight_height}m",
            fontsize=12,
            fontweight='bold'
        )

        # Legenda
        ax_3d.legend(loc='upper right', fontsize=9)

        # Ustaw kąt widzenia
        ax_3d.view_init(elev=25, azim=45)

        print("\n✓ Wizualizacja gotowa!")

        plt.tight_layout()
        plt.show()

        # Zwróć dane do dalszej analizy
        return {
            'point_a': point_a,
            'point_b': point_b,
            'distance': distance_ab,
            'bbox': bbox,
            'margin': margines,
            'buildings_in_area': len(buildings_in_bbox) if buildings_in_bbox is not None else 0,
            'max_building_height': max(heights_3d) if buildings_in_bbox is not None and len(buildings_in_bbox) > 0 else 0
        }

    def plan_drone_path(self) -> Optional[List]:
        """
        Zaplanuj ścieżkę drona z omijaniem przeszkód.
        Pyta użytkownika o parametry wysokości.
        """
        if len(self.points) != 2:
            return None

        if DronePathPlanner is None:
            print("\n⚠ Moduł planowania ścieżki niedostępny")
            return None

        print("\n" + "=" * 70)
        print("PARAMETRY LOTU DRONA")
        print("=" * 70)

        # Pobierz parametry od użytkownika
        try:
            h_start = float(input("Wysokość startowa [m] (0): ") or "0")
            h_cruise = float(input("Wysokość przelotowa [m] (30): ") or "30")
            h_end = float(input("Wysokość końcowa [m] (0): ") or "0")
            avoid_distance = float(input("Odległość avoid_distance [m] (2): ") or "2")
        except ValueError:
            print("✗ Błędne wartości, używam domyślnych")
            h_start, h_cruise, h_end, avoid_distance = 0, 30, 0, 2

        print(f"\n✓ Parametry:")
        print(f"  Start: {h_start:.1f}m")
        print(f"  Przelot: {h_cruise:.1f}m")
        print(f"  Koniec: {h_end:.1f}m")

        # Filtruj budynki do obszaru bbox (dla wydajności)
        point_a, point_b = self.points

        # Rozszerz bbox
        margines_deg = 150 / 111000  # 150m
        lats = [point_a[0], point_b[0]]
        lons = [point_a[1], point_b[1]]

        bbox_west = min(lons) - margines_deg
        bbox_east = max(lons) + margines_deg
        bbox_south = min(lats) - margines_deg
        bbox_north = max(lats) + margines_deg

        buildings_filtered = None
        if self.buildings is not None:
            try:
                buildings_filtered = self.buildings.cx[bbox_west:bbox_east, bbox_south:bbox_north]
                print(f"\n📦 Budynki w rozszerzonym obszarze: {len(buildings_filtered)}")
            except:
                buildings_filtered = self.buildings

        # Stwórz konfigurację planera
        config = PathPlannerConfig(
            h_start=h_start,
            h_cruise=h_cruise,
            h_end=h_end,
            avoid_distance=avoid_distance,
            step_size=10.0,
            vertical_penalty=1.5
        )

        # Stwórz planer z konfiguracją
        self.path_planner = DronePathPlanner(
            buildings_gdf=buildings_filtered,
            config=config
        )

        # Zaplanuj ścieżkę (bez parametrów wysokości - są już w config)
        # plan_path zwraca krotkę: (path, total_distance)
        self.drone_path, self.drone_path_distance = self.path_planner.plan_path(
            point_a=point_a,
            point_b=point_b
        )

        return self.drone_path, self.drone_path_distance

    def visualize_drone_path(self):
        """Wizualizuj zaplanowaną ścieżkę drona - 2D + 3D + profil wysokości."""
        if self.drone_path is None or len(self.drone_path) == 0:
            print("⚠ Brak zaplanowanej ścieżki do wizualizacji")
            return

        print("\n" + "=" * 70)
        print("WIZUALIZACJA ZAPLANOWANEJ TRASY DRONA")
        print("=" * 70)

        # Wyciągnij współrzędne z waypoints
        lats = [wp.lat for wp in self.drone_path]
        lons = [wp.lon for wp in self.drone_path]
        alts = [wp.alt for wp in self.drone_path]

        point_a, point_b = self.points

        # Oblicz bounding box z marginesem
        margines = 100  # metry
        margines_deg = margines / 111000

        # Rozszerz bbox o wszystkie punkty trasy
        all_lats = lats + [point_a[0], point_b[0]]
        all_lons = lons + [point_a[1], point_b[1]]

        bbox = {
            'north': max(all_lats) + margines_deg,
            'south': min(all_lats) - margines_deg,
            'east': max(all_lons) + margines_deg,
            'west': min(all_lons) - margines_deg
        }

        # Filtruj budynki w bbox
        buildings_in_bbox = None
        if self.buildings is not None:
            try:
                buildings_in_bbox = self.buildings.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]
                print(f"📦 Budynki w obszarze wizualizacji: {len(buildings_in_bbox)}")
            except Exception as e:
                print(f"⚠ Błąd filtrowania budynków: {e}")

        # ====================================================================
        # FIGURA Z TRZEMA PANELAMI: 2D (z góry) + 3D + PROFIL Z(X)
        # ====================================================================
        from matplotlib.gridspec import GridSpec
        fig = plt.figure(figsize=(18, 12))
        gs = GridSpec(2, 2, figure=fig, height_ratios=[1, 0.6])

        # ====================================================================
        # LEWY GÓRNY PANEL: Widok 2D z góry
        # ====================================================================
        print("\nTworzenie widoku 2D z góry...")
        ax_2d = fig.add_subplot(gs[0, 0])

        # Narysuj sieć drogową jako tło
        try:
            ox.plot_graph(
                self.G,
                ax=ax_2d,
                node_size=0,
                edge_linewidth=0.3,
                edge_color='#CCCCCC',
                bgcolor='white',
                show=False,
                close=False
            )
        except:
            pass

        # Narysuj budynki
        if buildings_in_bbox is not None and len(buildings_in_bbox) > 0:
            buildings_in_bbox.plot(
                ax=ax_2d,
                facecolor='orange',
                alpha=0.5,
                edgecolor='darkgray',
                linewidth=0.5
            )

        # Narysuj linię prostą (dla porównania)
        ax_2d.plot(
            [point_a[1], point_b[1]],
            [point_a[0], point_b[0]],
            'b--',
            linewidth=2,
            alpha=0.3,
            label='Linia prosta',
            zorder=400
        )

        # Narysuj trasę drona
        ax_2d.plot(
            lons, lats,
            'r-',
            linewidth=3,
            label='Trasa drona',
            zorder=500
        )

        # Zaznacz waypoints
        ax_2d.scatter(
            lons, lats,
            c=alts,
            cmap='viridis',
            s=50,
            edgecolor='black',
            linewidth=1,
            zorder=600,
            alpha=0.8
        )

        # Dodaj punkty A i B
        ax_2d.plot(
            point_a[1], point_a[0], 'o',
            color=COLORS['point_a'],
            markersize=15,
            markeredgecolor='black',
            markeredgewidth=2,
            label='Start (A)',
            zorder=1000
        )

        ax_2d.plot(
            point_b[1], point_b[0], 'o',
            color=COLORS['point_b'],
            markersize=15,
            markeredgecolor='black',
            markeredgewidth=2,
            label='Cel (B)',
            zorder=1000
        )

        # Etykiety
        ax_2d.text(point_a[1], point_a[0], '  A', fontsize=14, fontweight='bold', zorder=1001)
        ax_2d.text(point_b[1], point_b[0], '  B', fontsize=14, fontweight='bold', zorder=1001)

        ax_2d.set_title(f'Widok z góry - Trasa drona\n{len(self.drone_path)} waypoints',
                       fontsize=14, fontweight='bold')
        ax_2d.legend(loc='upper right')
        ax_2d.set_xlabel('Długość geograficzna [°]')
        ax_2d.set_ylabel('Szerokość geograficzna [°]')
        ax_2d.grid(True, alpha=0.3)

        # ====================================================================
        # PRAWY GÓRNY PANEL: Widok 3D
        # ====================================================================
        print("Tworzenie widoku 3D...")
        ax_3d = fig.add_subplot(gs[0, 1], projection='3d')

        # Skalowanie współrzędnych do metrów (przybliżone)
        # Punkt odniesienia - środek bbox
        ref_lat = (bbox['north'] + bbox['south']) / 2
        ref_lon = (bbox['east'] + bbox['west']) / 2

        def latlon_to_meters(lat, lon):
            """Konwertuj lat/lon na metry względem punktu odniesienia."""
            y = (lat - ref_lat) * 111000  # 1° ≈ 111km
            x = (lon - ref_lon) * 111000 * np.cos(np.radians(ref_lat))
            return x, y

        # Konwertuj trasę drona na metry
        path_x, path_y = [], []
        for lat, lon in zip(lats, lons):
            x, y = latlon_to_meters(lat, lon)
            path_x.append(x)
            path_y.append(y)

        # Narysuj budynki jako słupki 3D
        if buildings_in_bbox is not None and len(buildings_in_bbox) > 0:
            print(f"  - Renderowanie {len(buildings_in_bbox)} budynków w 3D...")
            for idx, building in buildings_in_bbox.iterrows():
                try:
                    geom = building.geometry
                    if geom.geom_type == 'Polygon':
                        coords = list(geom.exterior.coords)
                    elif geom.geom_type == 'MultiPolygon':
                        coords = list(geom.geoms[0].exterior.coords)
                    else:
                        continue

                    # Wysokość budynku
                    height = building.get('height_m', 10.0)
                    if pd.isna(height):
                        height = 10.0

                    # Konwertuj współrzędne
                    bldg_lons, bldg_lats = zip(*coords)
                    bldg_x, bldg_y = [], []
                    for lat, lon in zip(bldg_lats, bldg_lons):
                        x, y = latlon_to_meters(lat, lon)
                        bldg_x.append(x)
                        bldg_y.append(y)

                    # Rysuj podstawę i górę
                    ax_3d.plot(bldg_x, bldg_y, 0,
                              color='lightgray', alpha=0.3, linewidth=0.5)
                    ax_3d.plot(bldg_x, bldg_y, height,
                              color='orange', alpha=0.5, linewidth=0.5)

                    # Rysuj pionowe krawędzie
                    for x, y in zip(bldg_x[::3], bldg_y[::3]):  # co 3. punkt dla wydajności
                        ax_3d.plot([x, x], [y, y], [0, height],
                                  color='gray', alpha=0.2, linewidth=0.5)

                except Exception as e:
                    continue

        # Narysuj trasę drona w 3D
        ax_3d.plot(path_x, path_y, alts,
                  'r-', linewidth=3, label='Trasa drona', zorder=1000)

        # Zaznacz waypoints z gradientem wysokości
        scatter = ax_3d.scatter(path_x, path_y, alts,
                               c=alts,
                               cmap='viridis',
                               s=50,
                               edgecolor='black',
                               linewidth=1,
                               zorder=1001,
                               alpha=0.8)

        # Punkty A i B
        xa, ya = latlon_to_meters(point_a[0], point_a[1])
        xb, yb = latlon_to_meters(point_b[0], point_b[1])

        ax_3d.scatter([xa], [ya], [alts[0]],
                     color=COLORS['point_a'],
                     s=200,
                     edgecolor='black',
                     linewidth=2,
                     label='Start (A)',
                     zorder=2000)

        ax_3d.scatter([xb], [yb], [alts[-1]],
                     color=COLORS['point_b'],
                     s=200,
                     edgecolor='black',
                     linewidth=2,
                     label='Cel (B)',
                     zorder=2000)

        # Colorbar dla wysokości
        cbar = plt.colorbar(scatter, ax=ax_3d, pad=0.1, shrink=0.8)
        cbar.set_label('Wysokość [m]', rotation=270, labelpad=20)

        # Ustawienia osi
        ax_3d.set_xlabel('X [m]', fontsize=10)
        ax_3d.set_ylabel('Y [m]', fontsize=10)
        ax_3d.set_zlabel('Wysokość [m]', fontsize=10)
        ax_3d.set_title(f'Widok 3D - Trasa drona\nWyskokość: {min(alts):.1f}m - {max(alts):.1f}m',
                       fontsize=14, fontweight='bold')
        ax_3d.legend(loc='upper right')

        # Ustaw widok
        ax_3d.view_init(elev=25, azim=45)

        # ====================================================================
        # DOLNY PANEL (SZEROKI): PROFIL WYSOKOŚCI Z(X)
        # ====================================================================
        print("Tworzenie profilu wysokości z(x)...")
        ax_profile = fig.add_subplot(gs[1, :])  # Rozciągnięty na 2 kolumny

        # Oblicz dystans kumulacyjny wzdłuż trasy
        distances = [0.0]
        for i in range(1, len(path_x)):
            dx = path_x[i] - path_x[i-1]
            dy = path_y[i] - path_y[i-1]
            dist = np.sqrt(dx**2 + dy**2)
            distances.append(distances[-1] + dist)

        # Rysuj profil trasy drona
        ax_profile.plot(distances, alts, 'r-', linewidth=3, label='Trasa drona', zorder=100)
        ax_profile.scatter(distances, alts, c=alts, cmap='viridis',
                          s=30, edgecolor='black', linewidth=0.5, zorder=101)

        # Oznacz start i koniec
        ax_profile.scatter([distances[0]], [alts[0]], color=COLORS['point_a'],
                          s=150, marker='o', edgecolor='black', linewidth=2,
                          label='Start (A)', zorder=200)
        ax_profile.scatter([distances[-1]], [alts[-1]], color=COLORS['point_b'],
                          s=150, marker='o', edgecolor='black', linewidth=2,
                          label='Cel (B)', zorder=200)

        # Rysuj budynki NAD KTÓRYMI LECI DRON
        from shapely.geometry import Point, LineString

        if buildings_in_bbox is not None and len(buildings_in_bbox) > 0:
            print(f"  - Sprawdzanie budynków nad którymi leci dron...")

            # Stwórz linię trasy w 2D (lat, lon)
            route_line = LineString([(lon, lat) for lat, lon in zip(lats, lons)])

            # Bufor wokół trasy (metry -> stopnie)
            buffer_m = 10.0  # sprawdź budynki w promieniu 10m od trasy
            buffer_deg = buffer_m / 111000
            route_buffer = route_line.buffer(buffer_deg)

            buildings_on_route = []
            for idx, building in buildings_in_bbox.iterrows():
                if building.geometry.intersects(route_buffer):
                    buildings_on_route.append(building)

            print(f"  - Znaleziono {len(buildings_on_route)} budynków nad trasą")

            # Dla każdego budynku, znajdź w jakim miejscu trasy się znajduje
            for building in buildings_on_route:
                try:
                    # Znajdź punkty trasy które są nad budynkiem
                    height = building.get('height_m', 10.0)
                    if pd.isna(height):
                        height = 10.0

                    # Sprawdź które punkty trasy są nad budynkiem
                    x_ranges = []
                    for i in range(len(lats)):
                        point = Point(lons[i], lats[i])
                        if building.geometry.contains(point) or building.geometry.distance(point) < buffer_deg:
                            x_ranges.append(distances[i])

                    if len(x_ranges) > 0:
                        # Rysuj budynek jako wypełniony prostokąt
                        x_min, x_max = min(x_ranges), max(x_ranges)
                        # Rozszerz trochę dla lepszej widoczności
                        width = max(x_max - x_min, 5.0)  # min 5m szerokości
                        x_center = (x_min + x_max) / 2

                        ax_profile.add_patch(
                            patches.Rectangle(
                                (x_center - width/2, 0),
                                width, height,
                                facecolor='orange',
                                edgecolor='darkgray',
                                alpha=0.6,
                                linewidth=1,
                                zorder=50
                            )
                        )
                except Exception as e:
                    continue

        # Linia poziomu gruntu
        ax_profile.axhline(y=0, color='brown', linestyle='-', linewidth=2,
                          alpha=0.5, label='Poziom gruntu', zorder=10)

        # Wypełnienie pod trasą (do ziemi)
        ax_profile.fill_between(distances, 0, alts, alpha=0.1, color='skyblue', zorder=20)

        # Ustawienia osi
        ax_profile.set_xlabel('Dystans wzdłuż trasy [m]', fontsize=12)
        ax_profile.set_ylabel('Wysokość [m]', fontsize=12)
        ax_profile.set_title('Profil wysokości lotu - Z(x)', fontsize=14, fontweight='bold')
        ax_profile.grid(True, alpha=0.3, zorder=0)
        ax_profile.legend(loc='upper right')
        ax_profile.set_ylim(bottom=-2, top=max(alts) * 1.1)

        # Dodaj adnotacje
        max_alt = max(alts)
        min_alt = min(alts)
        max_idx = alts.index(max_alt)

        ax_profile.annotate(f'Max: {max_alt:.1f}m',
                           xy=(distances[max_idx], max_alt),
                           xytext=(10, 10), textcoords='offset points',
                           fontsize=10, fontweight='bold',
                           bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7),
                           arrowprops=dict(arrowstyle='->', color='black', lw=1.5))

        # Statystyki trasy
        # Oblicz długość trasy 3D (dla wizualizacji)
        total_distance_3d = sum([
            np.sqrt((path_x[i+1]-path_x[i])**2 +
                   (path_y[i+1]-path_y[i])**2 +
                   (alts[i+1]-alts[i])**2)
            for i in range(len(path_x)-1)
        ])

        print(f"\n📊 Statystyki trasy:")
        print(f"  - Liczba waypoints: {len(self.drone_path)}")

        # Użyj wartości z planera jeśli dostępna
        if self.drone_path_distance is not None:
            print(f"  - Długość trasy: {self.drone_path_distance:.2f} m")
        else:
            print(f"  - Długość trasy (3D): {total_distance_3d:.2f} m")

        print(f"  - Wysokość min/max: {min(alts):.1f}m / {max(alts):.1f}m")
        print(f"  - Średnia wysokość: {np.mean(alts):.1f}m")

        plt.tight_layout()
        print("\n✓ Wizualizacja gotowa! Zamknij okno aby kontynuować...")
        plt.show()

        print("=" * 70)


    def animate_fpv_flight(self):
        """
        Animacja 2D - widok z góry, kamera podąża za dronem.
        Pokazuje lokalny obszar wokół drona z trasą i budynkami.
        """
        if self.drone_path is None or len(self.drone_path) == 0:
            print("⚠ Brak zaplanowanej ścieżki do animacji")
            return

        print("\n" + "=" * 70)
        print("ANIMACJA 2D - WIDOK Z GÓRY")
        print("=" * 70)
        print("\nPrzygotowanie animacji...")

        # Wyciągnij współrzędne
        lats = [wp.lat for wp in self.drone_path]
        lons = [wp.lon for wp in self.drone_path]
        alts = [wp.alt for wp in self.drone_path]

        # Punkt odniesienia - środek trasy
        ref_lat = np.mean(lats)
        ref_lon = np.mean(lons)

        def latlon_to_meters(lat, lon):
            """Konwertuj lat/lon na metry względem punktu odniesienia."""
            y = (lat - ref_lat) * 111000
            x = (lon - ref_lon) * 111000 * np.cos(np.radians(ref_lat))
            return x, y

        # Konwertuj trasę na metry
        path_x, path_y = [], []
        for lat, lon in zip(lats, lons):
            x, y = latlon_to_meters(lat, lon)
            path_x.append(x)
            path_y.append(y)

        path_x = np.array(path_x)
        path_y = np.array(path_y)

        # Filtruj budynki w obszarze trasy
        margines = 200  # metry
        margines_deg = margines / 111000

        bbox = {
            'north': max(lats) + margines_deg,
            'south': min(lats) - margines_deg,
            'east': max(lons) + margines_deg,
            'west': min(lons) - margines_deg
        }

        buildings_in_bbox = None
        building_shapes = []

        if self.buildings is not None:
            try:
                buildings_in_bbox = self.buildings.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]
                print(f"  - Budynki w obszarze: {len(buildings_in_bbox)}")

                # Przygotuj geometrie budynków
                for idx, building in buildings_in_bbox.iterrows():
                    try:
                        geom = building.geometry
                        if geom.geom_type == 'Polygon':
                            coords = list(geom.exterior.coords)
                        elif geom.geom_type == 'MultiPolygon':
                            coords = list(geom.geoms[0].exterior.coords)
                        else:
                            continue

                        # Konwertuj współrzędne budynku
                        bldg_lons, bldg_lats = zip(*coords)
                        bldg_x, bldg_y = [], []
                        for lat, lon in zip(bldg_lats, bldg_lons):
                            x, y = latlon_to_meters(lat, lon)
                            bldg_x.append(x)
                            bldg_y.append(y)

                        building_shapes.append({
                            'x': bldg_x,
                            'y': bldg_y
                        })

                    except Exception:
                        continue

                print(f"  - Przygotowano {len(building_shapes)} budynków")

            except Exception as e:
                print(f"  ⚠ Błąd filtrowania budynków: {e}")

        # Ustawienia animacji
        fps = 30
        speed_multiplier = 5.0  # Szybko!

        # Oblicz czas trwania
        segment_times = []
        for i in range(len(path_x) - 1):
            dist = np.sqrt((path_x[i+1]-path_x[i])**2 + (path_y[i+1]-path_y[i])**2)
            time = dist / 5.0 / speed_multiplier  # 5 m/s prędkość drona
            segment_times.append(time)

        total_time = sum(segment_times)
        total_frames = int(total_time * fps)

        print(f"  - Całkowity czas animacji: {total_time:.1f}s")
        print(f"  - Liczba klatek: {total_frames}")
        print(f"  - Prędkość: {speed_multiplier:.1f}x")

        # Interpoluj pozycje drona
        t = np.linspace(0, 1, total_frames)
        drone_x = np.interp(t, np.linspace(0, 1, len(path_x)), path_x)
        drone_y = np.interp(t, np.linspace(0, 1, len(path_y)), path_y)

        # Stwórz figurę
        fig, ax = plt.subplots(figsize=(12, 10))
        ax.set_aspect('equal')
        ax.set_facecolor('#f0f0f0')

        # Elementy animacji
        drone_point, = ax.plot([], [], 'ro', markersize=12,
                               markeredgecolor='black', markeredgewidth=2,
                               label='Dron', zorder=1000)

        path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.5,
                             label='Ścieżka', zorder=100)

        path_traveled, = ax.plot([], [], 'g-', linewidth=3, alpha=0.7,
                                 label='Przebyta trasa', zorder=200)

        # Tytuł i informacje
        title_text = ax.text(0.5, 0.98, '', transform=ax.transAxes,
                            ha='center', va='top', fontsize=14, fontweight='bold')

        info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                           ha='left', va='top', fontsize=11, family='monospace',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_xlabel('X [m]', fontsize=11)
        ax.set_ylabel('Y [m]', fontsize=11)

        def init():
            """Inicjalizacja animacji."""
            drone_point.set_data([], [])
            path_line.set_data([], [])
            path_traveled.set_data([], [])
            return drone_point, path_line, path_traveled, title_text, info_text

        def update(frame):
            """Aktualizacja klatki animacji."""
            # Pozycja drona
            dx = drone_x[frame]
            dy = drone_y[frame]

            # Aktualizuj pozycję drona
            drone_point.set_data([dx], [dy])

            # Pokaż fragment trasy przed dronem
            look_ahead = 80  # metrów
            look_ahead_frames = min(int(look_ahead / 5), total_frames - frame - 1)

            if look_ahead_frames > 0:
                ahead_x = drone_x[frame:frame+look_ahead_frames]
                ahead_y = drone_y[frame:frame+look_ahead_frames]
                path_line.set_data(ahead_x, ahead_y)
            else:
                path_line.set_data([], [])

            # Pokaż przebyty szlak
            if frame > 0:
                traveled_x = drone_x[0:frame]
                traveled_y = drone_y[0:frame]
                path_traveled.set_data(traveled_x, traveled_y)

            # Rysuj budynki w lokalnym obszarze (czyszczenie i rysowanie)
            # Usuń poprzednie patches
            for patch in ax.patches[:]:
                patch.remove()

            # Lokalny obszar wokół drona
            view_range = 100  # metrów

            for bldg in building_shapes:
                bldg_x = np.array(bldg['x'])
                bldg_y = np.array(bldg['y'])

                # Sprawdź czy budynek jest w zasięgu
                bldg_center_x = np.mean(bldg_x)
                bldg_center_y = np.mean(bldg_y)
                dist = np.sqrt((bldg_center_x - dx)**2 + (bldg_center_y - dy)**2)

                if dist < view_range * 1.5:
                    # Rysuj budynek jako wypełniony polygon
                    poly = plt.Polygon(list(zip(bldg_x, bldg_y)),
                                     facecolor='#808080',
                                     edgecolor='#404040',
                                     alpha=0.7, linewidth=1, zorder=50)
                    ax.add_patch(poly)

            # Ustaw zakres widoku - kamera podąża za dronem
            ax.set_xlim(dx - view_range, dx + view_range)
            ax.set_ylim(dy - view_range, dy + view_range)

            # Aktualizuj tekst
            progress = (frame / total_frames) * 100
            current_waypoint = int((frame / total_frames) * len(self.drone_path))
            current_alt = alts[min(current_waypoint, len(alts)-1)]

            title_text.set_text(f'🚁 ANIMACJA 2D - WIDOK Z GÓRY')

            info_text.set_text(
                f'Postęp: {progress:.1f}%\n'
                f'Waypoint: {current_waypoint}/{len(self.drone_path)}\n'
                f'Wysokość: {current_alt:.1f}m\n'
                f'Prędkość: {speed_multiplier:.1f}x'
            )

            return drone_point, path_line, path_traveled, title_text, info_text

        # Stwórz animację
        print("\n🎬 Uruchamianie animacji 2D...")
        print("   (Zamknij okno aby zakończyć)")

        anim = animation.FuncAnimation(
            fig, update, init_func=init,
            frames=total_frames, interval=1000/fps,
            blit=False, repeat=True
        )

        plt.tight_layout()
        plt.show()

        print("=" * 70)



    def animate_zx_view(self):
        """
        Animacja w płaszczyźnie ZX (profil wzdłuż trasy):
        - Oś X: dystans wzdłuż ścieżki (metry)
        - Oś Z: wysokość (metry)
        - Budynki przecinające korytarz wokół trasy rysowane jako szare prostokąty (tylko w tej płaszczyźnie)
        """
        import numpy as np
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        import matplotlib.animation as animation
        import pandas as pd
        from shapely.geometry import Point, LineString

        if self.drone_path is None or len(self.drone_path) == 0:
            print("⚠ Brak zaplanowanej ścieżki do animacji ZX")
            return

        # Trasa
        lats = [wp.lat for wp in self.drone_path]
        lons = [wp.lon for wp in self.drone_path]
        alts = [wp.alt for wp in self.drone_path]

        # Lokalna metryka (przybliżona)
        ref_lat = float(np.mean(lats))
        ref_lon = float(np.mean(lons))
        def latlon_to_meters(lat, lon):
            y = (lat - ref_lat) * 111000.0
            x = (lon - ref_lon) * 111000.0 * np.cos(np.radians(ref_lat))
            return x, y

        xs, ys = [], []
        for lat, lon in zip(lats, lons):
            x, y = latlon_to_meters(lat, lon)
            xs.append(x); ys.append(y)
        xs = np.asarray(xs); ys = np.asarray(ys); zs = np.asarray(alts, dtype=float)

        # Dystans skumulowany (po XY)
        dist = np.zeros(len(xs))
        for i in range(1, len(xs)):
            dist[i] = dist[i-1] + np.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1])

        # Budynki w korytarzu trasy -> rzuty prostokątów na oś X (s)
        buildings_rects = []
        if self.buildings is not None and len(self.buildings) > 0:
            try:
                marg_m = 150.0; marg_deg = marg_m / 111000.0
                bbox = {
                    'north': max(lats) + marg_deg,
                    'south': min(lats) - marg_deg,
                    'east':  max(lons) + marg_deg,
                    'west':  min(lons) - marg_deg
                }
                bdf = self.buildings.cx[bbox['west']:bbox['east'], bbox['south']:bbox['north']]
            except Exception:
                bdf = self.buildings

            route_line = LineString([(lon, lat) for lat, lon in zip(lats, lons)])
            track_half_width_m = 10.0
            track_buff_deg = track_half_width_m / 111000.0
            route_buffer = route_line.buffer(track_buff_deg)

            def project_to_s(dist_xs, dist_ys, px, py):
                best_s = None; best_d = 1e18; acc_s = 0.0
                for i in range(len(dist_xs)-1):
                    x1,y1 = dist_xs[i], dist_ys[i]
                    x2,y2 = dist_xs[i+1], dist_ys[i+1]
                    vx,vy = x2-x1, y2-y1
                    wx,wy = px-x1, py-y1
                    seg_len = np.hypot(vx,vy); seg_len2 = seg_len*seg_len
                    t = 0.0 if seg_len2 <= 1e-9 else max(0.0, min(1.0, (wx*vx + wy*vy)/seg_len2))
                    projx = x1 + t*vx; projy = y1 + t*vy
                    d = np.hypot(px - projx, py - projy)
                    if d < best_d:
                        best_d = d; best_s = acc_s + seg_len * t
                    acc_s += seg_len
                return best_s if best_s is not None else 0.0

            for _, b in bdf.iterrows():
                geom = getattr(b, "geometry", None)
                if geom is None or not geom.intersects(route_buffer):
                    continue
                # wysokość
                h = None
                if 'height' in b and pd.notna(b['height']):
                    try:
                        h = float(str(b['height']).replace('m','').replace('M','').strip())
                    except Exception:
                        h = None
                if h is None and 'building:levels' in b:
                    try: h = float(int(b['building:levels']) * 3.0)
                    except Exception: h = None
                if h is None: h = 10.0

                # kontur -> metry
                try:
                    if geom.geom_type == 'Polygon':
                        coords = list(geom.exterior.coords)
                    elif geom.geom_type == 'MultiPolygon':
                        coords = list(geom.geoms[0].exterior.coords)
                    else:
                        continue
                    b_lons, b_lats = zip(*coords)
                    bxs,bys = [], []
                    for la,lo in zip(b_lats, b_lons):
                        x,y = latlon_to_meters(la, lo)
                        bxs.append(x); bys.append(y)
                    s_vals = [project_to_s(xs, ys, x, y) for x,y in zip(bxs,bys)]
                    if not s_vals:
                        continue
                    s_min, s_max = min(s_vals), max(s_vals)
                    width = max(s_max - s_min, 5.0)
                    s0 = 0.5*(s_min + s_max)
                    buildings_rects.append((s0 - width/2.0, width, h))
                except Exception:
                    continue

        # Animacja
        fps = 30; speed_mps = 5.0; speed_mul = 5.0
        total_time = (dist[-1] / speed_mps) / max(1e-6, speed_mul)
        total_frames = max(1, int(total_time * fps))

        t = np.linspace(0.0, 1.0, total_frames)
        s_interp = np.interp(t, np.linspace(0.0, 1.0, len(dist)), dist)
        z_interp = np.interp(t, np.linspace(0.0, 1.0, len(zs)), zs)

        fig, ax = plt.subplots(figsize=(14, 6))
        ax.set_facecolor('#f9f9f9')
        ax.grid(True, alpha=0.25, linestyle='--')

        # budynki - szare prostokąty
        for (x0,w,h) in buildings_rects:
            rect = patches.Rectangle((x0, 0), w, h, facecolor='#888888', edgecolor='#444444',
                                     linewidth=0.6, alpha=0.6, zorder=5)
            ax.add_patch(rect)

        # ścieżka Z(X)
        ax.plot(dist, zs, '-', linewidth=2.5, alpha=0.8, label='Profil lotu (Z–X)', zorder=10)

        # punkt drona
        drone_pt, = ax.plot([], [], 'ro', markersize=10, markeredgecolor='black', markeredgewidth=1.5,
                            label='Dron', zorder=20)
        info = ax.text(0.01, 0.98, '', transform=ax.transAxes, ha='left', va='top', fontsize=11,
                       family='monospace', bbox=dict(boxstyle='round', facecolor='white', alpha=0.85))

        ax.set_xlabel('Dystans wzdłuż trasy X [m]')
        ax.set_ylabel('Wysokość Z [m]')
        ax.set_title('Animacja w płaszczyźnie ZX (profil wzdłuż lotu)', fontsize=13, fontweight='bold')
        ax.legend(loc='upper right')

        ax.set_xlim(0, max(dist) * 1.02)
        ax.set_ylim(bottom=-2, top=max(max(zs)*1.15, 30))

        view_half_width = 80.0
        def update(frame):
            sx = s_interp[frame]; zz = z_interp[frame]
            drone_pt.set_data([sx], [zz])
            x_min = max(0.0, sx - view_half_width)
            x_max = min(dist[-1], sx + view_half_width)
            ax.set_xlim(x_min, x_max)
            progress = (frame / max(1, total_frames-1)) * 100.0
            info.set_text(f'Postęp: {progress:5.1f}%\\nX: {sx:7.1f} m\\nZ: {zz:6.1f} m')
            return drone_pt, info

        anim = animation.FuncAnimation(fig, update, frames=total_frames, interval=1000.0/fps,
                                       blit=False, repeat=True)
        plt.tight_layout()
        plt.show()


    def play_all_views(self):
        """
        Uruchamia po kolei:
        1) visualize_drone_path() – podgląd 2D+3D+profil
        2) animate_fpv_flight()   – animacja FPV (perspektywa drona)
        3) animate_zx_view()      – animacja profilu ZX
        """
        self.visualize_drone_path()
        self.animate_fpv_flight()
        self.animate_zx_view()

    def run(self):
        """
        Animacja FPV (First Person View) - widok z perspektywy drona.
        Pokazuje lot drona przez przeszkody w czasie rzeczywistym.
        """
        if self.drone_path is None or len(self.drone_path) == 0:
            print("⚠ Brak zaplanowanej ścieżki do animacji")
            return

        print("\n" + "=" * 70)
        print("ANIMACJA FPV - WIDOK Z DRONA")
        print("=" * 70)
        print("\nPrzygotowanie animacji...")

        # Wyciągnij współrzędne
        lats = [wp.lat for wp in self.drone_path]
        lons = [wp.lon for wp in self.drone_path]
        alts = [wp.alt for wp in self.drone_path]

        # Punkt odniesienia - środek trasy
        ref_lat = np.mean(lats)
        ref_lon = np.mean(lons)

        def latlon_to_meters(lat, lon):
            """Konwertuj lat/lon na metry względem punktu odniesienia."""
            y = (lat - ref_lat) * 111000
            x = (lon - ref_lon) * 111000 * np.cos(np.radians(ref_lat))
            return x, y

        # Konwertuj trasę na metry
        path_x, path_y, path_z = [], [], []
        for lat, lon, alt in zip(lats, lons, alts):
            x, y = latlon_to_meters(lat, lon)
            path_x.append(x)
            path_y.append(y)
            path_z.append(alt)

        path_x = np.array(path_x)
        path_y = np.array(path_y)
        path_z = np.array(path_z)

        # Filtruj budynki w obszarze trasy
        margines = 150  # metry
        margines_deg = margines / 111000

        bbox = {
            'north': max(lats) + margines_deg,
            'south': min(lats) - margines_deg,
            'east': max(lons) + margines_deg,
            'west': min(lons) - margines_deg
        }

        buildings_in_bbox = None
        building_boxes = []

        if self.buildings is not None:
            try:
                buildings_in_bbox = self.buildings.cx[
                    bbox['west']:bbox['east'],
                    bbox['south']:bbox['north']
                ]
                print(f"  - Budynki w obszarze: {len(buildings_in_bbox)}")

                # Przygotuj geometrie budynków jako pudełka 3D
                for idx, building in buildings_in_bbox.iterrows():
                    try:
                        geom = building.geometry
                        if geom.geom_type == 'Polygon':
                            coords = list(geom.exterior.coords)
                        elif geom.geom_type == 'MultiPolygon':
                            coords = list(geom.geoms[0].exterior.coords)
                        else:
                            continue

                        height = building.get('height_m', 10.0)
                        if pd.isna(height):
                            height = 10.0

                        # Konwertuj współrzędne budynku
                        bldg_lons, bldg_lats = zip(*coords)
                        bldg_x, bldg_y = [], []
                        for lat, lon in zip(bldg_lats, bldg_lons):
                            x, y = latlon_to_meters(lat, lon)
                            bldg_x.append(x)
                            bldg_y.append(y)

                        building_boxes.append({
                            'x': bldg_x,
                            'y': bldg_y,
                            'height': height
                        })

                    except Exception:
                        continue

                print(f"  - Przygotowano {len(building_boxes)} budynków do animacji")

            except Exception as e:
                print(f"  ⚠ Błąd filtrowania budynków: {e}")

        # Ustawienia animacji
        fps = 30
        speed_multiplier = 4.0  # Szybciej! (było 2.0)

        # Oblicz czas trwania każdego segmentu
        segment_times = []
        for i in range(len(path_x) - 1):
            dist = np.sqrt((path_x[i+1]-path_x[i])**2 +
                          (path_y[i+1]-path_y[i])**2 +
                          (path_z[i+1]-path_z[i])**2)
            # Załóżmy prędkość drona 5 m/s
            time = dist / 5.0 / speed_multiplier
            segment_times.append(time)

        total_time = sum(segment_times)
        total_frames = int(total_time * fps)

        print(f"  - Całkowity czas animacji: {total_time:.1f}s")
        print(f"  - Liczba klatek: {total_frames}")
        print(f"  - Prędkość odtwarzania: {speed_multiplier}x")

        # Interpoluj pozycje drona dla płynnej animacji
        t = np.linspace(0, 1, total_frames)

        # Interpolacja pozycji
        drone_x = np.interp(t, np.linspace(0, 1, len(path_x)), path_x)
        drone_y = np.interp(t, np.linspace(0, 1, len(path_y)), path_y)
        drone_z = np.interp(t, np.linspace(0, 1, len(path_z)), path_z)

        # Stwórz figurę
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        # NIE rysujemy budynków tutaj - będą rysowane dynamicznie w update()
        print("  - Budynki będą renderowane dynamicznie (tylko te przed dronem)")

        # Początkowe ustawienia kamery
        look_ahead = 30  # Ile metrów przed dronem patrzeć
        camera_height_offset = 2  # Kamera lekko nad dronem

        # Inicjalizacja elementów animacji
        drone_point, = ax.plot([], [], [], 'ro', markersize=8,
                              markeredgecolor='black', markeredgewidth=1,
                              label='Dron')

        path_ahead_line, = ax.plot([], [], [], 'r-', linewidth=2,
                                   alpha=0.7, label='Tor lotu')

        # Tytuł i informacje
        title_text = ax.text2D(0.5, 0.95, '', transform=ax.transAxes,
                              ha='center', fontsize=12, fontweight='bold')

        info_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes,
                             ha='left', fontsize=10, family='monospace')

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Wysokość [m]')
        ax.legend(loc='upper right')

        def init():
            """Inicjalizacja animacji."""
            drone_point.set_data([], [])
            drone_point.set_3d_properties([])
            path_ahead_line.set_data([], [])
            path_ahead_line.set_3d_properties([])
            return drone_point, path_ahead_line, title_text, info_text

        def update(frame):
            """Aktualizacja klatki animacji."""
            # Wyczyść poprzednie obiekty 3D (budynki)
            # Zachowaj tylko drone_point i path_ahead_line
            while len(ax.collections) > 0:
                ax.collections[0].remove()

            # Pozycja drona
            dx = drone_x[frame]
            dy = drone_y[frame]
            dz = drone_z[frame]

            # Aktualizuj pozycję drona
            drone_point.set_data([dx], [dy])
            drone_point.set_3d_properties([dz])

            # Kierunek lotu (wektor)
            if frame < total_frames - 1:
                dir_x = drone_x[frame+1] - drone_x[frame]
                dir_y = drone_y[frame+1] - drone_y[frame]
                dir_z = drone_z[frame+1] - drone_z[frame]
                norm = np.sqrt(dir_x**2 + dir_y**2 + dir_z**2)
                if norm > 0.001:
                    dir_x, dir_y, dir_z = dir_x/norm, dir_y/norm, dir_z/norm
                else:
                    dir_x, dir_y, dir_z = 0, 1, 0
            else:
                dir_x, dir_y, dir_z = 0, 1, 0

            # Rysuj fragment trasy przed dronem (look ahead)
            look_ahead_frames = min(int(look_ahead / 5), total_frames - frame - 1)
            if look_ahead_frames > 0:
                ahead_x = drone_x[frame:frame+look_ahead_frames]
                ahead_y = drone_y[frame:frame+look_ahead_frames]
                ahead_z = drone_z[frame:frame+look_ahead_frames]

                path_ahead_line.set_data(ahead_x, ahead_y)
                path_ahead_line.set_3d_properties(ahead_z)
            else:
                path_ahead_line.set_data([], [])
                path_ahead_line.set_3d_properties([])

            # DYNAMICZNE RENDEROWANIE BUDYNKÓW - tylko te przed dronem!
            view_distance = 80  # Jak daleko widzisz [m]

            for bldg in building_boxes:
                x_coords = np.array(bldg['x'])
                y_coords = np.array(bldg['y'])
                height = bldg['height']

                # Środek budynku
                bldg_center_x = np.mean(x_coords)
                bldg_center_y = np.mean(y_coords)

                # Wektor od drona do budynku
                to_bldg_x = bldg_center_x - dx
                to_bldg_y = bldg_center_y - dy
                dist_2d = np.sqrt(to_bldg_x**2 + to_bldg_y**2)

                # Sprawdź czy budynek jest w zasięgu
                if dist_2d > view_distance:
                    continue

                # Sprawdź czy budynek jest przed dronem (dot product > 0)
                if dist_2d > 0.001:
                    to_bldg_x_norm = to_bldg_x / dist_2d
                    to_bldg_y_norm = to_bldg_y / dist_2d

                    # Dot product z kierunkiem lotu
                    dot = dir_x * to_bldg_x_norm + dir_y * to_bldg_y_norm

                    # Rysuj tylko jeśli jest przed dronem (dot > -0.3 daje szerszy kąt widzenia)
                    if dot < -0.3:
                        continue

                # RYSUJ BUDYNEK jako NIEPRZEZROCZYSTE ŚCIANY
                # Dla każdej ściany budynku, stwórz wypełniony polygon
                n_verts = len(x_coords) - 1  # Ostatni punkt = pierwszy

                for i in range(n_verts):
                    # Ściana pionowa między dwoma wierzchołkami
                    x1, y1 = x_coords[i], y_coords[i]
                    x2, y2 = x_coords[i+1], y_coords[i+1]

                    # Stwórz wierzchołki ściany (prostokąt pionowy)
                    verts = [
                        [x1, y1, 0],           # Dół lewy
                        [x2, y2, 0],           # Dół prawy
                        [x2, y2, height],      # Góra prawa
                        [x1, y1, height]       # Góra lewa
                    ]

                    # Dodaj ścianę jako nieprzezroczysty polygon
                    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
                    poly = Poly3DCollection([verts], alpha=0.9,
                                          facecolor='#808080',
                                          edgecolor='#404040',
                                          linewidth=0.5)
                    ax.add_collection3d(poly)

                # Dodaj dach (górna powierzchnia)
                roof_verts = [[x_coords[i], y_coords[i], height]
                             for i in range(n_verts)]
                roof_poly = Poly3DCollection([roof_verts], alpha=0.8,
                                           facecolor='#606060',
                                           edgecolor='#404040',
                                           linewidth=0.5)
                ax.add_collection3d(roof_poly)

            # Kamera patrzy W KIERUNKU LOTU (normalnie do wektora ruchu)
            # Kamera jest lekko za i nad dronem
            camera_distance = 25  # Bliżej drona (było 40)
            camera_x = dx - dir_x * camera_distance
            camera_y = dy - dir_y * camera_distance
            camera_z = dz + camera_height_offset

            # Punkt na który patrzy kamera (przed dronem)
            target_x = dx + dir_x * look_ahead
            target_y = dy + dir_y * look_ahead
            target_z = dz + dir_z * 10  # Patrzy w kierunku lotu (z uwzględnieniem Z)

            # Oblicz kąty dla view_init
            azim = np.degrees(np.arctan2(target_y - camera_y, target_x - camera_x))
            elev = np.degrees(np.arctan2(target_z - camera_z,
                             np.sqrt((target_x-camera_x)**2 + (target_y-camera_y)**2)))

            ax.view_init(elev=elev, azim=azim)

            # Zakres widoku (pudełko wokół drona)
            view_range = 50  # Mniejszy zakres dla lepszego efektu FPV
            ax.set_xlim(dx - view_range, dx + view_range)
            ax.set_ylim(dy - view_range, dy + view_range)
            ax.set_zlim(max(0, dz - 20), dz + 40)  # Zakres względem drona

            # Aktualizuj tekst
            progress = (frame / total_frames) * 100
            current_waypoint = int((frame / total_frames) * len(self.drone_path))

            title_text.set_text(f'🚁 WIDOK FPV - LOT DRONA')

            info_text.set_text(
                f'Postęp: {progress:.1f}%\n'
                f'Waypoint: {current_waypoint}/{len(self.drone_path)}\n'
                f'Wysokość: {dz:.1f}m\n'
                f'Prędkość: {speed_multiplier:.1f}x'
            )

            return drone_point, path_ahead_line, title_text, info_text

        # Stwórz animację
        print("\n🎬 Uruchamianie animacji FPV...")
        print("   (Zamknij okno aby zakończyć)")

        anim = animation.FuncAnimation(
            fig, update, init_func=init,
            frames=total_frames, interval=1000/fps,
            blit=False, repeat=True
        )

        plt.tight_layout()
        plt.show()

        print("=" * 70)


    def run(self):
        """Uruchom pełny workflow."""
        # 1. Załaduj dane
        if not self.load_data():
            return None

        # 2. Stwórz mapę
        self.create_base_map()

        # 3. Dodaj warstwy
        self.add_buildings_layer()
        self.add_trees_layer()
        self.add_bridges_layer()

        # 4. Włącz tryb interaktywny
        self.setup_interactive_mode()

        # 5. Pokaż mapę (czeka na wybór punktów)
        plt.tight_layout()
        plt.show()

        # 6. Po zamknięciu okna - wizualizuj i planuj
        if len(self.points) == 2:
            result = self.visualize_route_and_bbox()

            # Zapytaj czy planować ścieżkę
            print("\n" + "=" * 70)
            plan = input("Czy zaplanować ścieżkę z omijaniem przeszkód? (t/n): ").strip().lower()

            if plan in ['t', 'tak', 'yes', 'y']:
                self.plan_drone_path()

                # Wizualizuj zaplanowaną trasę
                if self.drone_path is not None:
                    self.visualize_drone_path()

                    # Zapytaj czy uruchomić animację FPV
                    print("\n" + "=" * 70)
                    animate = input("Czy odpalić animację FPV (widok z drona)? (t/n): ").strip().lower()

                    if animate in ['t', 'tak', 'yes', 'y']:
                        self.animate_fpv_flight()

            print("\n" + "=" * 70)
            print("GOTOWE!")
            print("=" * 70)
            print("\nDane do dalszej analizy:")
            print(f"  punkt_a = {self.points[0]}")
            print(f"  punkt_b = {self.points[1]}")
            print(f"  odleglosc = {result['distance']:.2f} m")
            print(f"  budynki_w_obszarze = {result['buildings_in_area']}")
            print(f"  max_wysokosc_budynku = {result['max_building_height']:.1f} m")
            print("=" * 70)

            return result
        else:
            print("\n⚠ Nie wybrano obu punktów")
            return None


def main():
    """Główna funkcja programu."""
    selector = WarsawMapSelector()
    result = selector.run()

    return result


if __name__ == "__main__":
    main()