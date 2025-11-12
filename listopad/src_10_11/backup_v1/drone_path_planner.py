"""
3D A* Path Planner for Drone Navigation with OSM Buildings
===========================================================
Planuje optymalną 3D ścieżkę od punktu A do B omijając budynki.

Algorytm:
1. Tworzy 3D occupancy grid (X, Y, Z)
2. Oznacza komórki z budynkami jako occupied
3. A* znajduje najkrótszą ścieżkę
4. Path smoothing dla płynnej trajektorii
5. Export do formatu trajectory.py
"""

import numpy as np
import heapq
from scipy.interpolate import splprep, splev
from shapely.geometry import Point
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Node:
    """Węzeł w grafie A*."""

    def __init__(self, position, parent=None):
        self.position = position  # (x, y, z) w gridzie
        self.parent = parent
        self.g = 0  # Koszt od startu
        self.h = 0  # Heurystyka do celu
        self.f = 0  # f = g + h

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
        Parameters:
        -----------
        buildings : GeoDataFrame
            Budynki z OSM (geometry + wysokości)
        config : DroneConfig
            Konfiguracja drona
        """
        self.buildings = buildings
        self.config = config

        # Parametry gridu
        self.grid_resolution = config.grid_resolution
        self.altitude_min = config.altitude_cruise_min
        self.altitude_max = config.altitude_cruise_max
        self.avoid_distance = config.avoid_distance

        # Grid będzie stworzony w plan_path()
        self.occupancy_grid = None
        self.grid_origin = None  # (x_min, y_min, z_min)
        self.grid_shape = None   # (nx, ny, nz)

    def latlon_to_meters(self, lat, lon, alt, ref_lat, ref_lon):
        """Konwertuj lat/lon na metry względem punktu odniesienia."""
        y = (lat - ref_lat) * 111000
        x = (lon - ref_lon) * 111000 * np.cos(np.radians(ref_lat))
        z = alt
        return x, y, z

    def meters_to_latlon(self, x, y, z, ref_lat, ref_lon):
        """Konwertuj metry na lat/lon."""
        lat = ref_lat + y / 111000
        lon = ref_lon + x / (111000 * np.cos(np.radians(ref_lat)))
        alt = z
        return lat, lon, alt

    def estimate_building_height(self, building):
        """Szacuj wysokość budynku."""
        height = None

        # Spróbuj height
        if 'height' in building.index and not np.isnan(building.get('height', np.nan)):
            try:
                h = str(building['height']).replace('m', '').replace('M', '').strip()
                height = float(h)
            except:
                pass

        # Spróbuj building:levels
        if height is None and 'building:levels' in building.index:
            try:
                levels = int(building['building:levels'])
                building_type = building.get('building', 'yes')

                # Różne wysokości dla różnych typów
                if building_type in ['apartments', 'residential']:
                    height = levels * 3.0
                elif building_type == 'office':
                    height = levels * 4.0
                elif building_type == 'commercial':
                    height = levels * 4.5
                else:
                    height = levels * 3.5
            except:
                pass

        # Domyślnie w zależności od typu
        if height is None:
            building_type = building.get('building', 'yes')
            defaults = {
                'house': 6.0,
                'garage': 3.0,
                'shed': 2.5,
                'commercial': 12.0,
                'industrial': 8.0,
                'apartments': 15.0,
                'office': 20.0,
                'yes': 10.0
            }
            height = defaults.get(building_type, 10.0)

        return height

    def create_occupancy_grid(self, bbox, ref_lat, ref_lon):
        """
        Stwórz 3D occupancy grid.

        Parameters:
        -----------
        bbox : tuple
            (x_min, x_max, y_min, y_max) w metrach
        ref_lat, ref_lon : float
            Punkt odniesienia dla konwersji

        Returns:
        --------
        grid : ndarray
            3D boolean array, True = occupied
        """
        x_min, x_max, y_min, y_max = bbox

        # Rozmiar gridu
        nx = int((x_max - x_min) / self.grid_resolution) + 1
        ny = int((y_max - y_min) / self.grid_resolution) + 1
        nz = int((self.altitude_max - self.altitude_min) / self.grid_resolution) + 1

        print(f"\n[Grid] Wymiary: {nx} x {ny} x {nz} = {nx*ny*nz:,} komórek")
        print(f"[Grid] Rozdzielczość: {self.grid_resolution}m")
        print(f"[Grid] Wysokości: {self.altitude_min}m - {self.altitude_max}m")

        # Inicjalizacja gridu (False = wolne)
        grid = np.zeros((nx, ny, nz), dtype=bool)

        # Zapisz parametry gridu
        self.grid_origin = (x_min, y_min, self.altitude_min)
        self.grid_shape = (nx, ny, nz)

        # Oznacz budynki jako occupied
        buildings_marked = 0

        # FILTRUJ budynki do bounding box (konwersja metry -> lat/lon)
        lat_min, lon_min, _ = self.meters_to_latlon(x_min, y_min, 0, ref_lat, ref_lon)
        lat_max, lon_max, _ = self.meters_to_latlon(x_max, y_max, 0, ref_lat, ref_lon)

        try:
            buildings_in_bbox = self.buildings.cx[lon_min:lon_max, lat_min:lat_max]
            print(f"[Grid] Budynków w obszarze: {len(buildings_in_bbox)} (z {len(self.buildings):,} całkowitych)")
        except:
            # Jeśli cx nie działa, użyj prostego filtra
            buildings_in_bbox = self.buildings[
                (self.buildings.geometry.bounds['minx'] <= lon_max) &
                (self.buildings.geometry.bounds['maxx'] >= lon_min) &
                (self.buildings.geometry.bounds['miny'] <= lat_max) &
                (self.buildings.geometry.bounds['maxy'] >= lat_min)
            ]
            print(f"[Grid] Budynków w obszarze: {len(buildings_in_bbox)} (z {len(self.buildings):,} całkowitych)")

        for idx, building in buildings_in_bbox.iterrows():
            try:
                height = self.estimate_building_height(building)

                # Dodaj margines bezpieczeństwa
                height_with_margin = height + self.avoid_distance

                # Sprawdź czy budynek jest w bounding box
                geom = building.geometry

                # Dla każdej komórki gridu w XY
                for i in range(nx):
                    x = x_min + i * self.grid_resolution

                    for j in range(ny):
                        y = y_min + j * self.grid_resolution

                        # Konwertuj do lat/lon
                        lat, lon, _ = self.meters_to_latlon(x, y, 0, ref_lat, ref_lon)
                        point = Point(lon, lat)

                        # Sprawdź czy punkt jest w budynku (lub blisko)
                        distance = geom.distance(point)
                        distance_meters = distance * 111000  # Przybliżenie

                        if distance_meters < self.avoid_distance:
                            # Oznacz wszystkie wysokości od 0 do height_with_margin
                            for k in range(nz):
                                z = self.altitude_min + k * self.grid_resolution
                                if z < height_with_margin:
                                    grid[i, j, k] = True

                buildings_marked += 1

                if buildings_marked % 100 == 0:
                    print(f"[Grid] Przetworzono {buildings_marked}/{len(buildings_in_bbox)} budynków...")

            except Exception as e:
                continue

        occupied_cells = np.sum(grid)
        occupancy_ratio = occupied_cells / (nx * ny * nz) * 100

        print(f"[Grid] Budynków: {buildings_marked}")
        print(f"[Grid] Zajętych komórek: {occupied_cells:,} ({occupancy_ratio:.1f}%)")

        return grid

    def world_to_grid(self, x, y, z):
        """Konwertuj współrzędne świata (metry) na indeksy gridu."""
        i = int((x - self.grid_origin[0]) / self.grid_resolution)
        j = int((y - self.grid_origin[1]) / self.grid_resolution)
        k = int((z - self.grid_origin[2]) / self.grid_resolution)
        return (i, j, k)

    def grid_to_world(self, i, j, k):
        """Konwertuj indeksy gridu na współrzędne świata (metry)."""
        x = self.grid_origin[0] + i * self.grid_resolution
        y = self.grid_origin[1] + j * self.grid_resolution
        z = self.grid_origin[2] + k * self.grid_resolution
        return (x, y, z)

    def is_valid_node(self, position):
        """Sprawdź czy węzeł jest w gridzie i nie jest occupied."""
        i, j, k = position
        nx, ny, nz = self.grid_shape

        if i < 0 or i >= nx or j < 0 or j >= ny or k < 0 or k >= nz:
            return False

        return not self.occupancy_grid[i, j, k]

    def get_neighbors(self, node):
        """Pobierz sąsiadów węzła (26-connectivity w 3D)."""
        neighbors = []
        i, j, k = node.position

        # 26 kierunków (3^3 - 1)
        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                for dk in [-1, 0, 1]:
                    if di == 0 and dj == 0 and dk == 0:
                        continue

                    new_pos = (i + di, j + dj, k + dk)

                    if self.is_valid_node(new_pos):
                        neighbors.append(Node(new_pos, node))

        return neighbors

    def heuristic(self, pos1, pos2):
        """Heurystyka: Euklidesowa odległość w gridzie."""
        return np.sqrt((pos1[0] - pos2[0])**2 +
                      (pos1[1] - pos2[1])**2 +
                      (pos1[2] - pos2[2])**2)

    def movement_cost(self, from_pos, to_pos):
        """Koszt ruchu między węzłami z penalties."""
        # Podstawowy koszt: odległość
        dx = abs(to_pos[0] - from_pos[0])
        dy = abs(to_pos[1] - from_pos[1])
        dz = abs(to_pos[2] - from_pos[2])

        # Diagonal = sqrt(2), diagonal 3D = sqrt(3)
        if dx + dy + dz == 1:
            base_cost = 1.0
        elif dx + dy + dz == 2:
            base_cost = 1.414  # sqrt(2)
        else:
            base_cost = 1.732  # sqrt(3)

        # Penalty za zmianę wysokości (preferuj lot poziomy)
        altitude_penalty = abs(dz) * 0.5

        # Penalty za latanie zbyt wysoko (zużycie energii)
        _, _, z = self.grid_to_world(to_pos[0], to_pos[1], to_pos[2])
        height_penalty = (z - self.altitude_min) * 0.01

        return base_cost + altitude_penalty + height_penalty

    def astar_3d(self, start_pos, goal_pos):
        """
        3D A* algorithm.

        Parameters:
        -----------
        start_pos : tuple
            (i, j, k) w gridzie
        goal_pos : tuple
            (i, j, k) w gridzie

        Returns:
        --------
        path : list of tuples
            Lista współrzędnych gridu [(i,j,k), ...] lub None jeśli nie ma ścieżki
        """
        print("\n[A*] Rozpoczęcie planowania...")

        start_node = Node(start_pos)
        goal_node = Node(goal_pos)

        # Open i closed sets
        open_list = []
        heapq.heappush(open_list, (0, start_node))
        closed_set = set()

        # Dla szybszego wyszukiwania w open_list
        open_dict = {start_pos: start_node}

        iterations = 0
        max_iterations = 100000

        while open_list and iterations < max_iterations:
            iterations += 1

            if iterations % 5000 == 0:
                print(f"[A*] Iteracja {iterations}, open_list: {len(open_list)}")

            # Pobierz węzeł z najmniejszym f
            _, current_node = heapq.heappop(open_list)

            # Usuń z open_dict
            if current_node.position in open_dict:
                del open_dict[current_node.position]

            # Sprawdź czy dotarliśmy do celu
            if current_node.position == goal_pos:
                print(f"[A*] ✓ Znaleziono ścieżkę po {iterations} iteracjach!")

                # Rekonstruuj ścieżkę
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                path.reverse()

                return path

            closed_set.add(current_node.position)

            # Sprawdź wszystkich sąsiadów
            for neighbor in self.get_neighbors(current_node):
                if neighbor.position in closed_set:
                    continue

                # Oblicz g, h, f
                neighbor.g = current_node.g + self.movement_cost(current_node.position, neighbor.position)
                neighbor.h = self.heuristic(neighbor.position, goal_pos)
                neighbor.f = neighbor.g + neighbor.h

                # Sprawdź czy już jest w open_list z lepszym kosztem
                if neighbor.position in open_dict:
                    existing = open_dict[neighbor.position]
                    if neighbor.g >= existing.g:
                        continue

                # Dodaj do open_list
                heapq.heappush(open_list, (neighbor.f, neighbor))
                open_dict[neighbor.position] = neighbor

        print(f"[A*] ✗ Nie znaleziono ścieżki po {iterations} iteracjach")
        return None

    def smooth_path(self, path_grid):
        """
        Wygładź ścieżkę używając B-spline.

        Parameters:
        -----------
        path_grid : list of tuples
            Ścieżka w gridzie [(i,j,k), ...]

        Returns:
        --------
        smooth_path : ndarray
            Wygładzona ścieżka (N, 3) w metrach
        """
        print("\n[Smoothing] Wygładzanie ścieżki...")

        # Konwertuj grid → world coordinates
        path_world = np.array([self.grid_to_world(i, j, k) for i, j, k in path_grid])

        # Jeśli ścieżka jest krótka, nie wygładzaj
        if len(path_world) < 4:
            print("[Smoothing] Ścieżka zbyt krótka, pomijam wygładzanie")
            return path_world

        # B-spline interpolacja
        try:
            # Parametryzacja (0 do 1)
            tck, u = splprep([path_world[:, 0], path_world[:, 1], path_world[:, 2]],
                            s=len(path_world) * self.grid_resolution,  # Smoothing factor
                            k=min(3, len(path_world) - 1))  # Cubic spline jeśli możliwe

            # Próbkuj z większą gęstością
            u_fine = np.linspace(0, 1, len(path_world) * 5)
            smooth_x, smooth_y, smooth_z = splev(u_fine, tck)

            smooth_path = np.column_stack([smooth_x, smooth_y, smooth_z])

            print(f"[Smoothing] ✓ Wygładzono: {len(path_grid)} → {len(smooth_path)} punktów")

            return smooth_path

        except Exception as e:
            print(f"[Smoothing] ⚠ Błąd podczas wygładzania: {e}")
            return path_world

    def plan_path(self, lat_A, lon_A, lat_B, lon_B):
        """
        Główna funkcja planowania ścieżki.

        Parameters:
        -----------
        lat_A, lon_A : float
            Punkt startowy
        lat_B, lon_B : float
            Punkt końcowy

        Returns:
        --------
        path_result : dict
            {
                'path_latlon': [(lat, lon, alt), ...],
                'path_meters': [(x, y, z), ...],
                'distance': float,
                'ref_point': (lat, lon)
            }
        """
        print("\n" + "=" * 70)
        print("3D A* PATH PLANNING")
        print("=" * 70)

        # Punkt odniesienia: punkt A
        ref_lat, ref_lon = lat_A, lon_A

        # Konwertuj punkty A i B na metry
        start_x, start_y, _ = self.latlon_to_meters(lat_A, lon_A, 0, ref_lat, ref_lon)
        goal_x, goal_y, _ = self.latlon_to_meters(lat_B, lon_B, 0, ref_lat, ref_lon)

        # Wysokości startowa i końcowa
        start_z = self.config.altitude_start
        goal_z = self.config.altitude_end

        print(f"\n[Setup] Start: ({start_x:.1f}, {start_y:.1f}, {start_z:.1f})m")
        print(f"[Setup] Goal:  ({goal_x:.1f}, {goal_y:.1f}, {goal_z:.1f})m")

        # Oblicz bounding box z marginesem
        margin = 50  # metry
        x_min = min(start_x, goal_x) - margin
        x_max = max(start_x, goal_x) + margin
        y_min = min(start_y, goal_y) - margin
        y_max = max(start_y, goal_y) + margin

        bbox = (x_min, x_max, y_min, y_max)

        print(f"[Setup] BBox: X=[{x_min:.1f}, {x_max:.1f}], Y=[{y_min:.1f}, {y_max:.1f}]")

        # Stwórz occupancy grid
        self.occupancy_grid = self.create_occupancy_grid(bbox, ref_lat, ref_lon)

        # Konwertuj start/goal na grid coordinates
        start_grid = self.world_to_grid(start_x, start_y, start_z)
        goal_grid = self.world_to_grid(goal_x, goal_y, goal_z)

        print(f"\n[Setup] Start grid: {start_grid}")
        print(f"[Setup] Goal grid:  {goal_grid}")

        # Sprawdź czy start i goal są wolne
        if not self.is_valid_node(start_grid):
            print("[ERROR] ✗ Punkt startowy jest w przeszkodzie!")
            # Spróbuj znaleźć najbliższy wolny węzeł
            start_grid = self.find_nearest_free_node(start_grid)
            print(f"[Fix] Użyto najbliższego wolnego: {start_grid}")

        if not self.is_valid_node(goal_grid):
            print("[ERROR] ✗ Punkt końcowy jest w przeszkodzie!")
            goal_grid = self.find_nearest_free_node(goal_grid)
            print(f"[Fix] Użyto najbliższego wolnego: {goal_grid}")

        # Uruchom A*
        path_grid = self.astar_3d(start_grid, goal_grid)

        if path_grid is None:
            print("\n[ERROR] ✗ Nie znaleziono ścieżki!")
            return None

        # Wygładź ścieżkę
        path_smooth = self.smooth_path(path_grid)

        # Konwertuj z powrotem na lat/lon
        path_latlon = []
        for x, y, z in path_smooth:
            lat, lon, alt = self.meters_to_latlon(x, y, z, ref_lat, ref_lon)
            path_latlon.append((lat, lon, alt))

        # Oblicz całkowitą długość
        distance = 0
        for i in range(len(path_smooth) - 1):
            dx = path_smooth[i+1, 0] - path_smooth[i, 0]
            dy = path_smooth[i+1, 1] - path_smooth[i, 1]
            dz = path_smooth[i+1, 2] - path_smooth[i, 2]
            distance += np.sqrt(dx**2 + dy**2 + dz**2)

        print(f"\n[Result] ✓ Długość ścieżki: {distance:.1f}m")
        print(f"[Result] ✓ Punktów: {len(path_smooth)}")

        result = {
            'path_latlon': path_latlon,
            'path_meters': path_smooth,
            'distance': distance,
            'ref_point': (ref_lat, ref_lon)
        }

        print("=" * 70)

        return result

    def find_nearest_free_node(self, position):
        """Znajdź najbliższy wolny węzeł do danej pozycji."""
        i, j, k = position

        # Przeszukaj w spirali
        for radius in range(1, 20):
            for di in range(-radius, radius + 1):
                for dj in range(-radius, radius + 1):
                    for dk in range(-radius, radius + 1):
                        test_pos = (i + di, j + dj, k + dk)
                        if self.is_valid_node(test_pos):
                            return test_pos

        # Jeśli nie znaleziono, zwróć oryginalną
        return position

    def visualize_path_3d(self, path_result):
        """Wizualizuj zaplanowaną ścieżkę 3D."""
        if path_result is None:
            return

        path = path_result['path_meters']

        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Ścieżka
        ax.plot(path[:, 0], path[:, 1], path[:, 2],
               'b-', linewidth=2, label='Ścieżka drona')

        # Start i koniec
        ax.scatter(path[0, 0], path[0, 1], path[0, 2],
                  c='green', s=200, marker='o', label='Start', zorder=100)
        ax.scatter(path[-1, 0], path[-1, 1], path[-1, 2],
                  c='red', s=200, marker='o', label='Koniec', zorder=100)

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Wysokość [m]')
        ax.set_title('Zaplanowana ścieżka 3D', fontweight='bold', fontsize=14)
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()