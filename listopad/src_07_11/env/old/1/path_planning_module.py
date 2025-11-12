"""
Planowanie ścieżki drona z omijaniem przeszkód
==============================================
Autor: Warsaw Drone Simulation Project

Ten moduł:
- Planuje ścieżkę 3D z punktu A do B
- Omija budynki i inne przeszkody
- Minimalizuje długość trasy
- Uwzględnia wysokości startową, przelotową i końcową
"""

import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import List, Tuple, Optional
from geopy.distance import geodesic
import pickle
from pathlib import Path


@dataclass
class Waypoint3D:
    """Punkt trasy w 3D."""
    lat: float
    lon: float
    alt: float  # wysokość nad ziemią [m]

    def to_tuple(self):
        return (self.lat, self.lon, self.alt)

    def __repr__(self):
        return f"WP({self.lat:.6f}, {self.lon:.6f}, h={self.alt:.1f}m)"


class DronePathPlanner:
    """
    Planowanie ścieżki drona z adaptacyjnym omijaniem przeszkód.

    Zasady:
    - Trzyma się wysokości przelotowej gdy to możliwe
    - Omija przeszkody wybierając krótszą opcję (góra vs bok)
    - Koszt = długość trasy w 3D
    - Uwzględnia wysokość obiektów w punktach start/cel
    """

    def __init__(self,
                 buildings_gdf,
                 avoid_distance: float = 5.0,
                 step_size: float = 10.0,
                 vertical_cost_multiplier: float = 1.5):
        """
        Args:
            buildings_gdf: GeoDataFrame z budynkami
            avoid_distance: Minimalna odległość od przeszkód [m]
            step_size: Krok próbkowania wzdłuż trasy [m]
            vertical_cost_multiplier: Mnożnik kosztu dla ruchu pionowego
        """
        self.buildings = buildings_gdf
        self.avoid_distance = avoid_distance
        self.step_size = step_size
        self.vertical_cost = vertical_cost_multiplier

        # Przetwórz budynki - wyciągnij wysokości
        self._preprocess_buildings()

    def _preprocess_buildings(self):
        """Przygotuj dane o wysokościach budynków."""
        if self.buildings is None or len(self.buildings) == 0:
            self.buildings_with_heights = None
            return

        print("Przetwarzanie danych budynków...")

        heights = []
        for idx, building in self.buildings.iterrows():
            height = self._extract_height(building)
            heights.append(height)

        self.buildings = self.buildings.copy()
        self.buildings['height_m'] = heights

        print(f"  ✓ Przetworzono {len(self.buildings)} budynków")
        print(f"  ℹ Zakres wysokości: {min(heights):.1f}m - {max(heights):.1f}m")

    def _extract_height(self, building) -> float:
        """Wyciągnij wysokość budynku."""
        height = None

        # Próbuj z 'height'
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

        # Domyślnie
        if height is None:
            height = 10.0

        return height

    def get_height_at_point(self, lat: float, lon: float) -> float:
        """
        Sprawdź czy punkt znajduje się na budynku i zwróć jego wysokość.

        Returns:
            Wysokość budynku [m] lub 0.0 jeśli punkt nie jest na budynku
        """
        if self.buildings is None or len(self.buildings) == 0:
            return 0.0

        from shapely.geometry import Point
        point = Point(lon, lat)

        # Sprawdź czy punkt jest wewnątrz któregoś budynku
        for idx, building in self.buildings.iterrows():
            if building.geometry.contains(point):
                return building['height_m']

        return 0.0

    def check_collision(self, lat: float, lon: float, alt: float) -> Tuple[bool, float]:
        """
        Sprawdź czy punkt koliduje z budynkiem.

        Returns:
            (collision, building_height): (True/False, wysokość budynku lub 0)
        """
        if self.buildings is None or len(self.buildings) == 0:
            return False, 0.0

        from shapely.geometry import Point
        from shapely.ops import nearest_points

        point = Point(lon, lat)

        # Sprawdź kolizję z każdym budynkiem
        for idx, building in self.buildings.iterrows():
            geom = building.geometry
            building_height = building['height_m']

            # Sprawdź czy punkt jest blisko budynku (2D)
            distance_2d = point.distance(geom) * 111000  # stopnie → metry (przybliżenie)

            if distance_2d < self.avoid_distance:
                # Jesteśmy blisko budynku - sprawdź wysokość
                if alt < building_height + self.avoid_distance:
                    return True, building_height

        return False, 0.0

    def plan_path(self,
                  point_a: Tuple[float, float],  # (lat, lon)
                  point_b: Tuple[float, float],
                  h_start: float = 0.0,
                  h_cruise: float = 30.0,
                  h_end: float = 0.0) -> List[Waypoint3D]:
        """
        Zaplanuj ścieżkę 3D od A do B z omijaniem przeszkód.

        Args:
            point_a: (lat, lon) punkt startowy
            point_b: (lat, lon) punkt końcowy
            h_start: Wysokość startowa [m]
            h_cruise: Wysokość przelotowa [m]
            h_end: Wysokość końcowa [m]

        Returns:
            Lista waypoints 3D
        """
        print("\n" + "=" * 70)
        print("PLANOWANIE ŚCIEŻKI DRONA")
        print("=" * 70)

        path = []

        # 1. Sprawdź czy punkty są na budynkach
        building_height_a = self.get_height_at_point(point_a[0], point_a[1])
        building_height_b = self.get_height_at_point(point_b[0], point_b[1])

        print(f"\n📍 Punkt A: {point_a}")
        if building_height_a > 0:
            print(f"   ⚠ NA BUDYNKU! Wysokość: {building_height_a:.1f}m")
            print(f"   → Start z wysokości budynku")
            h_start = building_height_a
        else:
            print(f"   ✓ Na gruncie, start z h={h_start:.1f}m")

        print(f"\n📍 Punkt B: {point_b}")
        if building_height_b > 0:
            print(f"   ⚠ NA BUDYNKU! Wysokość: {building_height_b:.1f}m")
            print(f"   → Cel na wysokości budynku")
            h_end = building_height_b
        else:
            print(f"   ✓ Na gruncie, lądowanie na h={h_end:.1f}m")

        # 2. Dodaj punkt startowy
        current_lat, current_lon = point_a
        current_alt = h_start
        path.append(Waypoint3D(current_lat, current_lon, current_alt))

        # 3. Wznieś się do wysokości przelotowej (jeśli nie jesteśmy już wyżej)
        if current_alt < h_cruise:
            current_alt = h_cruise
            path.append(Waypoint3D(current_lat, current_lon, current_alt))
            print(f"\n🚁 Wznoszenie: {h_start:.1f}m → {h_cruise:.1f}m")

        # 4. Zaplanuj trasę poziomą z omijaniem przeszkód
        print(f"\n🗺️ Planowanie trasy poziomej...")
        horizontal_path = self._plan_horizontal_path(
            (current_lat, current_lon, current_alt),
            point_b,
            h_cruise
        )

        path.extend(horizontal_path)

        # 5. Opadnij do wysokości końcowej
        final_lat, final_lon = point_b
        if path[-1].alt != h_end:
            print(f"\n🚁 Opadanie: {path[-1].alt:.1f}m → {h_end:.1f}m")
            path.append(Waypoint3D(final_lat, final_lon, h_end))

        # 6. Oblicz całkowitą długość trasy
        total_distance = self._calculate_path_length(path)

        print(f"\n✅ ŚCIEŻKA ZAPLANOWANA!")
        print(f"   - Liczba waypoints: {len(path)}")
        print(f"   - Długość trasy: {total_distance:.2f} m")
        print(f"   - Wysokość start: {h_start:.1f} m")
        print(f"   - Wysokość przelot: {h_cruise:.1f} m")
        print(f"   - Wysokość koniec: {h_end:.1f} m")

        return path

    def _plan_horizontal_path(self,
                              start_3d: Tuple[float, float, float],
                              goal_2d: Tuple[float, float],
                              cruise_alt: float) -> List[Waypoint3D]:
        """
        Zaplanuj trasę poziomą z omijaniem przeszkód.

        Strategia:
        - Próbuj lecieć prosto na wysokości cruise_alt
        - Jeśli kolizja → znajdź obejście (góra lub bok)
        - Wybierz krótszą opcję
        """
        path = []

        current_lat, current_lon, current_alt = start_3d
        goal_lat, goal_lon = goal_2d

        max_iterations = 1000
        iteration = 0

        while iteration < max_iterations:
            iteration += 1

            # Oblicz odległość do celu
            dist_to_goal = geodesic((current_lat, current_lon), (goal_lat, goal_lon)).meters

            if dist_to_goal < self.step_size:
                # Dotarliśmy do celu!
                path.append(Waypoint3D(goal_lat, goal_lon, current_alt))
                break

            # Kierunek do celu
            dlat = goal_lat - current_lat
            dlon = goal_lon - current_lon
            norm = np.sqrt(dlat ** 2 + dlon ** 2)
            dlat /= norm
            dlon /= norm

            # Następny punkt na linii prostej
            step_lat = current_lat + dlat * (self.step_size / 111000)
            step_lon = current_lon + dlon * (self.step_size / 111000)

            # Sprawdź kolizję
            collision, obstacle_height = self.check_collision(step_lat, step_lon, current_alt)

            if not collision:
                # Brak kolizji - leć prosto
                current_lat, current_lon = step_lat, step_lon
                path.append(Waypoint3D(current_lat, current_lon, current_alt))
            else:
                # KOLIZJA! Znajdź obejście
                print(f"   ⚠ Wykryto przeszkodę h={obstacle_height:.1f}m, szukam obejścia...")

                waypoint = self._find_avoidance(
                    (current_lat, current_lon, current_alt),
                    (goal_lat, goal_lon),
                    obstacle_height
                )

                if waypoint:
                    current_lat, current_lon, current_alt = waypoint.lat, waypoint.lon, waypoint.alt
                    path.append(waypoint)
                else:
                    # Nie znaleziono obejścia - wznieś się awaryjnie
                    print(f"   ⚠ Brak obejścia, wznoszę się awaryjnie +20m")
                    current_alt += 20.0
                    path.append(Waypoint3D(current_lat, current_lon, current_alt))

        return path

    def _find_avoidance(self,
                        current: Tuple[float, float, float],
                        goal: Tuple[float, float],
                        obstacle_height: float) -> Optional[Waypoint3D]:
        """
        Znajdź najlepsze obejście przeszkody.

        Opcje:
        1. Wznieś się ponad przeszkodę
        2. Omiń z lewej
        3. Omiń z prawej

        Wybierz opcję o najmniejszym koszcie (długość trasy).
        """
        current_lat, current_lon, current_alt = current
        goal_lat, goal_lon = goal

        options = []

        # Kierunek do celu
        dlat = goal_lat - current_lat
        dlon = goal_lon - current_lon
        norm = np.sqrt(dlat ** 2 + dlon ** 2)
        dir_lat = dlat / norm
        dir_lon = dlon / norm

        # Kierunek prostopadły (lewo/prawo)
        perp_lat = -dir_lon
        perp_lon = dir_lat

        # OPCJA 1: Wznieś się ponad przeszkodę
        new_alt = obstacle_height + self.avoid_distance + 5.0  # +5m margines
        height_diff = new_alt - current_alt

        if height_diff > 0:
            # Koszt wznoszenia = pionowa odległość × mnożnik
            cost_up = height_diff * self.vertical_cost
            options.append(('up', Waypoint3D(current_lat, current_lon, new_alt), cost_up))

        # OPCJA 2 & 3: Omiń z boku
        lateral_offset = self.avoid_distance + 10.0  # +10m obejście

        # Lewo
        left_lat = current_lat + perp_lat * (lateral_offset / 111000)
        left_lon = current_lon + perp_lon * (lateral_offset / 111000)

        collision_left, _ = self.check_collision(left_lat, left_lon, current_alt)
        if not collision_left:
            # Oblicz dodatkową długość trasy
            dist_to_left = geodesic((current_lat, current_lon), (left_lat, left_lon)).meters
            cost_left = dist_to_left
            options.append(('left', Waypoint3D(left_lat, left_lon, current_alt), cost_left))

        # Prawo
        right_lat = current_lat - perp_lat * (lateral_offset / 111000)
        right_lon = current_lon - perp_lon * (lateral_offset / 111000)

        collision_right, _ = self.check_collision(right_lat, right_lon, current_alt)
        if not collision_right:
            dist_to_right = geodesic((current_lat, current_lon), (right_lat, right_lon)).meters
            cost_right = dist_to_right
            options.append(('right', Waypoint3D(right_lat, right_lon, current_alt), cost_right))

        # Wybierz najtańszą opcję
        if options:
            best = min(options, key=lambda x: x[2])
            direction, waypoint, cost = best
            print(f"      → Wybieram: {direction} (koszt: {cost:.1f}m)")
            return waypoint

        return None

    def _calculate_path_length(self, path: List[Waypoint3D]) -> float:
        """Oblicz całkowitą długość ścieżki 3D."""
        total = 0.0

        for i in range(len(path) - 1):
            wp1 = path[i]
            wp2 = path[i + 1]

            # Odległość horyzontalna
            dist_2d = geodesic((wp1.lat, wp1.lon), (wp2.lat, wp2.lon)).meters

            # Różnica wysokości
            dist_vertical = abs(wp2.alt - wp1.alt)

            # Odległość 3D (Pitagoras)
            dist_3d = np.sqrt(dist_2d ** 2 + dist_vertical ** 2)

            total += dist_3d

        return total


def main():
    """Przykład użycia planera ścieżki."""
    DATA_DIR = Path("data")

    print("=" * 70)
    print("PLANER ŚCIEŻKI DRONA - TEST")
    print("=" * 70)

    # Załaduj budynki
    buildings_path = DATA_DIR / "warsaw_buildings.pkl"
    if not buildings_path.exists():
        print("\n✗ Brak pliku z budynkami!")
        print("  Uruchom najpierw: 01_download_warsaw_data.py")
        return

    buildings = pd.read_pickle(buildings_path)
    print(f"\n✓ Załadowano {len(buildings)} budynków")

    # Stwórz planer
    planner = DronePathPlanner(
        buildings_gdf=buildings,
        avoid_distance=5.0,
        step_size=10.0,
        vertical_cost_multiplier=1.5
    )

    # Przykładowe punkty (musisz podać swoje!)
    point_a = (52.2297, 21.0122)  # Przykład
    point_b = (52.2340, 21.0189)

    # Zaplanuj ścieżkę
    path = planner.plan_path(
        point_a=point_a,
        point_b=point_b,
        h_start=0.0,
        h_cruise=30.0,
        h_end=0.0
    )

    # Wyświetl waypoints
    print("\n" + "=" * 70)
    print("WAYPOINTS:")
    print("=" * 70)
    for i, wp in enumerate(path):
        print(f"{i:3d}. {wp}")

    return path


if __name__ == "__main__":
    main()