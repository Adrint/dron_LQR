"""
Planowanie ścieżki drona z omijaniem przeszkód
==============================================
Autor: Warsaw Drone Simulation Project

Funkcje:
- Wykrywanie budynków pod punktami startu/lądowania
- Planowanie ścieżki z omijaniem przeszkód
- Inteligentny wybór: omijanie bokiem vs wznoszenie
- Wizualizacja 3D trajektorii
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass
from typing import List, Tuple, Optional
from shapely.geometry import Point, LineString
from geopy.distance import geodesic
from pyproj import Transformer
from shapely.ops import unary_union


@dataclass
class Waypoint:
    """Punkt kontrolny na trasie."""
    lon: float
    lat: float
    alt: float  # wysokość nad ziemią [m]
    action: str = "fly"  # fly, takeoff, landing


@dataclass
class PathPlannerConfig:
    """Konfiguracja planera ścieżki."""
    h_start: float = 0.0  # wysokość startowa [m]
    h_cruise: float = 30.0  # wysokość przelotowa [m]
    h_end: float = 0.0  # wysokość końcowa [m]
    avoid_distance: float = 2.0  # margines od przeszkód [m]
    step_size: float = 5.0  # krok próbkowania [m]
    vertical_penalty: float = 5.0  # kara za wznoszenie (koszt × penalty)



from pyproj import Transformer
from shapely.ops import unary_union

class _Projector:
    """WGS84 <-> EPSG:2180 (metry)"""
    def __init__(self):
        self.to_m = Transformer.from_crs("EPSG:4326","EPSG:2180",always_xy=True)
        self.to_deg = Transformer.from_crs("EPSG:2180","EPSG:4326",always_xy=True)
    def ll_to_m(self, lat, lon):
        x,y = self.to_m.transform(lon, lat)
        return x,y
    def m_to_ll(self, x, y):
        lon,lat = self.to_deg.transform(x, y)
        return lat,lon

class DronePathPlanner:
    """
    Planer ścieżki drona z omijaniem przeszkód.
    """

    def __init__(self, buildings_gdf, config: PathPlannerConfig):
        """
        Args:
            buildings_gdf: GeoDataFrame z budynkami
            config: Konfiguracja planera
        """
        self.buildings = buildings_gdf
        self.config = config

        # Przygotuj wysokości budynków
        self._prepare_building_heights()

    def _prepare_building_heights(self):
        """Przygotuj dane o wysokościach budynków."""
        if self.buildings is None or len(self.buildings) == 0:
            return

        heights = []
        for idx, building in self.buildings.iterrows():
            height = self._extract_building_height(building)
            heights.append(height)

        self.buildings['height_m'] = heights
        # --- PATCH: precompute XY buffers in meters for avoid_distance ---
        self._proj = _Projector()
        buffers = []
        for _, b in self.buildings.iterrows():
            g = getattr(b, 'geometry', None)
            if g is None:
                buffers.append(None); continue
            try:
                if g.geom_type == 'Polygon':
                    ext = [(self._proj.ll_to_m(lat=pt[1], lon=pt[0])) for pt in g.exterior.coords]
                    poly = type(g)([ext])
                elif g.geom_type == 'MultiPolygon':
                    parts = []
                    for sub in g.geoms:
                        ext = [(self._proj.ll_to_m(lat=pt[1], lon=pt[0])) for pt in sub.exterior.coords]
                        parts.append(type(sub)([ext]))
                    poly = type(g)(parts)
                else:
                    buffers.append(None); continue
                buf = poly.buffer(self.config.avoid_distance)
                buffers.append(buf)
            except Exception:
                buffers.append(None)
        self.buildings['buffer_xy'] = buffers
        valid = [b for b in buffers if b is not None]
        self._union_buffer = unary_union(valid) if valid else None


    def _extract_building_height(self, building) -> float:
        """Wyciągnij wysokość budynku z danych OSM."""
        # Próbuj z tagu 'height'
        if 'height' in building.index and not pd.isna(building['height']):
            try:
                h = str(building['height'])
                h = h.replace('m', '').replace('M', '').strip()
                return float(h)
            except:
                pass

        # Szacuj z liczby pięter
        if 'building:levels' in building.index:
            try:
                levels = int(building['building:levels'])
                return levels * 3.0
            except:
                pass

        # Domyślnie
        return 10.0

    def get_building_at_point(self, lat: float, lon: float) -> Optional[dict]:
        """
        Sprawdź czy w punkcie (lat, lon) znajduje się budynek.

        Returns:
            dict z 'height' i 'geometry' lub None
        """
        if self.buildings is None or len(self.buildings) == 0:
            return None

        point = Point(lon, lat)

        for idx, building in self.buildings.iterrows():
            if building.geometry.contains(point):
                return {
                    'height': building['height_m'],
                    'geometry': building.geometry,
                    'index': idx
                }

        return None

    def check_collision(self, lat: float, lon: float, alt: float) -> bool:
        """
        Sprawdź czy punkt (lat, lon, alt) koliduje z budynkiem.

        Uwzględnia avoid_distance jako bufor.
        """
        if self.buildings is None or len(self.buildings) == 0:
            return False

        point = Point(lon, lat)
        # --- PATCH: metric collision using precomputed XY buffers ---
        if self._union_buffer is not None:
            x,y = self._proj.ll_to_m(lat, lon)
            p_xy = Point(x,y)
            if not self._union_buffer.contains(p_xy):
                return False
        for idx, building in self.buildings.iterrows():
            buf = building.get('buffer_xy', None)
            if buf is None:
                continue
            x,y = self._proj.ll_to_m(lat, lon)
            p_xy = Point(x,y)
            if buf.contains(p_xy):
                if alt < building['height_m'] + self.config.avoid_distance:
                    return True

        return False

    def find_safe_altitude(self, lat: float, lon: float) -> float:
        """
        Znajdź bezpieczną wysokość przelotu w danym punkcie.

        Returns:
            Minimalna bezpieczna wysokość [m]
        """
        if self.buildings is None or len(self.buildings) == 0:
            return self.config.h_cruise

        point = Point(lon, lat)
        max_height = 0.0
        if self._union_buffer is None:
            return max(self.config.h_cruise, 0.0)
        x,y = self._proj.ll_to_m(lat, lon)
        p_xy = Point(x,y)
        if not self._union_buffer.contains(p_xy):
            return max(self.config.h_cruise, 0.0)
        for _, building in self.buildings.iterrows():
            buf = building.get('buffer_xy', None)
            if buf is None: continue
            if buf.contains(p_xy):
                required = building['height_m'] + self.config.avoid_distance
                if required > max_height:
                    max_height = required
        return max(max_height, self.config.h_cruise)

    def plan_path(self,
                  point_a: Tuple[float, float],  # (lat, lon)
                  point_b: Tuple[float, float]) -> List[Waypoint]:
        """
        Zaplanuj ścieżkę od A do B z omijaniem przeszkód.

        Args:
            point_a: (lat, lon) punkt startowy
            point_b: (lat, lon) punkt końcowy

        Returns:
            Lista waypoints
        """
        print("\n" + "=" * 70)
        print("PLANOWANIE ŚCIEŻKI Z OMIJANIEM PRZESZKÓD")
        print("=" * 70)

        lat_a, lon_a = point_a
        lat_b, lon_b = point_b

        # 1. Sprawdź czy punkty są na budynkach
        building_a = self.get_building_at_point(lat_a, lon_a)
        building_b = self.get_building_at_point(lat_b, lon_b)

        # Określ wysokości startu i lądowania
        if building_a:
            h_start = building_a['height']
            print(f"\n📍 Punkt A na budynku (h={h_start:.1f}m)")
            print(f"   → Start z dachu budynku")
        else:
            h_start = self.config.h_start
            print(f"\n📍 Punkt A na gruncie (h={h_start:.1f}m)")

        if building_b:
            h_end = building_b['height']
            print(f"📍 Punkt B na budynku (h={h_end:.1f}m)")
            print(f"   → Lądowanie na dachu budynku")
        else:
            h_end = self.config.h_end
            print(f"📍 Punkt B na gruncie (h={h_end:.1f}m)")

        # 2. Inicjalizuj ścieżkę
        path = []

        # Punkt startowy
        current = np.array([lat_a, lon_a, h_start])
        path.append(Waypoint(lon_a, lat_a, h_start, action="takeoff"))

        # 3. Wznieś się do wysokości przelotowej
        h_safe_start = self.find_safe_altitude(lat_a, lon_a)
        cruise_alt = max(self.config.h_cruise, h_safe_start)

        if abs(current[2] - cruise_alt) > 0.5:  # Jeśli różnica > 0.5m
            current[2] = cruise_alt
            path.append(Waypoint(lon_a, lat_a, cruise_alt, action="climb"))
            print(f"\n↑ Wznoszenie do h={cruise_alt:.1f}m")

        # 4. Planuj trasę poziomą z omijaniem przeszkód
        print(f"\n🛫 Planowanie trasy poziomej...")

        horizontal_waypoints = self._plan_horizontal_path(
            (lat_a, lon_a), (lat_b, lon_b), cruise_alt
        )

        path.extend(horizontal_waypoints)

        # 5. Opadnij do wysokości lądowania
        if abs(cruise_alt - h_end) > 0.5:
            path.append(Waypoint(lon_b, lat_b, h_end, action="descend"))
            print(f"↓ Opadanie do h={h_end:.1f}m")

        # Punkt końcowy
        path.append(Waypoint(lon_b, lat_b, h_end, action="landing"))

        # 6. Oblicz statystyki
        total_distance = self._calculate_path_length(path)

        print(f"\n" + "=" * 70)
        print(f"✓ Ścieżka zaplanowana!")
        print(f"  - Waypoints: {len(path)}")
        print(f"  - Długość trasy: {total_distance:.2f} m")
        print(f"  - Wysokość przelotu: {cruise_alt:.1f} m")
        print("=" * 70)

        return path, total_distance

    def _plan_horizontal_path(self,
                              start: Tuple[float, float],
                              goal: Tuple[float, float],
                              altitude: float) -> List[Waypoint]:
        """
        Planuj trasę poziomą z omijaniem przeszkód.
        """
        lat_start, lon_start = start
        lat_goal, lon_goal = goal

        waypoints = []

        # Wektor kierunku
        direction = np.array([lat_goal - lat_start, lon_goal - lon_start])
        distance_2d = np.linalg.norm(direction)

        if distance_2d < 0.001:  # Punkty prawie identyczne
            return waypoints

        direction = direction / distance_2d  # Normalizacja

        # Iteruj wzdłuż linii prostej
        current_pos = np.array([lat_start, lon_start])
        current_alt = altitude

        total_m = geodesic((lat_start, lon_start), (lat_goal, lon_goal)).meters
        num_steps = max(1, int(total_m / max(1.0, self.config.step_size)))

        for i in range(1, num_steps + 1):
            # Następny punkt na linii prostej
            t = i / num_steps
            next_pos = np.array([lat_start, lon_start]) + direction * distance_2d * t

            # Sprawdź kolizje
            if self.check_collision(next_pos[0], next_pos[1], current_alt):
                print(f"  ⚠ Przeszkoda wykryta w kroku {i}/{num_steps}")

                # Znajdź opcje omijania
                best_waypoint = self._find_avoidance_maneuver(
                    current_pos, next_pos, current_alt, direction
                )

                if best_waypoint:
                    waypoints.append(best_waypoint)
                    current_pos = np.array([best_waypoint.lat, best_waypoint.lon])
                    current_alt = best_waypoint.alt
            else:
                # Brak kolizji - dodaj waypoint
                waypoints.append(Waypoint(next_pos[1], next_pos[0], current_alt))
                current_pos = next_pos

        return waypoints

    def _find_avoidance_maneuver(self,
                                 current: np.ndarray,
                                 blocked: np.ndarray,
                                 altitude: float,
                                 direction: np.ndarray) -> Optional[Waypoint]:
        """
        Znajdź najlepszy manewr omijania przeszkody.

        Opcje:
        1. Omijanie z lewej
        2. Omijanie z prawej
        3. Wznoszenie
        """
        lat_curr, lon_curr = current

        # Kierunek prostopadły (lewo/prawo)
        perpendicular = np.array([-direction[1], direction[0]])

        options = []

        # OPCJA 1: Lewo
        offset = self.config.avoid_distance / 111000 * 3  # 3× avoid_distance
        left_pos = current + perpendicular * offset
        if not self.check_collision(left_pos[0], left_pos[1], altitude):
            distance = np.linalg.norm(left_pos - current) * 111000
            cost = distance
            options.append(('left', left_pos, altitude, cost))
            print(f"    → Lewo: koszt={cost:.1f}m")

        # OPCJA 2: Prawo
        right_pos = current - perpendicular * offset
        if not self.check_collision(right_pos[0], right_pos[1], altitude):
            distance = np.linalg.norm(right_pos - current) * 111000
            cost = distance
            options.append(('right', right_pos, altitude, cost))
            print(f"    → Prawo: koszt={cost:.1f}m")

        # OPCJA 3: Wznoszenie
        safe_altitude = self.find_safe_altitude(blocked[0], blocked[1])
        if safe_altitude > altitude:
            height_gain = safe_altitude - altitude
            cost = height_gain * self.config.vertical_penalty
            options.append(('up', current, safe_altitude, cost))
            print(f"    → Góra (+{height_gain:.1f}m): koszt={cost:.1f}")

        # Wybierz najtańszą opcję
        if not options:
            print(f"    ✗ Brak opcji omijania!")
            return None

        best = min(options, key=lambda x: x[3])
        action, pos, alt, cost = best

        print(f"    ✓ Wybrano: {action} (koszt={cost:.1f})")

        return Waypoint(pos[1], pos[0], alt, action=f"avoid_{action}")

    def _calculate_path_length(self, waypoints: List[Waypoint]) -> float:
        """Oblicz całkowitą długość ścieżki."""
        if len(waypoints) < 2:
            return 0.0

        total = 0.0

        for i in range(len(waypoints) - 1):
            wp1 = waypoints[i]
            wp2 = waypoints[i + 1]

            # Odległość pozioma (geodezic)
            dist_horiz = geodesic((wp1.lat, wp1.lon), (wp2.lat, wp2.lon)).meters

            # Odległość pionowa
            dist_vert = abs(wp2.alt - wp1.alt)

            # Całkowita odległość (3D)
            dist_3d = np.sqrt(dist_horiz ** 2 + dist_vert ** 2)

            total += dist_3d

        return total


def visualize_path_3d(waypoints: List[Waypoint],
                      buildings_gdf,
                      bbox: dict,
                      title: str = "Trajektoria drona"):
    """
    Wizualizuj ścieżkę drona w 3D.
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 1. Rysuj budynki
    if buildings_gdf is not None and len(buildings_gdf) > 0:
        print("Renderowanie budynków...")

        for idx, building in buildings_gdf.iterrows():
            try:
                geom = building.geometry
                height = building['height_m'] if 'height_m' in building.index else 10.0

                if geom.geom_type == 'Polygon':
                    coords = list(geom.exterior.coords)
                elif geom.geom_type == 'MultiPolygon':
                    coords = list(geom.geoms[0].exterior.coords)
                else:
                    continue

                xs = [c[0] for c in coords]
                ys = [c[1] for c in coords]

                # Podstawa
                ax.plot(xs, ys, 0, 'k-', alpha=0.2, linewidth=0.5)

                # Góra
                ax.plot(xs, ys, height, 'orange', alpha=0.7, linewidth=1)

                # Ściany (próbkuj co 4 punkt)
                for i in range(0, len(coords) - 1, 4):
                    ax.plot([coords[i][0], coords[i][0]],
                            [coords[i][1], coords[i][1]],
                            [0, height],
                            'orange', alpha=0.3, linewidth=0.5)
            except:
                continue

    # 2. Rysuj ścieżkę drona
    if waypoints:
        lons = [wp.lon for wp in waypoints]
        lats = [wp.lat for wp in waypoints]
        alts = [wp.alt for wp in waypoints]

        # Linia trajektorii
        ax.plot(lons, lats, alts,
                'b-', linewidth=3, label='Trajektoria drona', zorder=1000)

        # Waypoints jako punkty
        ax.scatter(lons, lats, alts,
                   c='blue', s=50, marker='o',
                   edgecolors='black', linewidths=1,
                   label='Waypoints', zorder=1001)

        # Punkt startowy (zielony)
        ax.scatter([lons[0]], [lats[0]], [alts[0]],
                   c='green', s=200, marker='o',
                   edgecolors='black', linewidths=2,
                   label='Start', zorder=1002)

        # Punkt końcowy (czerwony)
        ax.scatter([lons[-1]], [lats[-1]], [alts[-1]],
                   c='red', s=200, marker='o',
                   edgecolors='black', linewidths=2,
                   label='Koniec', zorder=1002)

        # Linie pionowe (start i koniec)
        ax.plot([lons[0], lons[0]], [lats[0], lats[0]], [0, alts[0]],
                'g--', linewidth=2, alpha=0.6)
        ax.plot([lons[-1], lons[-1]], [lats[-1], lats[-1]], [0, alts[-1]],
                'r--', linewidth=2, alpha=0.6)

    # 3. Ustawienia osi
    ax.set_xlim(bbox['west'], bbox['east'])
    ax.set_ylim(bbox['south'], bbox['north'])

    if buildings_gdf is not None and len(buildings_gdf) > 0:
        max_building = buildings_gdf['height_m'].max() if 'height_m' in buildings_gdf.columns else 50
    else:
        max_building = 50

    max_alt = max([wp.alt for wp in waypoints]) if waypoints else 50
    ax.set_zlim(0, max(max_building, max_alt) * 1.2)

    ax.set_xlabel('Longitude', fontsize=10)
    ax.set_ylabel('Latitude', fontsize=10)
    ax.set_zlabel('Wysokość [m]', fontsize=10)

    ax.set_title(title, fontsize=14, fontweight='bold', pad=20)
    ax.legend(loc='upper right', fontsize=10)

    # Kąt widzenia
    ax.view_init(elev=20, azim=45)

    plt.tight_layout()
    plt.show()


# ============================================================================
# PRZYKŁAD UŻYCIA
# ============================================================================

if __name__ == "__main__":
    print("Ten moduł jest używany przez 02_visualize_and_select_points.py")
    print("Uruchom: python 02_visualize_and_select_points.py")