"""
Test skrypt dla path_planner_3d.py
===================================
Prosty test 3D A* bez mapy OSM - używa sztucznych przeszkód.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from drone_path_planner import PathPlanner3D, Node
from config import config
import geopandas as gpd
from shapely.geometry import box


def create_test_obstacles():
    """Stwórz testowe przeszkody (sztuczne budynki)."""

    # Symuluj kilka budynków jako prostokąty
    buildings = []

    # Budynek 1: (10, 10) - (30, 30), wysokość 20m
    buildings.append({
        'geometry': box(21.00, 52.22, 21.0002, 52.2202),  # lon/lat
        'height': 20.0,
        'building': 'yes'
    })

    # Budynek 2: (50, 20) - (70, 40), wysokość 15m
    buildings.append({
        'geometry': box(21.0005, 52.2202, 21.0007, 52.2204),
        'height': 15.0,
        'building': 'yes'
    })

    # Budynek 3: (30, 50) - (50, 70), wysokość 25m
    buildings.append({
        'geometry': box(21.0003, 52.2205, 21.0005, 52.2207),
        'height': 25.0,
        'building': 'yes'
    })

    # Konwertuj na GeoDataFrame
    gdf = gpd.GeoDataFrame(buildings, crs='EPSG:4326')

    return gdf


def test_path_planner():
    """Testuj path planner z sztucznymi przeszkodami."""

    print("\n" + "=" * 70)
    print("TEST 3D A* PATH PLANNER")
    print("=" * 70)

    # Ustaw parametry w config
    config.altitude_start = 5.0
    config.altitude_cruise_min = 10.0
    config.altitude_cruise_max = 30.0
    config.altitude_end = 5.0
    config.avoid_distance = 3.0
    config.grid_resolution = 5.0

    print("\n[Config]")
    print(f"  Start altitude: {config.altitude_start}m")
    print(f"  Cruise altitude: {config.altitude_cruise_min}-{config.altitude_cruise_max}m")
    print(f"  End altitude: {config.altitude_end}m")
    print(f"  Grid resolution: {config.grid_resolution}m")
    print(f"  Avoid distance: {config.avoid_distance}m")

    # Stwórz testowe przeszkody
    print("\n[Obstacles]")
    buildings = create_test_obstacles()
    print(f"  Created {len(buildings)} test buildings")

    # Punkty start i end (lat, lon)
    # Prosta linia z (0, 0) do (100, 100) w metrach
    # W lat/lon: ~0.0009 stopnia ≈ 100m
    lat_A = 52.220
    lon_A = 21.000
    lat_B = 52.2209  # ~100m na północ
    lon_B = 21.0009  # ~100m na wschód

    print("\n[Route]")
    print(f"  Point A: ({lat_A:.6f}, {lon_A:.6f})")
    print(f"  Point B: ({lat_B:.6f}, {lon_B:.6f})")
    print(f"  Distance: ~141m (diagonal)")

    # Stwórz planner
    planner = PathPlanner3D(buildings, config)

    # Zaplanuj ścieżkę
    result = planner.plan_path(lat_A, lon_A, lat_B, lon_B)

    if result is None:
        print("\n✗ FAILED: No path found!")
        return None

    print(f"\n✓ SUCCESS!")
    print(f"  Path length: {result['distance']:.1f}m")
    print(f"  Waypoints: {len(result['path_latlon'])}")

    # Wizualizuj
    visualize_test_result(result, buildings)

    return result


def visualize_test_result(result, buildings):
    """Wizualizuj wynik testu."""

    path_meters = result['path_meters']
    ref_point = result['ref_point']

    fig = plt.figure(figsize=(15, 5))

    # Panel 1: Widok 3D
    ax1 = fig.add_subplot(131, projection='3d')

    # Ścieżka
    ax1.plot(path_meters[:, 0], path_meters[:, 1], path_meters[:, 2],
             'b-', linewidth=2, label='Drone path')

    # Start i koniec
    ax1.scatter(path_meters[0, 0], path_meters[0, 1], path_meters[0, 2],
                c='green', s=200, marker='o', label='Start', zorder=100)
    ax1.scatter(path_meters[-1, 0], path_meters[-1, 1], path_meters[-1, 2],
                c='red', s=200, marker='o', label='End', zorder=100)

    # Budynki (przybliżone)
    for _, building in buildings.iterrows():
        bounds = building.geometry.bounds  # (minx, miny, maxx, maxy)

        # Konwertuj na metry
        from drone_path_planner import PathPlanner3D
        planner = PathPlanner3D(buildings, config)

        corners = [
            planner.latlon_to_meters(bounds[1], bounds[0], 0, ref_point[0], ref_point[1]),
            planner.latlon_to_meters(bounds[3], bounds[2], 0, ref_point[0], ref_point[1])
        ]

        x_min = min(corners[0][0], corners[1][0])
        x_max = max(corners[0][0], corners[1][0])
        y_min = min(corners[0][1], corners[1][1])
        y_max = max(corners[0][1], corners[1][1])
        height = building.get('height', 10.0)

        # Rysuj prostopadłościan
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection

        vertices = [
            [x_min, y_min, 0],
            [x_max, y_min, 0],
            [x_max, y_max, 0],
            [x_min, y_max, 0],
            [x_min, y_min, height],
            [x_max, y_min, height],
            [x_max, y_max, height],
            [x_min, y_max, height]
        ]

        faces = [
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[1], vertices[2], vertices[6], vertices[5]],
            [vertices[2], vertices[3], vertices[7], vertices[6]],
            [vertices[3], vertices[0], vertices[4], vertices[7]],
            [vertices[4], vertices[5], vertices[6], vertices[7]]
        ]

        poly = Poly3DCollection(faces, alpha=0.3, facecolor='orange', edgecolor='black')
        ax1.add_collection3d(poly)

    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Path (with obstacles)', fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Panel 2: Widok z góry (XY)
    ax2 = fig.add_subplot(132)
    ax2.plot(path_meters[:, 0], path_meters[:, 1], 'b-', linewidth=2)
    ax2.scatter(path_meters[0, 0], path_meters[0, 1], c='green', s=100, marker='o', zorder=100)
    ax2.scatter(path_meters[-1, 0], path_meters[-1, 1], c='red', s=100, marker='o', zorder=100)

    # Budynki
    for _, building in buildings.iterrows():
        bounds = building.geometry.bounds
        planner = PathPlanner3D(buildings, config)
        corners = [
            planner.latlon_to_meters(bounds[1], bounds[0], 0, ref_point[0], ref_point[1]),
            planner.latlon_to_meters(bounds[3], bounds[2], 0, ref_point[0], ref_point[1])
        ]
        x_min = min(corners[0][0], corners[1][0])
        x_max = max(corners[0][0], corners[1][0])
        y_min = min(corners[0][1], corners[1][1])
        y_max = max(corners[0][1], corners[1][1])

        ax2.add_patch(plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                                    facecolor='orange', alpha=0.5, edgecolor='black'))

    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('Top view (XY)', fontweight='bold')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    # Panel 3: Profil wysokości
    ax3 = fig.add_subplot(133)
    distance = np.cumsum([0] + [np.linalg.norm(path_meters[i + 1] - path_meters[i])
                                for i in range(len(path_meters) - 1)])
    ax3.plot(distance, path_meters[:, 2], 'b-', linewidth=2)
    ax3.scatter(distance[0], path_meters[0, 2], c='green', s=100, marker='o', zorder=100)
    ax3.scatter(distance[-1], path_meters[-1, 2], c='red', s=100, marker='o', zorder=100)
    ax3.set_xlabel('Distance [m]')
    ax3.set_ylabel('Altitude [m]')
    ax3.set_title('Altitude profile', fontweight='bold')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    try:
        result = test_path_planner()

        if result:
            print("\n✅ Test completed successfully!")
        else:
            print("\n✗ Test failed")

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback

        traceback.print_exc()