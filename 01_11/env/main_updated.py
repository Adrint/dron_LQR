import osmnx as ox
import pickle
import matplotlib.pyplot as plt

# Wczytaj graf
with open("warszawa_graph.pkl", "rb") as f:
    G = pickle.load(f)

# Przygotuj mapę
fig, ax = ox.plot_graph(G, node_size=0, edge_linewidth=0.5,
                        show=False, close=False)

# Lista na współrzędne i tryb
punkty = []
tryb_zaznaczania = False  # Na start wyłączony


def on_click(event):
    """Funkcja wywoływana przy kliknięciu"""
    global tryb_zaznaczania

    # Sprawdź czy toolbar jest aktywny (zoom/pan)
    if fig.canvas.toolbar.mode != '':
        return  # Ignoruj kliknięcia gdy zoom/pan jest aktywny

    # Sprawdź czy tryb zaznaczania jest włączony
    if not tryb_zaznaczania:
        return

    if event.xdata is not None and event.ydata is not None:
        lon = event.xdata
        lat = event.ydata

        punkty.append((lat, lon))

        kolor = 'green' if len(punkty) == 1 else 'red'
        nazwa = 'A' if len(punkty) == 1 else 'B'
        ax.plot(lon, lat, 'o', color=kolor, markersize=10,
                markeredgecolor='black', markeredgewidth=2)
        ax.text(lon, lat, f'  {nazwa}', fontsize=12, fontweight='bold')

        print(f"✓ Zaznaczono punkt {nazwa}: lat={lat:.6f}, lon={lon:.6f}")

        plt.draw()

        if len(punkty) == 2:
            print("\n✓ Wybrano oba punkty!")
            print(f"Punkt A: {punkty[0]}")
            print(f"Punkt B: {punkty[1]}")
            plt.close()


def on_key(event):
    """Przełącznik trybu na klawisz SPACJA"""
    global tryb_zaznaczania

    if event.key == ' ':  # Spacja
        tryb_zaznaczania = not tryb_zaznaczania

        if tryb_zaznaczania:
            print("\n🟢 TRYB ZAZNACZANIA - klikaj punkty A i B")
            ax.set_title("TRYB ZAZNACZANIA: Wybierz punkty A (zielony) i B (czerwony)",
                         color='green', fontweight='bold')
        else:
            print("\n🔵 TRYB ZOOM - używaj przycisków zoom/pan")
            ax.set_title("TRYB ZOOM: Użyj przycisków zoom/pan, potem SPACJA",
                         color='blue', fontweight='bold')

        plt.draw()


# Podłącz obsługę
fig.canvas.mpl_connect('button_press_event', on_click)
fig.canvas.mpl_connect('key_press_event', on_key)

ax.set_title("TRYB ZOOM: Użyj przycisków zoom/pan, potem wciśnij SPACJA",
             color='blue', fontweight='bold')
print("🔵 TRYB ZOOM - używaj przycisków zoom/pan")
print("   Wciśnij SPACJA aby przełączyć na tryb zaznaczania")

plt.show()

# Po zamknięciu okna
if len(punkty) == 2:
    punkt_A = punkty[0]
    punkt_B = punkty[1]
    print(f"\n✅ Gotowe do użycia:")
    print(f"punkt_A = {punkt_A}")
    print(f"punkt_B = {punkt_B}")

    # --- KROK 2B: Wyznacz obszar analizy ---
    from geopy.distance import geodesic
    import numpy as np

    # Oblicz odległość między A i B
    odleglosc_AB = geodesic(punkt_A, punkt_B).meters
    print(f"\n📏 Odległość A-B: {odleglosc_AB:.2f} metrów")

    # Margines z każdej strony
    margines = 100  # metry

    # Oblicz bounding box z marginesem
    lats = [punkt_A[0], punkt_B[0]]
    lons = [punkt_A[1], punkt_B[1]]

    min_lat = min(lats)
    max_lat = max(lats)
    min_lon = min(lons)
    max_lon = max(lons)

    # Rozszerz o margines (przybliżone: 1° ≈ 111km)
    margines_stopnie = margines / 111000

    bbox_north = max_lat + margines_stopnie
    bbox_south = min_lat - margines_stopnie
    bbox_east = max_lon + margines_stopnie
    bbox_west = min_lon - margines_stopnie

    print(f"\n📦 Obszar analizy (bounding box):")
    print(f"   North: {bbox_north:.6f}")
    print(f"   South: {bbox_south:.6f}")
    print(f"   East: {bbox_east:.6f}")
    print(f"   West: {bbox_west:.6f}")

    # Zwizualizuj obszar
    import matplotlib.patches as patches

    fig2, ax2 = ox.plot_graph(G, node_size=0, edge_linewidth=0.5,
                              show=False, close=False)

    # Narysuj punkty A i B
    ax2.plot(punkt_A[1], punkt_A[0], 'o', color='green', markersize=12,
             markeredgecolor='black', markeredgewidth=2, label='Punkt A')
    ax2.plot(punkt_B[1], punkt_B[0], 'o', color='red', markersize=12,
             markeredgecolor='black', markeredgewidth=2, label='Punkt B')

    # Ograniczenie widoku
    ax2.set_xlim(bbox_west, bbox_east)
    ax2.set_ylim(bbox_south, bbox_north)

    # Narysuj linię między A i B
    ax2.plot([punkt_A[1], punkt_B[1]], [punkt_A[0], punkt_B[0]],
             'b--', linewidth=2, label=f'Trasa ({odleglosc_AB:.0f}m)')

    # Narysuj prostokąt obszaru analizy
    width = bbox_east - bbox_west
    height = bbox_north - bbox_south
    rect = patches.Rectangle((bbox_west, bbox_south), width, height,
                             linewidth=3, edgecolor='orange',
                             facecolor='yellow', alpha=0.2,
                             label=f'Obszar +{margines}m')
    ax2.add_patch(rect)

    ax2.legend()
    ax2.set_title(f"Obszar analizy: {odleglosc_AB:.0f}m + {margines}m margines")

    plt.show()

    print("\n✅ Obszar wyznaczony!")

    # ===== KROK 3A: Stwórz przykładowe budynki do testów =====

    print("\n🏗️ Tworzę przykładowe budynki do testów...")

    import geopandas as gpd
    from shapely.geometry import Polygon, LineString
    import pandas as pd

    # 🆕 PYTAJ O LICZBĘ BUDYNKÓW
    while True:
        try:
            num_buildings = int(input("\n🏢 Ile budynków wygenerować? (np. 50-200): "))
            if num_buildings > 0:
                break
            else:
                print("   ⚠️ Podaj liczbę większą od 0")
        except ValueError:
            print("   ⚠️ Podaj poprawną liczbę!")

    # Oblicz rozmiar siatki na podstawie liczby budynków
    grid_size = int(np.sqrt(num_buildings * 1.7)) + 1

    # Stwórz siatkę budynków w obszarze
    buildings_data = []

    lat_range = np.linspace(bbox_south, bbox_north, grid_size)
    lon_range = np.linspace(bbox_west, bbox_east, grid_size)

    for i, lat in enumerate(lat_range[:-1]):
        for j, lon in enumerate(lon_range[:-1]):
            if len(buildings_data) >= num_buildings:
                break
            if np.random.random() > 0.3:  # 70% szans na budynek
                # Budynek jako mały prostokąt
                # Większe budynki (50m × 30m):
                building_poly = Polygon([
                    (lon, lat),
                    (lon + 0.00045, lat),  # szerokość
                    (lon + 0.00045, lat + 0.00027),  # długość
                    (lon, lat + 0.00027)
                ])

                levels = np.random.randint(2, 12)  # 2-11 pięter
                buildings_data.append({
                    'geometry': building_poly,
                    'building': 'yes',
                    'building:levels': levels,
                    'height': levels * 3  # ~3m na piętro
                })
        if len(buildings_data) >= num_buildings:
            break

    buildings = gpd.GeoDataFrame(buildings_data, crs='EPSG:4326')
    print(f"   ✓ Stworzono {len(buildings)} przykładowych budynków")

    print("\n✅ Dane testowe gotowe!")
    print(f"\n📊 Podsumowanie:")
    print(f"   Budynki: {len(buildings)} obiektów")

    # ===== KROK 3B: Wizualizacja 3D =====

    print("\n🎨 Tworzę wizualizację 3D...")

    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    # Stwórz dużą figurę 3D
    fig = plt.figure(figsize=(18, 14))
    ax = fig.add_subplot(111, projection='3d')

    # 🆕 RYSUJ PŁASZCZYZNĘ Z=0 (SZARA PRZEZROCZYSTA PODŁOGA)
    print("   Rysuję podłoże (z=0)...")
    ground_corners = [
        (bbox_west, bbox_south, 0),
        (bbox_east, bbox_south, 0),
        (bbox_east, bbox_north, 0),
        (bbox_west, bbox_north, 0)
    ]
    ground_poly = Poly3DCollection([ground_corners], alpha=0.3, facecolor='gray', edgecolor='darkgray', linewidth=1)
    ax.add_collection3d(ground_poly)

    # Funkcja pomocnicza do rysowania budynków
    def draw_building_3d(ax, polygon, height, color='gray'):
        """Rysuje budynek jako prostopadłościan z 50% przezroczystością"""
        coords = list(polygon.exterior.coords)

        # Narysuj górę (z=height) - 🆕 ALPHA=0.5 (50% przezroczystość)
        top = [(x, y, height) for x, y in coords]
        top_poly = Poly3DCollection([top], alpha=0.5, facecolor=color, edgecolor='black', linewidth=0.5)
        ax.add_collection3d(top_poly)

        # Narysuj ściany boczne z przezroczystością
        for i in range(len(coords) - 1):
            x1, y1 = coords[i]
            x2, y2 = coords[i + 1]

            # Ściana boczna jako czworokąt
            wall = [
                (x1, y1, 0),
                (x2, y2, 0),
                (x2, y2, height),
                (x1, y1, height)
            ]
            wall_poly = Poly3DCollection([wall], alpha=0.5, facecolor=color, edgecolor='black', linewidth=0.3)
            ax.add_collection3d(wall_poly)

    # Rysuj wszystkie budynki
    print(f"   Rysuję {len(buildings)} budynków...")
    for idx, building in buildings.iterrows():
        height = building['height']
        # Zróżnicuj kolory wg wysokości
        if height > 25:
            color = 'darkred'
        elif height > 15:
            color = 'orangered'
        else:
            color = 'lightgray'

        draw_building_3d(ax, building.geometry, height, color)

   # Zaznacz punkty A i B
    ax.scatter([punkt_A[1]], [punkt_A[0]], [0], color='green', s=300, marker='o',
              edgecolor='black', linewidth=3, label='Punkt A', zorder=1000)
    ax.scatter([punkt_B[1]], [punkt_B[0]], [0], color='red', s=300, marker='o',
              edgecolor='black', linewidth=3, label='Punkt B', zorder=1000)

    # Linia między A i B
    ax.plot([punkt_A[1], punkt_B[1]], [punkt_A[0], punkt_B[0]], [0, 0],
           'b--', linewidth=3, label=f'Trasa ({odleglosc_AB:.0f}m)', alpha=0.8)

    # Ustawienia osi
    ax.set_xlabel('Longitude (°)', fontsize=12)
    ax.set_ylabel('Latitude (°)', fontsize=12)
    ax.set_zlabel('Wysokość (m)', fontsize=12)
    ax.set_xlim(bbox_west, bbox_east)
    ax.set_ylim(bbox_south, bbox_north)
    ax.set_zlim(0, 50)

    # Tytuł i legenda
    ax.set_title(f'Mapa 3D - Obszar {odleglosc_AB:.0f}m + {margines}m margines\n{len(buildings)} budynków',
                fontsize=16, fontweight='bold')
    ax.legend(loc='upper right', fontsize=10)

    # Ustaw widok (kąt kamery)
    ax.view_init(elev=25, azim=45)

    print("\n✅ Wizualizacja 3D gotowa!")
    print("💡 Możesz obracać mapę myszką!")

    plt.tight_layout()
    plt.show()

    # ===== KROK 3C: Planowanie trasy 3D z omijaniem przeszkód BOKIEM =====

    print("\n✈️ Planowanie trasy lotu...")

    # 🆕 INPUT - Wysokości lotu
    while True:
        try:
            h_start = float(input("\n📍 Wysokość startu (punkt A) w metrach (np. 5): "))
            h_cruise = float(input("✈️  Wysokość przelotowa w metrach (np. 15): "))
            h_end = float(input("📍 Wysokość lądowania (punkt B) w metrach (np. 5): "))

            if h_start >= 0 and h_cruise >= 0 and h_end >= 0:
                break
            else:
                print("   ⚠️ Wysokości muszą być >= 0")
        except ValueError:
            print("   ⚠️ Podaj poprawne liczby!")

    print(f"\n📊 Parametry lotu:")
    print(f"   Start: {h_start}m")
    print(f"   Przelot: {h_cruise}m")
    print(f"   Lądowanie: {h_end}m")

    SAFETY_MARGIN = 2  # metry bezpieczeństwa od budynków


    # Funkcja sprawdzająca kolizję punktu z budynkami
    def check_collision_3d(point_lon, point_lat, point_height, buildings, margin=SAFETY_MARGIN):
        """
        Sprawdza czy punkt (lon, lat, height) koliduje z jakimkolwiek budynkiem
        Zwraca: (czy_kolizja, ID_budynku_kolizji, wysokość_budynku)
        """
        from shapely.geometry import Point

        point_2d = Point(point_lon, point_lat)

        for idx, building in buildings.iterrows():
            building_height = building['height']

            # Sprawdź czy punkt jest nad budynkiem (w rzucie 2D)
            if building.geometry.contains(point_2d):
                # Kolizja jeśli punkt jest niżej niż budynek + margines
                if point_height < building_height + margin:
                    return True, idx, building_height

        return False, None, 0


    def find_detour_point(start, end, blocking_building, buildings, cruise_height, margin=SAFETY_MARGIN):
        """
        Znajduje punkt obejścia budynku z boku
        Zwraca waypoint omijający przeszkodę
        """
        from shapely.geometry import LineString, Point
        from shapely import affinity
        import numpy as np

        # Pobierz geometrię blokującego budynku
        building_geom = blocking_building.geometry
        building_height = blocking_building['height']

        # Linia A->B w 2D
        line_2d = LineString([start[:2], end[:2]])

        # Rozszerz budynek o margines bezpieczeństwa w 2D
        expanded_building = building_geom.buffer(margin / 111000)  # margines w stopniach

        # Znajdź punkt środkowy na linii nad budynkiem
        intersection = line_2d.intersection(expanded_building)

        if intersection.is_empty:
            return None

        # Środek kolizji
        if hasattr(intersection, 'centroid'):
            collision_center = intersection.centroid
        else:
            collision_center = Point(intersection.coords[0])

        # Wektor kierunku lotu (normalizowany)
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = np.sqrt(dx ** 2 + dy ** 2)

        if length < 1e-10:
            return None

        dir_x = dx / length
        dir_y = dy / length

        # Wektor prostopadły (w lewo i w prawo)
        perp_left_x = -dir_y
        perp_left_y = dir_x

        perp_right_x = dir_y
        perp_right_y = -dir_x

        # Odległość obejścia (szerokość budynku + margines)
        bounds = expanded_building.bounds  # (minx, miny, maxx, maxy)
        building_width = max(bounds[2] - bounds[0], bounds[3] - bounds[1])
        detour_distance = building_width * 1.5  # 1.5x szerokości budynku

        # Kandydaci na punkt obejścia (lewo i prawo)
        candidates = []

        for side_name, perp_x, perp_y in [("LEFT", perp_left_x, perp_left_y),
                                          ("RIGHT", perp_right_x, perp_right_y)]:
            # Punkt obejścia
            detour_lon = collision_center.x + perp_x * detour_distance
            detour_lat = collision_center.y + perp_y * detour_distance

            # Sprawdź czy punkt obejścia nie koliduje z innym budynkiem
            collision, _, _ = check_collision_3d(detour_lon, detour_lat, cruise_height, buildings, margin)

            if not collision:
                candidates.append((detour_lon, detour_lat, cruise_height, side_name))

        # Wybierz pierwszego kandydata (preferuj lewo jeśli dostępne)
        if candidates:
            return candidates[0]

        # Jeśli boki nie działają, spróbuj podnieść się NAD budynek
        return (collision_center.x, collision_center.y, building_height + margin + 3, "UP")


    def plan_route_with_detours(start, end, buildings, cruise_height, margin=SAFETY_MARGIN, max_iterations=10):
        """
        Planuje trasę z omijaniem przeszkód bokiem
        """
        import numpy as np
        from shapely.geometry import LineString

        waypoints = [start]
        current_pos = start
        target_pos = end
        iteration = 0

        print("\n🔍 Sprawdzam trasę i szukam obejść...")

        while iteration < max_iterations:
            iteration += 1

            # Sprawdź linię prostą od current do target
            num_check_points = 50
            collision_found = False
            first_collision_idx = None
            first_collision_building = None

            for i in range(num_check_points):
                t = i / (num_check_points - 1)
                test_lon = current_pos[0] + t * (target_pos[0] - current_pos[0])
                test_lat = current_pos[1] + t * (target_pos[1] - current_pos[1])
                test_h = current_pos[2] + t * (target_pos[2] - current_pos[2])

                collision, building_idx, building_h = check_collision_3d(test_lon, test_lat, test_h, buildings, margin)

                if collision:
                    collision_found = True
                    first_collision_idx = building_idx
                    first_collision_building = buildings.iloc[building_idx]
                    print(f"   ⚠️  Iter {iteration}: Kolizja z budynkiem {building_idx} (h={building_h:.1f}m)")
                    break

            if not collision_found:
                # Brak kolizji - dodaj cel i zakończ
                if not np.allclose(current_pos, target_pos, atol=1e-6):
                    waypoints.append(target_pos)
                print(f"   ✓ Iter {iteration}: Prosta trasa do celu - OK!")
                break

            # Znaleziono kolizję - znajdź punkt obejścia
            detour = find_detour_point(current_pos, target_pos, first_collision_building,
                                       buildings, cruise_height, margin)

            if detour is None:
                print(f"   ⚠️  Nie można znaleźć obejścia - próbuję przelot NAD")
                # Fallback - przelećmy nad budynkiem
                mid_lon = (current_pos[0] + target_pos[0]) / 2
                mid_lat = (current_pos[1] + target_pos[1]) / 2
                max_building_h = first_collision_building['height']
                detour = (mid_lon, mid_lat, max_building_h + margin + 3, "UP")

            # Wyciągnij side_name dla informacji
            side_name = detour[3] if len(detour) > 3 else "?"
            # Zostaw tylko współrzędne (lon, lat, h)
            detour_coords = (detour[0], detour[1], detour[2])

            side_name = detour[3] if len(detour) > 3 else "?"
            detour_coords = (detour[0], detour[1], detour[2])
            waypoints.append(detour_coords)
            current_pos = detour_coords
            print(f"   ✓ Dodano waypoint obejścia {side_name}: h={detour_coords[2]:.1f}m")

        if iteration >= max_iterations:
            print(f"   ⚠️  Osiągnięto limit iteracji ({max_iterations})")

        return waypoints


    # Wyznacz trasę z omijaniem przeszkód
    segment1_start = (punkt_A[1], punkt_A[0], h_start)  # (lon, lat, h)
    segment1_end = (punkt_A[1], punkt_A[0], h_cruise)  # wznoszenie pionowe

    segment3_start = (punkt_B[1], punkt_B[0], h_cruise)  # zniżanie startuje z cruise
    segment3_end = (punkt_B[1], punkt_B[0], h_end)  # opadanie pionowe

    # Planuj trasę poziomą z omijaniem
    horizontal_waypoints = plan_route_with_detours(
        segment1_end,
        segment3_start,
        buildings,
        h_cruise,
        SAFETY_MARGIN
    )

    # Połącz wszystkie segmenty
    all_waypoints = [segment1_start] + horizontal_waypoints + [segment3_end]

    collision_detected = len(horizontal_waypoints) > 2  # więcej niż start i end = były obejścia

    if not collision_detected:
        print("\n✅ Trasa bezpieczna - brak kolizji!")
    else:
        print(f"\n✅ Trasa z {len(all_waypoints)} waypointami - omija wszystkie przeszkody!")

    print(f"\n📍 Punkty trasy (waypoints): {len(all_waypoints)}")
    for i, wp in enumerate(all_waypoints):
        print(f"   {i + 1}. lon={wp[0]:.6f}, lat={wp[1]:.6f}, h={wp[2]:.1f}m")

    # ===== KROK 3D: Wizualizacja 3D TRASY LOTU =====

    print("\n🎨 Tworzę wizualizację trasy lotu 3D...")

# Stwórz nową figurę 3D dla trasy
fig_route = plt.figure(figsize=(20, 16))
ax_route = fig_route.add_subplot(111, projection='3d')

# PODŁOŻE
print("   Rysuję podłoże...")
ground_corners_route = [
    (bbox_west, bbox_south, 0),
    (bbox_east, bbox_south, 0),
    (bbox_east, bbox_north, 0),
    (bbox_west, bbox_north, 0)
]
ground_poly_route = Poly3DCollection([ground_corners_route], alpha=0.2, facecolor='lightgray',
                                     edgecolor='gray', linewidth=1)
ax_route.add_collection3d(ground_poly_route)


# BUDYNKI (bardziej przezroczyste niż poprzednio)
def draw_building_simple(ax, polygon, height, color='gray', alpha=0.3):
    """Rysuje uproszczony budynek dla lepszej widoczności trasy"""
    coords = list(polygon.exterior.coords)

    # Tylko górny dach
    top = [(x, y, height) for x, y in coords]
    top_poly = Poly3DCollection([top], alpha=alpha, facecolor=color,
                                edgecolor='black', linewidth=0.3)
    ax.add_collection3d(top_poly)

    # Ściany (bardzo przezroczyste)
    for i in range(len(coords) - 1):
        x1, y1 = coords[i]
        x2, y2 = coords[i + 1]
        wall = [(x1, y1, 0), (x2, y2, 0), (x2, y2, height), (x1, y1, height)]
        wall_poly = Poly3DCollection([wall], alpha=alpha * 0.5, facecolor=color,
                                     edgecolor='black', linewidth=0.2)
        ax.add_collection3d(wall_poly)


print("   Rysuję budynki...")
for idx, building in buildings.iterrows():
    height = building['height']
    if height > 25:
        color = 'darkred'
    elif height > 15:
        color = 'orangered'
    else:
        color = 'lightgray'

    draw_building_simple(ax_route, building.geometry, height, color, alpha=0.3)

# STREFA BEZPIECZEŃSTWA wokół budynków (czerwone obrysy)
print("   Rysuję strefy bezpieczeństwa...")
for idx, building in buildings.iterrows():
    height = building['height'] + SAFETY_MARGIN
    # Rysuj tylko obwód na wysokości budynek + margines
    coords = list(building.geometry.exterior.coords)
    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]
    zs = [height] * len(coords)
    ax_route.plot(xs, ys, zs, 'r-', linewidth=1, alpha=0.3)

# TRASA LOTU - główna atrakcja!
print("   Rysuję trasę lotu...")

# Wyciągnij współrzędne waypoints
route_lons = [wp[0] for wp in all_waypoints]
route_lats = [wp[1] for wp in all_waypoints]
route_heights = [wp[2] for wp in all_waypoints]

# NARYSUJ TRASĘ jako grubą linię
ax_route.plot(route_lons, route_lats, route_heights,
              color='lime', linewidth=5, marker='o', markersize=8,
              label='Trasa lotu', zorder=1000, alpha=0.9)

# Zaznacz segmenty różnymi kolorami
if len(all_waypoints) >= 4:
    # Wznoszenie (zielony)
    ax_route.plot(route_lons[0:2], route_lats[0:2], route_heights[0:2],
                  color='green', linewidth=6, linestyle='--',
                  label='Wznoszenie', alpha=0.8, zorder=999)

    # Przelot (niebieski)
    ax_route.plot(route_lons[1:3], route_lats[1:3], route_heights[1:3],
                  color='cyan', linewidth=6,
                  label='Przelot', alpha=0.8, zorder=999)

    # Opadanie (pomarańczowy)
    ax_route.plot(route_lons[2:4], route_lats[2:4], route_heights[2:4],
                  color='orange', linewidth=6, linestyle='--',
                  label='Opadanie', alpha=0.8, zorder=999)

# WAYPOINTS jako duże markery
for i, (lon, lat, h) in enumerate(all_waypoints):
    if i == 0:
        # Start (zielony)
        ax_route.scatter([lon], [lat], [h], color='green', s=500,
                         marker='^', edgecolor='black', linewidth=3,
                         label='START (A)', zorder=1001)
        # Pionowa linia od ziemi
        ax_route.plot([lon, lon], [lat, lat], [0, h],
                      'g--', linewidth=2, alpha=0.5)
    elif i == len(all_waypoints) - 1:
        # Koniec (czerwony)
        ax_route.scatter([lon], [lat], [h], color='red', s=500,
                         marker='v', edgecolor='black', linewidth=3,
                         label='KONIEC (B)', zorder=1001)
        # Pionowa linia do ziemi
        ax_route.plot([lon, lon], [lat, lat], [0, h],
                      'r--', linewidth=2, alpha=0.5)
    else:
        # Waypoint pośredni (żółty)
        ax_route.scatter([lon], [lat], [h], color='yellow', s=300,
                         marker='o', edgecolor='black', linewidth=2,
                         label=f'Waypoint {i}' if i == 1 else '', zorder=1001)

# Zaznacz punkty A i B na ziemi
ax_route.scatter([punkt_A[1]], [punkt_A[0]], [0],
                 color='darkgreen', s=200, marker='s',
                 edgecolor='black', linewidth=2, alpha=0.7, zorder=500)
ax_route.scatter([punkt_B[1]], [punkt_B[0]], [0],
                 color='darkred', s=200, marker='s',
                 edgecolor='black', linewidth=2, alpha=0.7, zorder=500)

# Tekst z informacjami
ax_route.text2D(0.02, 0.98,
                f'Długość trasy: {odleglosc_AB:.1f}m\n'
                f'Wysokość startowa: {h_start}m\n'
                f'Wysokość przelotowa: {route_heights[1]:.1f}m\n'
                f'Wysokość lądowania: {h_end}m\n'
                f'Margines bezpieczeństwa: {SAFETY_MARGIN}m',
                transform=ax_route.transAxes,
                fontsize=11, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# Ustawienia osi
ax_route.set_xlabel('Longitude (°)', fontsize=12, labelpad=10)
ax_route.set_ylabel('Latitude (°)', fontsize=12, labelpad=10)
ax_route.set_zlabel('Wysokość (m)', fontsize=12, labelpad=10)
ax_route.set_xlim(bbox_west, bbox_east)
ax_route.set_ylim(bbox_south, bbox_north)
ax_route.set_zlim(0, max(50, max(route_heights) + 10))

# Tytuł
collision_status = "⚠️ Z KOREKTĄ" if collision_detected else "✓ BEZ KOLIZJI"
ax_route.set_title(
    f'Trasa lotu 3D {collision_status}\n'
    f'{len(all_waypoints)} punktów trasy | {len(buildings)} budynków',
    fontsize=18, fontweight='bold', pad=20)

# Legenda (bez duplikatów)
handles, labels = ax_route.get_legend_handles_labels()
by_label = dict(zip(labels, handles))
ax_route.legend(by_label.values(), by_label.keys(),
                loc='upper right', fontsize=10, framealpha=0.9)

# Ustaw lepszy kąt widzenia
ax_route.view_init(elev=20, azim=45)

# Siatka
ax_route.grid(True, alpha=0.3)

print("\n✅ Wizualizacja trasy gotowa!")
print("💡 Możesz obracać mapę myszką aby zobaczyć trasę z różnych kątów!")

plt.tight_layout()
plt.show()

