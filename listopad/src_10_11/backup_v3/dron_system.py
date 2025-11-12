import matplotlib.pyplot as plt
from pathlib import Path

# Import modułów systemu
from drone_parameters import input_parameters
from config import config
from map_1_data import main as download_data
from map_2_select_points import MapABSelector
from map_3_path_planner import DroneRoutePlanner

def print_header(title):
    """Wyświetl nagłówek sekcji."""
    print("\n" + "=" * 80)
    print(title.center(80))
    print("=" * 80)

def ask_yes_no(question, default=True):
    """Zapytaj użytkownika tak/nie."""
    default_str = "[T/n]" if default else "[t/N]"
    answer = input(f"{question} {default_str}: ").strip().lower()

    if not answer:
        return default

    return answer in ['t', 'tak', 'y', 'yes']

def section_1_data_management():
    """SEKCJA 1: Zarządzanie danymi OpenStreetMap."""

    data_dir = Path("data")
    street_network_path = data_dir / "warsaw_street_network.pkl"
    buildings_path = data_dir / "warsaw_buildings.pkl"

    # Sprawdź czy dane istnieją
    if street_network_path.exists() and buildings_path.exists():
        print("\n✓ Znaleziono zapisane dane Warszawy")
        print(f"  - Sieć drogowa: {street_network_path}")
        print(f"  - Budynki: {buildings_path}")

        if not ask_yes_no("Czy chcesz pobrać dane ponownie?", default=False):
            print("✓ Używam istniejących danych")
            return True

    print("\n⚠ Brak danych")
    try:
        download_data()
        return True
    except Exception as e:
        print(f"✗ Błąd podczas pobierania danych: {e}")
        return False

def section_2_route_planning():

    print("\nOpcje:")
    print("1. Interaktywny wybór na mapie (wizualnie)")
    print("2. Ręczne wprowadzenie współrzędnych")
    print("3. Użyj przykładowych punktów (szybki test)")

    choice = input("\nWybór [1]: ").strip()

    if choice == "2":
        # Ręczne wprowadzenie
        print("\nPodaj współrzędne punktów:")
        lat_a = float(input("Punkt A - szerokość geograficzna (lat): "))
        lon_a = float(input("Punkt A - długość geograficzna (lon): "))
        lat_b = float(input("Punkt B - szerokość geograficzna (lat): "))
        lon_b = float(input("Punkt B - długość geograficzna (lon): "))

        point_a = (lat_a, lon_a)
        point_b = (lat_b, lon_b)
        print(f"  A: {point_a}")
        print(f"  B: {point_b}")

    elif choice == "3":
        # Przykładowe punkty w centrum Warszawy
        print("\n✓ Używam przykładowych punktów:")
        point_a = (52.2297, 21.0122)  # Plac Zamkowy
        point_b = (52.2330, 21.0180)  # Katedra
        print(f"  A: {point_a}")
        print(f"  B: {point_b}")

    else:
        # Interaktywny wybór (domyślnie)
        print("\n→ Otwieranie interaktywnej mapy...")
        print("  1. Użyj zoom/pan aby znaleźć obszar")
        print("  2. Wciśnij SPACJĘ aby włączyć tryb zaznaczania")
        print("  3. Kliknij punkt A (zielony), potem B (czerwony)")

        selector = MapABSelector()

        result = selector.run()  # Zwraca {"A": a, "B": b, ...} lub None

        if result is None or "A" not in result or "B" not in result:
            print("✗ Nie wybrano punktów lub wystąpił błąd")

            return None, None

        point_a = result["A"]  # (lat, lon)
        point_b = result["B"]  # (lat, lon)

        return point_a, point_b

def section_3_drone_configuration():
    """SEKCJA 3: Wczytanie parametrów drona."""
    input_parameters()

def section_4_path_planning(point_a, point_b):

    print_header("SEKCJA 4: PLANOWANIE ŚCIEŻKI 3D (A* z omijaniem budynków)")
    print("\n--- Parametry planowania (z konfiguracji drona) ---")
    print(f"  Wysokość startowa: {config.altitude_start}m")
    print(f"  Wysokość przelotu: {config.altitude_cruise_min}-{config.altitude_cruise_max}m")
    print(f"  Wysokość końcowa: {config.altitude_end}m")
    print(f"  Margines bezpieczeństwa: {config.avoid_distance}m")
    print(f"  Rozdzielczość gridu: {config.grid_resolution}m")

    planner = DroneRoutePlanner(config)
    result = planner.plan(point_a, point_b, show_plots=True)
    return result


def main():
    """Główna funkcja programu."""
    print_header("SYSTEM SYMULACJI DRONA - INTEGRACJA MODUŁÓW")

    print("\nTen program przeprowadzi Cię przez wszystkie etapy:")
    print("  1. Pobieranie/ładowanie danych Warszawy")
    print("  2. Wybór punktów A i B na mapie")
    print("  3. Konfiguracja parametrów drona")
    print("  4. Planowanie ścieżki 3D z omijaniem budynków")
    print("  5. Symulacja lotu z wizualizacją")


    # SEKCJA 1: Dane
    print_header("SEKCJA 1: DANE OPENSTREETMAP WARSZAWY")
    section_1_data_management()

    # SEKCJA 2: Wybór punktów
    print_header("SEKCJA 2: WYBÓR PUNKTÓW A i B NA MAPIE")
    point_a, point_b = section_2_route_planning()

    # SEKCJA 3: Konfiguracja
    print_header("SEKCJA 3: KONFIGURACJA DRONA")
    section_3_drone_configuration()  # Wywołaj z trybem domyśln

    # SEKCJA 4: Planowanie ścieżki
    print_header("SEKCJA 4: WYZNACZENIE OPTYMALNEJ TRASY")
    section_4_path_planning(point_a, point_b)

if __name__ == "__main__":
    main()