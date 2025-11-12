import numpy as np
from pathlib import Path
import pandas as pd

# Import modułów systemu
from config import config
from map_data import main as download_data
from map_vis import WarsawMapSelector
from drone_path_planner import PathPlanner3D
from dron_lqr import dron_lqr
from drone_parameters import input_parameters

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
    print_header("SEKCJA 1: DANE OPENSTREETMAP WARSZAWY")

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

    print("\n⚠ Brak danych lub pobieranie ponowne...")
    print("To może potrwać kilka minut (pobieranie z OpenStreetMap)")

    if not ask_yes_no("Rozpocząć pobieranie?", default=True):
        print("✗ Anulowano. Nie można kontynuować bez danych.")
        return False

    try:
        download_data()
        return True
    except Exception as e:
        print(f"✗ Błąd podczas pobierania danych: {e}")
        return False

def section_2_route_planning():
    """SEKCJA 2: Planowanie trasy drona."""
    print_header("SEKCJA 2: WYBÓR PUNKTÓW A i B NA MAPIE")

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

        selector = WarsawMapSelector()

        if not selector.load_data():
            print("✗ Nie można załadować danych")
            point_a = None
            point_b = None

        selector.create_base_map()
        selector.setup_interactive_mode()

        import matplotlib.pyplot as plt
        plt.show()

        if len(selector.points) == 2:
            point_a = selector.points[0]
            point_b = selector.points[1]
        else:
            print("✗ Nie wybrano punktów")
            point_a = None
            point_b = None

    return point_a, point_b

def section_3_drone_configuration():
    """SEKCJA 3: Wczytanie parametrów drona."""
    print_header("SEKCJA 3: KONFIGURACJA DRONA")
    input_parameters()

def section_4_path_planning(point_a, point_b):
    from map_vis import drone_pathing
    print_header("SEKCJA 4: PLANOWANIE ŚCIEŻKI 3D (A* z omijaniem budynków)")
    print("\n--- Parametry planowania (z konfiguracji drona) ---")
    print(f"  Wysokość startowa: {config.altitude_start}m")
    print(f"  Wysokość przelotu: {config.altitude_cruise_min}-{config.altitude_cruise_max}m")
    print(f"  Wysokość końcowa: {config.altitude_end}m")
    print(f"  Margines bezpieczeństwa: {config.avoid_distance}m")
    print(f"  Rozdzielczość gridu: {config.grid_resolution}m")


    if point_a is not None and point_b is not None:
        drone_pathing(point_a, point_b, headless=False)
    else:
        drone_pathing(None, None, headless=True)

def section_5_simulation():
    """SEKCJA 5: Uruchomienie symulacji."""
    print_header("SEKCJA 5: SYMULACJA LOTU DRONA")

    print("\n✓ Wszystkie parametry ustawione")
    print(f"  - Prędkość: {config.velocity} m/s")
    print(f"  - Wysokość startowa: {config.altitude_start} m")
    print(f"  - Dynamika silników: {'TAK' if config.n == 16 else 'NIE'}")

    if not ask_yes_no("\nCzy rozpocząć symulację?", default=True):
        print("✗ Anulowano symulację")
        return False

    print("\n→ Uruchamianie symulacji...")
    print("  (Wizualizacja będzie aktualizowana w czasie rzeczywistym)\n")

    try:
        dron_lqr()
        print("\n✓ Symulacja zakończona pomyślnie")
        return True
    except Exception as e:
        print(f"\n✗ Błąd podczas symulacji: {e}")
        import traceback
        traceback.print_exc()
        return False


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
    if not ask_yes_no("\nCzy wykonać SEKCJĘ 1 (Dane OpenStreetMap)?", default=True):
        print("→ Pomijam sekcję 1")
    else:
        if not section_1_data_management():
            print("\n✗ Nie można kontynuować bez danych")
            return

    # SEKCJA 2: Wybór punktów
    if not ask_yes_no("\nCzy wykonać SEKCJĘ 2 (Wybór punktów A i B)?", default=True):
        print("→ Pomijam sekcję 2 - używam przykładowych punktów")
        point_a = (52.2297, 21.0122)
        point_b = (52.2330, 21.0180)
    else:
        point_a, point_b = section_2_route_planning()
        if point_a is None or point_b is None:
            print("\n✗ Nie można kontynuować bez punktów")
            return

    # SEKCJA 3: Konfiguracja (NAJPIERW!)
    print("\nKonfiguracja drona")
    section_3_drone_configuration()  # Wywołaj z trybem domyślnym


    # SEKCJA 4: Planowanie ścieżki (POTEM, bo używa config!)
    if not ask_yes_no("\nCzy wykonać SEKCJĘ 4 (Planowanie ścieżki A*)?", default=True):
        print("→ Pomijam sekcję 4 - użyję syntetycznej trajektorii")
        path_result = None
    else:
        section_4_path_planning(point_a, point_b)

if __name__ == "__main__":
    main()