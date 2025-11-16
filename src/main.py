import matplotlib.pyplot as plt
from pathlib import Path

# Import modułów systemu
from config_parameters import input_parameters
from config import config
import pickle
import pandas as pd
from map_1_data import download_street_network, download_buildings, DATA_DIR
from map_2_select_points import MapABSelector
from map_3_path_planner import DroneRoutePlanner
from control_dron_lqr import dron_lqr

def print_header(title):
    """Wyświetl nagłówek sekcji."""
    print("\n" + "=" * 80)
    print(title.center(80))
    print("=" * 80)

def section_1_data_management():
    """SEKCJA 1: Zarządzanie danymi OpenStreetMap."""

    from map_1_data import download_street_network, download_buildings, DATA_DIR

    street_network_path = DATA_DIR / "warsaw_street_network.pkl"
    buildings_path = DATA_DIR / "warsaw_buildings.pkl"

    # Sprawdź czy dane istnieją
    if street_network_path.exists() and buildings_path.exists():
        print("\n✓ Znaleziono zapisane dane Warszawy")
        print(f"  - Sieć drogowa: {street_network_path}")
        print(f"  - Budynki: {buildings_path}")

        while True:
            user_input = input("Czy chcesz pobrać dane ponownie? [n]: ").strip().lower()
            if user_input in ['t', 'tak', 'y', 'yes']:
                break  # pobieraj dane – wychodzimy z pętli
            elif user_input in ['n', 'nie', 'no', '']:
                try:
                    with open(street_network_path, "rb") as f:
                        G = pickle.load(f)
                    buildings = pd.read_pickle(buildings_path)
                    print("✓ Wczytano dane z pliku")
                    return G, buildings
                except Exception as e:
                    print(f"✗ Błąd podczas wczytywania danych: {e}")
                    return None, None
            else:
                print("✗ Nieprawidłowa odpowiedź. Wpisz 't' (tak) lub 'n' (nie).")

    print("\n⚠ Rozpoczynanie pobierania danych z OSM...")

    try:
        G = download_street_network()
        buildings = download_buildings()

        if G is None or buildings is None:
            raise RuntimeError("Dane nie zostały pobrane prawidłowo.")

        return G, buildings

    except Exception as e:
        print(f"✗ Błąd podczas pobierania danych: {e}")
        return None, None


def section_2_route_planning(G, buildings):
    while True:
        try:
            print("\nOpcje:")
            print("1. Interaktywny wybór na mapie (wizualnie)")
            print("2. Ręczne wprowadzenie współrzędnych")
            print("3. Użyj przykładowych punktów (szybki test)")

            choice = input("\nWybór [1]: ").strip()

            if choice == "" or choice == "1":
                print("\n→ Otwieranie interaktywnej mapy...")
                print("  1. Użyj zoom/pan aby znaleźć obszar")
                print("  2. Wciśnij SPACJĘ aby włączyć tryb zaznaczania")
                print("  3. Kliknij punkt A (zielony), potem B (czerwony)")
                print("  4. Zamknij wykres po wybraniu punktów aby kontynuować")

                try:
                    selector = MapABSelector(G, buildings)
                    point_a, point_b = selector.run()  # TERAZ run() już zwraca parę punktów

                    if point_a is None or point_b is None:
                        print("✗ Nie wybrano punktów lub wystąpił błąd")
                        return None, None

                    return point_a, point_b

                except Exception as e:
                    print(f"✗ Błąd interaktywnego wyboru punktów: {e}")
                    return None, None

            elif choice == "2":
                # Ręczne wprowadzenie
                try:
                    print("\nPodaj współrzędne punktów:")
                    lat_a = float(input("Punkt A - szerokość geograficzna (lat): "))
                    lon_a = float(input("Punkt A - długość geograficzna (lon): "))
                    lat_b = float(input("Punkt B - szerokość geograficzna (lat): "))
                    lon_b = float(input("Punkt B - długość geograficzna (lon): "))

                    point_a = (lat_a, lon_a)
                    point_b = (lat_b, lon_b)
                    print(f"  A: {point_a}")
                    print(f"  B: {point_b}")
                    return point_a, point_b

                except ValueError:
                    print("✗ Błąd: współrzędne muszą być liczbami zmiennoprzecinkowymi.")
                except Exception as e:
                    print(f"✗ Nieoczekiwany błąd: {e}")

            elif choice == "3":
                # Przykładowe punkty
                print("\n✓ Używam przykładowych punktów:")
                point_a = (52.22481371485093, 20.98913003830705)  # Plac Zamkowy
                point_b = (52.217125576827954, 20.997711209573286)  # Katedra
                print(f"  A: {point_a}")
                print(f"  B: {point_b}")
                return point_a, point_b

            else:
                print("✗ Nieprawidłowy wybór. Wybierz 1, 2, 3 lub naciśnij Enter.")

        except Exception as e:
            print(f"✗ Wystąpił błąd podczas wyboru opcji: {e}")

def section_3_drone_configuration():
    """SEKCJA 3: Wczytanie parametrów drona."""
    input_parameters()

def section_4_path_planning(point_a, point_b, buildings):

    print("\n--- Parametry planowania (z konfiguracji drona) ---")
    print(f"  Wysokość startowa: {config.altitude_start}m")
    print(f"  Wysokość przelotu: {config.altitude_cruise_min}-{config.altitude_cruise_max}m")
    print(f"  Wysokość końcowa: {config.altitude_end}m")
    print(f"  Margines bezpieczeństwa: {config.avoid_distance}m")
    print(f"  Rozdzielczość gridu: {config.grid_resolution}m")

    planner = DroneRoutePlanner(config)
    planner.buildings_gdf = buildings
    result = planner.plan(point_a, point_b, show_plots=True)
    return result


def main():
    """Główna funkcja programu."""
    print_header("SYSTEM SYMULACJI DRONA - INTEGRACJA MODUŁÓW")

    print("  1. Pobieranie/ładowanie danych Warszawy")
    print("  2. Wybór punktów A i B na mapie")
    print("  3. Konfiguracja parametrów drona")
    print("  4. Planowanie ścieżki 3D z omijaniem budynków")
    print("  5. Symulacja lotu z wizualizacją")

    print(" Wciśnij 'Enter' aby użyć domyślnej konfiguracji ukazanej w nawiasie kwadratowym []")


    print_header("SEKCJA 1: DANE OPENSTREETMAP WARSZAWY")
    G, buildings = section_1_data_management()
    if G is None or buildings is None:
        print("✗ Nie udało się załadować danych. Zakończono.")
        return

    print_header("SEKCJA 2: WYBÓR PUNKTÓW A i B NA MAPIE")
    point_a, point_b = section_2_route_planning(G, buildings)

    print_header("SEKCJA 3: KONFIGURACJA DRONA")
    section_3_drone_configuration()

    print_header("SEKCJA 4: WYZNACZENIE OPTYMALNEJ TRASY")
    path_result = section_4_path_planning(point_a, point_b, buildings)

    print_header("SEKCJA 5: STEROWANIE DRONA Z LQR")
    dron_lqr(path_result)

if __name__ == "__main__":
    main()