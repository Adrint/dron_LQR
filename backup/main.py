import matplotlib.pyplot as plt
from pathlib import Path

# Import modułów systemu
from config_parameters import input_parameters
from config import config
import pickle
import pandas as pd
from map_1_data import download_street_network, download_buildings, DATA_DIR, select_city
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

    # Wybór miasta
    place_name, city_id = select_city()

    street_network_path = DATA_DIR / f"{city_id}_street_network.pkl"
    buildings_path = DATA_DIR / f"{city_id}_buildings.pkl"

    # Sprawdzenie czy dane istnieją
    if street_network_path.exists() and buildings_path.exists():
        print(f"\n Znaleziono zapisane dane dla: {place_name}")
        print(f"- Sieć drogowa: {street_network_path}")
        print(f"- Budynki: {buildings_path}")

        while True:
            user_input = input("Czy chcesz pobrać dane ponownie? [n]: ").strip().lower()

            if user_input in ['t', 'tak', 'y', 'yes']:
                break  # przejście do pobierania nowych danych
            elif user_input in ['n', 'nie', 'no', '']:
                try:
                    with open(street_network_path, "rb") as f:
                        G = pickle.load(f)
                    buildings = pd.read_pickle(buildings_path)
                    print("Wczytano dane z pliku")
                    return G, buildings, city_id
                except Exception as e:
                    print(f" Błąd podczas wczytywania danych: {e}")
                    return None, None, None
            else:
                print("Nieprawidłowa odpowiedź. Wpisz 't' (tak) lub 'n' (nie).")

    # Pobieranie danych z obsługą błędów
    while True:
        czy_pobrac = input(f"Czy chcesz pobrać dane dla miasta: {place_name}? [t/n]: ").strip().lower()

        if czy_pobrac in ['t', 'tak', 'y', 'yes']:
            try:
                print(f"\n Rozpoczynanie pobierania danych dla: {place_name}")
                G = download_street_network(place_name, city_id)
                buildings = download_buildings(place_name, city_id)

                if G is None or buildings is None:
                    print("\n" + "!" * 70)
                    print("BŁĄD POBIERANIA DANYCH")
                    print("!" * 70)
                    print("Możliwe przyczyny:")
                    print(f"- Nieprawidłowa nazwa miasta lub kraju")
                    print(f"- Miasto nie istnieje w bazie OpenStreetMap")
                    print(f"- Brak połączenia z internetem")
                    print("!" * 70)

                    retry = input("\nCzy chcesz wybrać inne miasto? [t/n]: ").strip().lower()
                    if retry in ['t', 'tak', 'y', 'yes', '']:
                        place_name, city_id = select_city()
                        continue
                    else:
                        print("Anulowano pobieranie danych.")
                        return None, None, None

                return G, buildings, city_id

            except Exception as e:
                print(f" Błąd podczas pobierania danych: {e}")
                retry = input("\nCzy chcesz wybrać inne miasto? [t/n]: ").strip().lower()
                if retry in ['t', 'tak', 'y', 'yes', '']:
                    place_name, city_id = select_city()
                    continue
                else:
                    return None, None, None

        elif czy_pobrac in ['n', 'nie', 'no', '']:
            print("Anulowano pobieranie danych.")
            return None, None, None

        else:
            print("Nieprawidłowa odpowiedź. Wpisz 't' (tak) lub 'n' (nie).")


def section_2_route_planning(G, buildings, city_id):
    while True:
        try:
            print("\nOpcje:")
            print("1. Interaktywny wybór na mapie (wizualnie)")
            print("2. Ręczne wprowadzenie współrzędnych")
            if city_id == "warsaw":
                print("3. Użyj przykładowych punktów (szybki test) - tylko Warszawa")
            else:
                print("3. Użyj przykładowych punktów (niedostępne dla tego miasta - tylko dla Warszawy)")

            choice = input("\nWybór [1]: ").strip()


            # OPCJA 1 – interaktywna mapa
            if choice == "" or choice == "1":
                print("\n Otwieranie interaktywnej mapy...")
                print(f"1. Użyj zoom/pan aby znaleźć obszar")
                print(f"2. Wciśnij SPACJĘ aby włączyć tryb zaznaczania")
                print(f"3. Kliknij punkt A (zielony), potem B (czerwony)")
                print(f"4. Zamknij wykres po wybraniu punktów aby kontynuować")
                print(f"\nOtwieranie mapy\n")

                try:
                    selector = MapABSelector(G, buildings, city_id)
                    point_a, point_b = selector.run()

                    if point_a is None or point_b is None:
                        print("Nie wybrano punktów lub wystąpił błąd")
                        return None, None

                    return point_a, point_b

                except Exception as e:
                    print(f"Błąd interaktywnego wyboru punktów: {e}")
                    return None, None

            # OPCJA 2 – ręczne współrzędne
            elif choice == "2":
                try:
                    print("\nPodaj współrzędne punktów:")
                    lat_a = float(input("Punkt A - szerokość geograficzna (lat): "))
                    lon_a = float(input("Punkt A - długość geograficzna (lon): "))
                    lat_b = float(input("Punkt B - szerokość geograficzna (lat): "))
                    lon_b = float(input("Punkt B - długość geograficzna (lon): "))

                    point_a = (lat_a, lon_a)
                    point_b = (lat_b, lon_b)
                    print(f"A: {point_a}")
                    print(f"B: {point_b}")

                    try:
                        selector = MapABSelector(G, buildings, city_id)  # lub MapABSelector(G, buildings)
                        selector.points = [point_a, point_b]  # ustawiamy punkty ręcznie
                        selector.show_yx_zx()                 # rysuje wykres + zapisuje JSONy
                    except Exception as e:
                        print(f"Błąd rysowania profilu trasy: {e}")

                    return point_a, point_b

                except ValueError:
                    print("Błąd: współrzędne muszą być liczbami zmiennoprzecinkowymi.")
                except Exception as e:
                    print(f"Nieoczekiwany błąd: {e}")

            # OPCJA 3 – przykładowe punkty
            elif choice == "3":
                if city_id == "warsaw":
                    print("\n Używam przykładowych punktów:")
                    point_a = (52.22481371485093, 20.98913003830705)      # Plac Zamkowy
                    point_b = (52.217125576827954, 20.997711209573286)    # Katedra
                    print(f"A: {point_a}")
                    print(f"B: {point_b}")

                    try:
                        selector = MapABSelector(G, buildings, city_id)  # lub MapABSelector(G, buildings)
                        selector.points = [point_a, point_b]
                        selector.show_yx_zx()
                    except Exception as e:
                        print(f"Błąd rysowania profilu trasy: {e}")

                    return point_a, point_b
                else:
                    print("Przykładowe punkty są dostępne tylko dla Warszawy!")
                    print("Wybierz opcję 1 (mapa) lub 2 (ręczne współrzędne)")
                    continue

            else:
                print(
                    "Nieprawidłowy wybór. Wybierz 1, 2"
                    + (" lub 3" if city_id == "warsaw" else "")
                    + " lub naciśnij Enter."
                )

        except Exception as e:
            print(f"Wystąpił błąd podczas wyboru opcji: {e}")



def section_3_drone_configuration():
    """SEKCJA 3: Wczytanie parametrów drona."""
    input_parameters()


def section_4_path_planning(point_a, point_b, buildings, city_id):
    print("\n--- Parametry planowania (z konfiguracji drona) ---")
    print(f"Wysokość startowa: {config.altitude_start}m")
    print(f"Wysokość przelotu: {config.altitude_cruise_min}-{config.altitude_cruise_max}m")
    print(f"Wysokość końcowa: {config.altitude_end}m")
    print(f"Margines bezpieczeństwa: {config.avoid_distance}m")
    print(f"Rozdzielczość gridu: {config.grid_resolution}m")

    planner = DroneRoutePlanner(config, city_id)
    planner.buildings_gdf = buildings
    result = planner.plan(point_a, point_b, show_plots=True)
    return result


def main():
    """Główna funkcja programu."""
    print_header("SYSTEM SYMULACJI DRONA - INTEGRACJA MODUŁÓW")

    print(f"1. Wybór i pobieranie/ładowanie danych miasta")
    print(f"2. Wybór punktów A i B na mapie")
    print(f"3. Konfiguracja parametrów drona")
    print(f"4. Planowanie ścieżki 3D z omijaniem budynków")
    print(f"5. Symulacja lotu z wizualizacją")

    print("Wciśnij 'Enter' aby użyć domyślnej konfiguracji ukazanej w nawiasie kwadratowym []")

    print_header("SEKCJA 1: WYBÓR MIASTA I DANE OPENSTREETMAP")
    G, buildings, city_id = section_1_data_management()
    if G is None or buildings is None:
        print("Nie udało się załadować danych. Zakończono.")
        return

    print_header("SEKCJA 2: WYBÓR PUNKTÓW A i B NA MAPIE")
    point_a, point_b = section_2_route_planning(G, buildings, city_id)

    print_header("SEKCJA 3: KONFIGURACJA DRONA")
    section_3_drone_configuration()

    print_header("SEKCJA 4: WYZNACZENIE OPTYMALNEJ TRASY")
    path_result = section_4_path_planning(point_a, point_b, buildings, city_id)

    print_header("SEKCJA 5: STEROWANIE DRONA Z LQR")
    dron_lqr(path_result)


if __name__ == "__main__":
    main()