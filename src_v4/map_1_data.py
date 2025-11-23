import osmnx as ox
import pickle
import geopandas as gpd
from pathlib import Path
import warnings
import matplotlib.pyplot as plt
import pandas as pd

warnings.filterwarnings('ignore')

# Konfiguracja OSMnx
ox.settings.use_cache = True
ox.settings.log_console = True

# Katalog na dane
DATA_DIR = Path("data")
DATA_DIR.mkdir(exist_ok=True)


def select_city():
    """Wybór miasta do pobrania danych."""
    while True:
        print("WYBÓR MIASTA")
        print("1. Warszawa - duże miasto (Warsaw, Poland)")
        print("2. Pionki - małe miasto (Pionki, Poland)")
        print("3. Wpisz własne miasto")
        print("=" * 70)

        choice = input("\nWybierz opcję [1/2/3]: ").strip()

        if choice == "1":
            place_name = "Warsaw, Poland"
            city_id = "warsaw"
            print(f"Wybrano: {place_name}")
            return place_name, city_id

        elif choice == "2":
            place_name = "Pionki, Poland"
            city_id = "pionki"
            print(f"\Wybrano: {place_name}")
            return place_name, city_id

        elif choice == "3":
            print("Podaj nazwę miasta i kraju po angielsku")
            print("Przykłady: Warsaw, Kozienice, Gdansk")

            city_name = input("Nazwa miasta (po angielsku): ").strip()
            country_name = input("Nazwa kraju (po angielsku) [Poland]: ").strip()

            if not city_name:
                print("Błąd: Nazwa miasta nie może być pusta!")
                continue

            if not country_name:
                country_name = "Poland"

            place_name = f"{city_name}, {country_name}"
            city_id = city_name.lower().replace(" ", "_")

            print(f"\nWybrano: {place_name}")
            confirm = input(f"Czy dane są poprawne? [t]: ").strip().lower()

            if confirm in ['t', 'tak', 'y', 'yes', '']:
                return place_name, city_id
            else:
                print("Anulowano wybór, spróbuj ponownie.")
                continue
        else:
            print("Nieprawidłowa odpowiedź! Wybierz 1, 2 lub 3.")


def download_street_network(place_name, city_id):
    print("\n[1/2] Pobieranie sieci drogowej...")
    try:
        G = ox.graph_from_place(place_name, network_type='drive')

        graphml_path = DATA_DIR / f"{city_id}_street_network.graphml"
        pickle_path = DATA_DIR / f"{city_id}_street_network.pkl"

        ox.save_graphml(G, filepath=graphml_path)
        with open(pickle_path, "wb") as f:
            pickle.dump(G, f)

        print(f"Zapisano sieć drogową: {pickle_path.name}")
        return G

    except Exception as e:
        print(f"Błąd podczas pobierania sieci drogowej: {e}")
        print(f"Sprawdź poprawność nazwy miasta i kraju.")
        return None


def download_buildings(place_name, city_id):
    """Pobranie tylko budynków, które mają wysokość."""
    print("\n[2/2] Pobieranie budynków z wysokościami...")

    tags = {
        'building': True
    }

    try:
        buildings = ox.features_from_place(place_name, tags)
        print(f"Pobrano: {len(buildings):,} budynków ogółem")

        # Filtruj tylko budynki z wysokością lub liczbą pięter
        height_cols = ['height', 'building:levels']
        has_height = buildings[height_cols].notna().any(axis=1)
        buildings_with_height = buildings[has_height].copy()

        print(f"Budynki z danymi wysokościowymi: {len(buildings_with_height):,}")

        # Zapisz tylko budynki z wysokościami
        pickle_path = DATA_DIR / f"{city_id}_buildings.pkl"
        geojson_path = DATA_DIR / f"{city_id}_buildings.geojson"

        buildings_with_height.to_pickle(pickle_path)
        buildings_with_height.to_file(geojson_path, driver='GeoJSON')

        print(f"Zapisano tylko budynki z wysokościami: {pickle_path.name} oraz {geojson_path.name}")

        return buildings_with_height

    except Exception as e:
        print(f"Błąd podczas pobierania budynków: {e}")
        print(f"Sprawdź poprawność nazwy miasta i kraju.")
        return None

def print_summary(G, buildings):
    print("\n" + "=" * 70)
    print("PODSUMOWANIE POBRANYCH DANYCH")
    print("=" * 70)

    if G is not None:
        print(f"\n Sieć drogowa:")
        print(f"  - Węzły: {len(G.nodes):,}")
        print(f"  - Krawędzie: {len(G.edges):,}")

    if buildings is not None:
        print(f"\n Budynki (tylko z wysokościami):")
        print(f"  - Łącznie: {len(buildings):,}")
        if 'height' in buildings.columns:
            print(f"  - Z height: {buildings['height'].notna().sum():,}")

    print(f"\n Dane zapisane w katalogu: {DATA_DIR.absolute()}")
    print("=" * 70)


def visualize_map(G, buildings, place_name):
    print("\nWIZUALIZACJA POBRANYCH DANYCH\n")

    fig, ax = ox.plot_graph(
        G,
        node_size=0,
        edge_linewidth=0.5,
        edge_color='#999999',
        bgcolor='white',
        show=False,
        close=False
    )

    if buildings is not None and len(buildings) > 0:
        buildings.plot(
            ax=ax,
            facecolor='orange',
            alpha=0.5,
            edgecolor='darkgray',
            linewidth=0.3
        )

    ax.set_title(f"Mapa {place_name} - Budynki z wysokościami", fontsize=14, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.show()


def main():
    """Główna funkcja programu pobierania danych mapy."""

    # Wybór miasta
    place_name, city_id = select_city()

    street_network_path = DATA_DIR / f"{city_id}_street_network.pkl"
    buildings_path = DATA_DIR / f"{city_id}_buildings.pkl"

    # Sprawdzenie czy dane istnieją
    if street_network_path.exists() and buildings_path.exists():
        print(f"\n Znaleziono zapisane dane dla: {place_name}")
        print(f"  - Sieć drogowa: {street_network_path}")
        print(f"  - Budynki: {buildings_path}")

        czy_pobrac_dane = input("\nCzy pobrać dane ponownie? [n]: ").strip().lower()

        if czy_pobrac_dane not in ['t', 'tak', 'yes', 'y']:
            try:
                with open(street_network_path, "rb") as f:
                    G = pickle.load(f)
                buildings = pd.read_pickle(buildings_path)
                print("Wczytano dane z pliku")
            except Exception as e:
                print(f"Błąd podczas wczytywania danych: {e}")
                return
        else:
            # Pobieranie nowych danych z obsługą błędów
            while True:
                print(f"\n Rozpoczynanie pobierania danych dla: {place_name}")
                G = download_street_network(place_name, city_id)
                buildings = download_buildings(place_name, city_id)

                if G is None or buildings is None:
                    print("\n" + "!" * 70)
                    print("BŁĄD POBIERANIA DANYCH")
                    print("!" * 70)
                    print("Możliwe przyczyny:")
                    print("  - Nieprawidłowa nazwa miasta lub kraju")
                    print("  - Miasto nie istnieje w bazie OpenStreetMap")
                    print("  - Brak połączenia z internetem")
                    print("!" * 70)

                    retry = input("\nCzy chcesz wybrać inne miasto? [t/n]: ").strip().lower()
                    if retry in ['t', 'tak', 'y', 'yes', '']:
                        place_name, city_id = select_city()
                        continue
                    else:
                        print("Zakończono program.")
                        return
                else:
                    break
    else:
        # Brak zapisanych danych - pobierz nowe
        while True:
            print(f"\n Rozpoczynanie pobierania danych dla: {place_name}")
            G = download_street_network(place_name, city_id)
            buildings = download_buildings(place_name, city_id)

            if G is None or buildings is None:
                print("\n" + "!" * 70)
                print("BŁĄD POBIERANIA DANYCH")
                print("!" * 70)
                print("Możliwe przyczyny:")
                print("  - Nieprawidłowa nazwa miasta lub kraju")
                print("  - Miasto nie istnieje w bazie OpenStreetMap")
                print("  - Brak połączenia z internetem")
                print("!" * 70)

                retry = input("\nCzy chcesz wybrać inne miasto? [t/n]: ").strip().lower()
                if retry in ['t', 'tak', 'y', 'yes', '']:
                    place_name, city_id = select_city()
                    continue
                else:
                    print("Zakończono program.")
                    return
            else:
                break

    print_summary(G, buildings)
    visualize_map(G, buildings, place_name)


if __name__ == "__main__":
    main()