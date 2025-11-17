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

# Nazwa miejsca do pobrania
PLACE_NAME = "Warsaw, Poland"


def download_street_network():
    print("\n[1/2] Pobieranie sieci drogowej...")
    G = ox.graph_from_place(PLACE_NAME, network_type='drive')

    graphml_path = DATA_DIR / "warsaw_street_network.graphml"
    pickle_path = DATA_DIR / "warsaw_street_network.pkl"

    ox.save_graphml(G, filepath=graphml_path)
    with open(pickle_path, "wb") as f:
        pickle.dump(G, f)

    return G


def download_buildings():
    """Pobierz tylko budynki, które mają wysokość (height lub building:levels)."""
    print("\n[2/2] Pobieranie budynków z wysokościami...")

    tags = {
        'building': True
    }

    try:
        buildings = ox.features_from_place(PLACE_NAME, tags)
        print(f"      ✓ Pobrano: {len(buildings):,} budynków ogółem")

        # Filtruj tylko budynki z wysokością lub liczbą pięter
        height_cols = ['height', 'building:levels']
        has_height = buildings[height_cols].notna().any(axis=1)
        buildings_with_height = buildings[has_height].copy()

        print(f"      ✓ Budynki z danymi wysokościowymi: {len(buildings_with_height):,}")

        # Zapisz tylko budynki z wysokościami
        pickle_path = DATA_DIR / "warsaw_buildings.pkl"
        geojson_path = DATA_DIR / "warsaw_buildings.geojson"

        buildings_with_height.to_pickle(pickle_path)
        buildings_with_height.to_file(geojson_path, driver='GeoJSON')

        print(f"      ✓ Zapisano tylko budynki z wysokościami: {pickle_path.name} oraz {geojson_path.name}")

        return buildings_with_height

    except Exception as e:
        print(f"      ✗ Błąd podczas pobierania budynków: {e}")
        return None


def print_summary(G, buildings):
    print("\n" + "=" * 70)
    print("PODSUMOWANIE POBRANYCH DANYCH")
    print("=" * 70)

    if G is not None:
        print(f"\n✓ Sieć drogowa:")
        print(f"  - Węzły: {len(G.nodes):,}")
        print(f"  - Krawędzie: {len(G.edges):,}")

    if buildings is not None:
        print(f"\n✓ Budynki (tylko z wysokościami):")
        print(f"  - Łącznie: {len(buildings):,}")
        if 'height' in buildings.columns:
            print(f"  - Z height: {buildings['height'].notna().sum():,}")

    print(f"\n✓ Dane zapisane w katalogu: {DATA_DIR.absolute()}")
    print("=" * 70)


def visualize_map(G, buildings):
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

    ax.set_title("Mapa Warszawy - Budynki z wysokościami", fontsize=14, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.show()


def main():
    Czy_pobrac_dane = input("Czy rozpocząć pobieranie danych dla mapy Warszawy? [t, tak, y, yes] ").strip().lower()

    if Czy_pobrac_dane in ['t', 'tak', 'yes', 'y']:
        print("\n Rozpoczynanie pobierania danych")
        G = download_street_network()
        buildings = download_buildings()
    else:
        with open(DATA_DIR / "warsaw_street_network.pkl", "rb") as f:
            G = pickle.load(f)

        buildings = pd.read_pickle(DATA_DIR / "warsaw_buildings.pkl")

    print_summary(G, buildings)
    visualize_map(G, buildings)


if __name__ == "__main__":
    main()
