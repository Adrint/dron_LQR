"""
Pobieranie i zapisywanie danych Warszawy z OpenStreetMap
=========================================================
Autor: Warsaw Drone Simulation Project
Inspiracja: https://github.com/gboeing/osmnx-examples

Ten skrypt pobiera:
- Sieć drogową Warszawy
- Budynki z wysokościami
- Drzewa z wysokościami
- Mosty

Dane są zapisywane do plików dla późniejszego użycia.
"""

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

print("=" * 70)
print("POBIERANIE DANYCH WARSZAWY Z OPENSTREETMAP")
print("=" * 70)


def download_street_network():
    """Pobierz sieć drogową Warszawy."""
    print("\n[1/4] Pobieranie sieci drogowej...")
    print(f"      Miejsce: {PLACE_NAME}")
    print(f"      Typ sieci: drive (drogi dostępne dla pojazdów)")

    G = ox.graph_from_place(PLACE_NAME, network_type='drive')

    print(f"      ✓ Pobrano: {len(G.nodes):,} węzłów, {len(G.edges):,} krawędzi")

    # Zapisz jako GraphML i Pickle
    graphml_path = DATA_DIR / "warsaw_street_network.graphml"
    pickle_path = DATA_DIR / "warsaw_street_network.pkl"

    ox.save_graphml(G, filepath=graphml_path)
    with open(pickle_path, "wb") as f:
        pickle.dump(G, f)

    print(f"      ✓ Zapisano: {graphml_path.name} oraz {pickle_path.name}")

    return G


def download_buildings():
    """Pobierz budynki z wysokościami."""
    print("\n[2/4] Pobieranie budynków z wysokościami...")

    # Tags dla budynków z informacją o wysokości
    tags = {
        'building': True
    }

    try:
        buildings = ox.features_from_place(PLACE_NAME, tags)
        print(f"      ✓ Pobrano: {len(buildings):,} budynków")

        # Sprawdź ile ma informacje o wysokości
        height_cols = ['height', 'building:levels']
        available_heights = 0

        for col in height_cols:
            if col in buildings.columns:
                available_heights += buildings[col].notna().sum()

        print(f"      ℹ Budynki z danymi wysokościowymi: ~{available_heights:,}")

        # Zapisz
        pickle_path = DATA_DIR / "warsaw_buildings.pkl"
        buildings.to_pickle(pickle_path)

        # Opcjonalnie: GeoJSON dla łatwego podglądu
        geojson_path = DATA_DIR / "warsaw_buildings.geojson"
        buildings.to_file(geojson_path, driver='GeoJSON')

        print(f"      ✓ Zapisano: {pickle_path.name} oraz {geojson_path.name}")

        return buildings

    except Exception as e:
        print(f"      ✗ Błąd podczas pobierania budynków: {e}")
        return None


def download_trees():
    """Pobierz drzewa z wysokościami."""
    print("\n[3/4] Pobieranie drzew...")

    # Tags dla drzew
    tags = {
        'natural': 'tree'
    }

    try:
        trees = ox.features_from_place(PLACE_NAME, tags)
        print(f"      ✓ Pobrano: {len(trees):,} drzew")

        # Sprawdź ile ma informacje o wysokości
        if 'height' in trees.columns:
            trees_with_height = trees['height'].notna().sum()
            print(f"      ℹ Drzewa z danymi o wysokości: {trees_with_height:,}")

        # Zapisz
        pickle_path = DATA_DIR / "warsaw_trees.pkl"
        trees.to_pickle(pickle_path)

        print(f"      ✓ Zapisano: {pickle_path.name}")

        return trees

    except Exception as e:
        print(f"      ✗ Błąd podczas pobierania drzew: {e}")
        return None


def download_bridges():
    """Pobierz mosty."""
    print("\n[4/4] Pobieranie mostów...")

    # Tags dla mostów
    tags = {
        'bridge': True
    }

    try:
        bridges = ox.features_from_place(PLACE_NAME, tags)
        print(f"      ✓ Pobrano: {len(bridges):,} mostów/wiaduktów")

        # Zapisz
        pickle_path = DATA_DIR / "warsaw_bridges.pkl"
        bridges.to_pickle(pickle_path)

        print(f"      ✓ Zapisano: {pickle_path.name}")

        return bridges

    except Exception as e:
        print(f"      ✗ Błąd podczas pobierania mostów: {e}")
        return None


def print_summary(G, buildings, trees, bridges):
    """Wyświetl podsumowanie pobranych danych."""
    print("\n" + "=" * 70)
    print("PODSUMOWANIE POBRANYCH DANYCH")
    print("=" * 70)

    if G is not None:
        print(f"\n✓ Sieć drogowa:")
        print(f"  - Węzły (skrzyżowania): {len(G.nodes):,}")
        print(f"  - Krawędzie (drogi): {len(G.edges):,}")

    if buildings is not None:
        print(f"\n✓ Budynki:")
        print(f"  - Łącznie: {len(buildings):,}")
        if 'height' in buildings.columns:
            print(f"  - Z wysokością: {buildings['height'].notna().sum():,}")
        if 'building:levels' in buildings.columns:
            print(f"  - Z liczbą pięter: {buildings['building:levels'].notna().sum():,}")

    if trees is not None:
        print(f"\n✓ Drzewa:")
        print(f"  - Łącznie: {len(trees):,}")
        if 'height' in trees.columns:
            print(f"  - Z wysokością: {trees['height'].notna().sum():,}")

    if bridges is not None:
        print(f"\n✓ Mosty/wiadukty:")
        print(f"  - Łącznie: {len(bridges):,}")

    print(f"\n✓ Wszystkie dane zapisane w katalogu: {DATA_DIR.absolute()}")
    print("\n" + "=" * 70)
    print("GOTOWE! Uruchom teraz: 02_visualize_and_select_points.py")
    print("=" * 70)


def visualize_map(G, buildings, trees, bridges):
    """Wyświetl wizualizację pobranych danych."""
    print("\n" + "=" * 70)
    print("WIZUALIZACJA POBRANYCH DANYCH")
    print("=" * 70)
    print("\nTworzenie mapy...")

    # Podstawowa mapa z siecią drogową
    fig, ax = ox.plot_graph(
        G,
        node_size=0,
        edge_linewidth=0.5,
        edge_color='#999999',
        bgcolor='white',
        show=False,
        close=False
    )

    # Dodaj budynki
    if buildings is not None:
        print("  - Dodawanie budynków...")
        try:
            buildings.plot(
                ax=ax,
                facecolor='orange',
                alpha=0.5,
                edgecolor='darkgray',
                linewidth=0.3
            )
        except Exception as e:
            print(f"    ⚠ Nie udało się wyrenderować budynków: {e}")

    # Dodaj drzewa
    if trees is not None:
        print("  - Dodawanie drzew...")
        try:
            trees.plot(
                ax=ax,
                color='green',
                markersize=1,
                alpha=0.3
            )
        except Exception as e:
            print(f"    ⚠ Nie udało się wyrenderować drzew: {e}")

    # Dodaj mosty
    if bridges is not None:
        print("  - Dodawanie mostów...")
        try:
            bridges.plot(
                ax=ax,
                color='blue',
                linewidth=2,
                alpha=0.5
            )
        except Exception as e:
            print(f"    ⚠ Nie udało się wyrenderować mostów: {e}")

    ax.set_title("Mapa Warszawy - Pobrane dane z OpenStreetMap",
                 fontsize=14, fontweight='bold', pad=20)

    print("\n✓ Mapa gotowa! Zamknij okno aby kontynuować...")
    plt.tight_layout()
    plt.show()


def main():
    """Główna funkcja - pobierz wszystkie dane."""

    # Pobierz dane
    Czy_pobrac_dane = input("Czy_pobrac_dane: ").strip().lower()
    if Czy_pobrac_dane in ['t', 'tak', 'yes', 'Y']:
        G = download_street_network()
        buildings = download_buildings()
        trees = download_trees()
        bridges = download_bridges()
        print("Zakończono pobieranie danych")
    else:
        print("Ładowanie danych z plików...")

        # Załaduj graf
        with open(DATA_DIR / "warsaw_street_network.pkl", "rb") as f:
            G = pickle.load(f)
        print(f"  ✓ Graf: {len(G.nodes):,} węzłów")

        buildings = pd.read_pickle(DATA_DIR / "warsaw_buildings.pkl")
        print(f"  ✓ Budynki: {len(buildings):,}")

        # Załaduj drzewa
        trees = pd.read_pickle(DATA_DIR / "warsaw_trees.pkl")
        print(f"  ✓ Drzewa: {len(trees):,}")

        # Załaduj mosty
        bridges = pd.read_pickle(DATA_DIR / "warsaw_bridges.pkl")
        print(f"  ✓ Mosty: {len(bridges):,}")

        print("Dane załadowane!")

    # Wyświetl podsumowanie
    print_summary(G, buildings, trees, bridges)
    visualize_map(G, buildings, trees, bridges)

if __name__ == "__main__":
    main()