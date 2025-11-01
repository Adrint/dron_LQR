import osmnx as ox
import pickle

# --- URUCHOM RAZ - pobierz i zapisz ---
print("Pobieram dane Warszawy...")
G = ox.graph_from_place("Warsaw, Poland", network_type='drive')
print(f"Pobrano! Węzłów: {len(G.nodes)}, Dróg: {len(G.edges)}")

# Zapisz do pliku (możesz użyć jednej z metod):

# Metoda 1: GraphML (uniwersalny format)
ox.save_graphml(G, filepath="warszawa_graph.graphml")

# LUB Metoda 2: Pickle (szybszy, mniejszy plik)
with open("warszawa_graph.pkl", "wb") as f:
    pickle.dump(G, f)

print("Zapisano!")