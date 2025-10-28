# System Planowania Trajektorii Drona

System do automatycznego wyznaczania optymalnej trajektorii lotu drona z punktu A do punktu B z ominięciem przeszkód. Kompatybilny z projektem `dron_lqr_with_orientation.py`.

## 📋 Spis Treści

- [Funkcjonalności](#funkcjonalności)
- [Wymagania](#wymagania)
- [Instalacja](#instalacja)
- [Szybki Start](#szybki-start)
- [Struktura Projektu](#struktura-projektu)
- [Użycie](#użycie)
- [Algorytmy](#algorytmy)
- [Integracja z Dronem LQR](#integracja-z-dronem-lqr)
- [Przykłady](#przykłady)
- [API](#api)

## ✨ Funkcjonalności

- **Algorytmy planowania**:
  - A* (A-star) - szybki, optymalny dla siatki
  - RRT (Rapidly-exploring Random Tree) - dobry dla złożonych przestrzeni
  - RRT* - optymalny wariant RRT

- **Typy przeszkód**:
  - Prostopadłościany (budynki)
  - Cylindry (drzewa, wieże)
  - Kule (balony, kopuły)
  - Wielokąty (nieregularne budynki)

- **Eksport trajektorii**:
  - Format NumPy (.npz) - do bezpośredniego wczytania
  - Format JSON - czytelny dla człowieka
  - Moduł Python - bezpośrednia zamiana trajectory.py

- **Wizualizacja**:
  - Wizualizacja 3D
  - Projekcje 2D (XY, XZ, YZ)
  - Statystyki trajektorii

## 📦 Wymagania

```
Python >= 3.7
numpy
matplotlib
```

## 🚀 Instalacja

1. Skopiuj folder `path_planning` do projektu:
```bash
D:\Praca_Magisterska_PW\
├── src_28_10\           # Twój obecny projekt
└── path_planning\       # Nowy folder
```

2. Zainstaluj zależności (jeśli jeszcze nie masz):
```bash
pip install numpy matplotlib
```

## ⚡ Szybki Start

### Sposób 1: Interaktywny (polecany dla początkujących)

```bash
cd path_planning
python main.py
```

Program przeprowadzi Cię przez:
1. Wybór punktu startowego (A)
2. Wybór punktu docelowego (B)
3. Dodawanie przeszkód (gotowe scenariusze lub własne)
4. Wybór algorytmu
5. Generowanie i eksport trajektorii

### Sposób 2: Programatyczny (dla zaawansowanych)

```python
from obstacles import ObstacleManager
from path_planner import create_planner
from trajectory_export import TrajectoryExporter

# Setup
obstacles = ObstacleManager()
obstacles.add_box((25, 0, -7.5), (10, 10, 15), "Budynek")

# Plan
planner = create_planner('astar', obstacles, safety_distance=1.5)
path = planner.plan(start=(0, 0, -2), goal=(50, 0, -2))

# Export
exporter = TrajectoryExporter(velocity=2.0, dt=0.01)
exporter.export_numpy(path, 'trajectory.npz')
```

## 📁 Struktura Projektu

```
path_planning/
├── main.py                    # Program główny (interaktywny)
├── config.py                  # Konfiguracja
├── obstacles.py               # Definicje przeszkód
├── path_planner.py            # Algorytmy planowania (A*, RRT, RRT*)
├── trajectory_export.py       # Eksport do różnych formatów
├── visualization.py           # Wizualizacja 2D/3D
│
├── data/
│   ├── generated/             # Wygenerowane trajektorie
│   ├── obstacles/             # Zapisane konfiguracje przeszkód
│   └── maps/                  # Mapy (opcjonalnie)
│
├── examples/
│   └── example_usage.py       # Przykłady użycia
│
└── README.md
```

## 📖 Użycie

### System Współrzędnych NED

System używa konwencji NED (North-East-Down) zgodnie z `dron_lqr_with_orientation.py`:

- **X**: Kierunek do przodu (North)
- **Y**: Kierunek w prawo (East)
- **Z**: Kierunek w dół (Down)
  - Z = 0: Poziom gruntu
  - Z < 0: Wysokość nad ziemią (np. Z = -5 oznacza 5m AGL)

**Uwaga**: W interfejsie użytkownika podajesz wysokość jako wartość pozytywną (np. "5m"), a system automatycznie konwertuje do NED (Z = -5).

### Dodawanie Przeszkód

#### Prostopadłościan (Budynek)

```python
obstacles.add_box(
    position=(x, y, z_center),  # Pozycja środka
    size=(width, depth, height),  # Wymiary
    name="Budynek_1"
)
```

Przykład:
```python
# Budynek 10x10x15m na pozycji X=25, Y=0, wysokość środka 7.5m
obstacles.add_box((25, 0, -7.5), (10, 10, 15), "Budynek")
```

#### Cylinder (Drzewo, Wieża)

```python
obstacles.add_cylinder(
    position=(x, y, z_base),  # Pozycja podstawy (na ziemi: z_base=0)
    radius=r,
    height=h,
    name="Drzewo_1"
)
```

Przykład:
```python
# Drzewo o promieniu 2m i wysokości 12m
obstacles.add_cylinder((15, 5, 0), radius=2.0, height=12.0, name="Drzewo")
```

#### Kula

```python
obstacles.add_sphere(
    position=(x, y, z_center),
    radius=r,
    name="Balon"
)
```

### Wybór Algorytmu

#### A* (domyślny)
- **Zalety**: Szybki, zawsze znajduje optymalną ścieżkę w siatce
- **Wady**: Może być wolny dla bardzo dużych przestrzeni
- **Użycie**: Środowiska o umiarkowanej złożoności

```python
planner = create_planner('astar', obstacles, 
                        grid_resolution=0.5,
                        safety_distance=1.5)
```

#### RRT
- **Zalety**: Szybki, dobry dla złożonych przestrzeni
- **Wady**: Ścieżki nieoptymalne, losowy charakter
- **Użycie**: Złożone środowiska 3D, wiele przeszkód

```python
planner = create_planner('rrt', obstacles,
                        safety_distance=1.5,
                        max_iterations=5000,
                        step_size=1.0)
```

#### RRT*
- **Zalety**: Optymalne ścieżki, dobre dla złożonych przestrzeni
- **Wady**: Wolniejszy niż RRT
- **Użycie**: Gdy potrzebujesz optymalnej ścieżki w złożonym środowisku

```python
planner = create_planner('rrt_star', obstacles,
                        safety_distance=1.5,
                        max_iterations=5000,
                        step_size=1.0,
                        search_radius=3.0)
```

## 🔗 Integracja z Dronem LQR

### Metoda 1: Zamień trajectory.py (najłatwiejsza)

1. Wygeneruj trajektorię w `path_planning`:
```bash
python main.py
```

2. Plik `custom_trajectory.py` zostanie utworzony w `data/generated/`

3. Zastąp oryginalny plik:
```bash
# Windows
copy data\generated\custom_trajectory.py D:\Praca_Magisterska_PW\src_28_10\trajectory.py

# Linux/Mac
cp data/generated/custom_trajectory.py ../src_28_10/trajectory.py
```

4. Uruchom symulację drona:
```bash
cd D:\Praca_Magisterska_PW\src_28_10
python dron_lqr_with_orientation.py
```

### Metoda 2: Wczytaj z pliku .npz

Zmodyfikuj `dron_lqr_with_orientation.py`:

```python
# PRZED (linia ~131):
# X_ref_all, Y_terr_all, ... = generate_reference_profile(Vel, dt, 50)

# PO:
import numpy as np
trajectory_data = np.load('../path_planning/data/generated/trajectory_output.npz')
X_ref_all = trajectory_data['X_ref']
Y_ref_all = trajectory_data['Y_ref']
Z_ref_all = trajectory_data['Z_ref']
Y_terr_all = trajectory_data['Y_terr']
Z_terr_all = trajectory_data['Z_terr']
alpha_all = trajectory_data['alpha']
beta_all = trajectory_data['beta']
```

### Metoda 3: Import jako moduł

```python
# W dron_lqr_with_orientation.py
import sys
sys.path.insert(0, '../path_planning/data/generated')
from custom_trajectory import generate_reference_profile

# Reszta kodu pozostaje bez zmian
X_ref_all, Y_terr_all, ... = generate_reference_profile(Vel, dt, 50)
```

## 💡 Przykłady

### Przykład 1: Pojedyncza przeszkoda

```python
from obstacles import ObstacleManager
from path_planner import create_planner
from visualization import PathVisualizer

obstacles = ObstacleManager()
obstacles.add_box((25, 0, -7.5), (10, 10, 15), "Budynek")

planner = create_planner('astar', obstacles, safety_distance=1.5)
path = planner.plan((0, 0, -2), (50, 0, -2))

visualizer = PathVisualizer(obstacles)
visualizer.plot_3d(path, start, goal)
visualizer.show()
```

### Przykład 2: Las

```python
obstacles = ObstacleManager()

# Dodaj wiele drzew
for i in range(10):
    x = np.random.uniform(10, 40)
    y = np.random.uniform(-10, 10)
    obstacles.add_cylinder((x, y, 0), radius=2.0, height=12.0)

planner = create_planner('rrt', obstacles, 
                        max_iterations=3000, step_size=1.5)
path = planner.plan((0, 0, -3), (50, 0, -3))
```

### Przykład 3: Miasto

```python
obstacles = ObstacleManager()

# Dodaj budynki różnych wysokości
buildings = [
    ((15, -5, -7.5), (8, 6, 15)),
    ((30, 5, -15), (12, 10, 30)),
    ((45, 0, -10), (10, 10, 20)),
]

for pos, size in buildings:
    obstacles.add_box(pos, size)

planner = create_planner('rrt_star', obstacles,
                        max_iterations=5000, search_radius=5.0)
path = planner.plan((0, 0, -5), (55, 0, -5))
```

Więcej przykładów w `examples/example_usage.py`:
```bash
python examples/example_usage.py
```

## 🛠️ API

### ObstacleManager

```python
class ObstacleManager:
    def add_box(position, size, name="")
    def add_cylinder(position, radius, height, name="")
    def add_sphere(position, radius, name="")
    def is_collision_free(point, safety_distance=0.0) -> bool
    def is_path_collision_free(point1, point2, num_checks=10) -> bool
```

### PathPlanner

```python
# Tworzenie plannera
planner = create_planner(
    algorithm='astar',  # 'astar', 'rrt', 'rrt_star'
    obstacle_manager=obstacles,
    grid_resolution=0.5,  # Tylko dla A*
    safety_distance=1.5,
    max_iterations=5000,  # Dla RRT/RRT*
    step_size=1.0,        # Dla RRT/RRT*
    search_radius=3.0     # Dla RRT*
)

# Planowanie
path = planner.plan(start=(x1,y1,z1), goal=(x2,y2,z2))
# Zwraca: List[np.ndarray] lub None
```

### TrajectoryExporter

```python
exporter = TrajectoryExporter(velocity=2.0, dt=0.01)

# Export do NumPy
data = exporter.export_numpy(path, 'output.npz', smooth=True)

# Export do JSON
exporter.export_json(path, 'output.json')

# Generuj moduł Python
exporter.generate_trajectory_function(path, 'custom_trajectory.py')
```

### PathVisualizer

```python
visualizer = PathVisualizer(obstacle_manager)

# Wizualizacja 3D
visualizer.plot_3d(path, start, goal, title="Trajektoria")

# Projekcje 2D
visualizer.plot_2d_projections(path, start, goal)

visualizer.show()
```

## ⚙️ Konfiguracja

Edytuj `config.py` aby zmienić domyślne parametry:

```python
# Parametry drona
DRONE_VELOCITY = 2.0        # m/s
SAFETY_DISTANCE = 1.5       # m
DRONE_SIZE = 0.5            # m

# Planowanie ścieżki
GRID_RESOLUTION = 0.5       # m (dla A*)
DEFAULT_ALGORITHM = 'astar'

# RRT parametry
RRT_MAX_ITERATIONS = 5000
RRT_STEP_SIZE = 1.0         # m

# Export
EXPORT_TIME_STEP = 0.01     # s (musi zgadzać się z dt w kontrolerze)
```

## 📊 Formaty Wyjściowe

### NumPy (.npz)
```python
data = np.load('trajectory.npz')
X_ref = data['X_ref']    # Pozycje X
Y_ref = data['Y_ref']    # Pozycje Y
Z_ref = data['Z_ref']    # Pozycje Z (NED)
alpha = data['alpha']    # Kąty pitch (radiany)
beta = data['beta']      # Kąty roll (radiany)
```

### JSON
```json
{
  "waypoints": [
    {"x": 0.0, "y": 0.0, "z": -2.0},
    {"x": 1.5, "y": 0.5, "z": -2.1},
    ...
  ],
  "velocity": 2.0,
  "dt": 0.01,
  "num_waypoints": 150
}
```

### Python Module
```python
from custom_trajectory import generate_reference_profile

X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta = generate_reference_profile(
    Vel=2.0, 
    dt=0.01, 
    X_max=50
)
```

## 🐛 Rozwiązywanie Problemów

### "Nie znaleziono ścieżki"

1. **Zwiększ liczbę iteracji** (dla RRT/RRT*):
   ```python
   planner = create_planner('rrt', obstacles, max_iterations=10000)
   ```

2. **Zmniejsz odległość bezpieczeństwa**:
   ```python
   planner = create_planner('astar', obstacles, safety_distance=1.0)
   ```

3. **Zmień algorytm** - spróbuj RRT zamiast A*

4. **Sprawdź przeszkody** - upewnij się, że start/goal nie są w kolizji

### Trajektoria jest niegładka

1. Zwiększ `smoothing_factor` w eksporcie:
   ```python
   exporter.smooth_path(path, smoothing_factor=0.7)
   ```

2. Dla RRT*, zwiększ `search_radius`

3. Zmniejsz `grid_resolution` dla A*

### Błędy importu

Upewnij się, że jesteś w odpowiednim folderze:
```bash
cd path_planning
python main.py
```

Lub dodaj ścieżkę:
```python
import sys
sys.path.insert(0, '/ścieżka/do/path_planning')
```

## 📝 Notatki

- System używa układu współrzędnych NED zgodnego z `dron_lqr_with_orientation.py`
- Wszystkie wysokości w interfejsie są podawane jako wartości dodatnie (AGL)
- Wewnętrznie Z jest ujemne (NED convention)
- Eksportowane trajektorie są kompatybilne z obecnym kontrolerem LQR
- Parametr `dt` musi być zgodny z krokiem czasowym kontrolera (domyślnie 0.01s)

## 🚀 Dalszy Rozwój

Możliwe rozszerzenia:

1. **Integracja z mapami**:
   - Import z OpenStreetMap
   - Mapy wysokościowe terenu
   - Google Maps API

2. **Zaawansowane algorytmy**:
   - Hybrid A*
   - PRM (Probabilistic Roadmap)
   - Dijkstra

3. **Dynamiczne przeszkody**:
   - Ruchome obiekty
   - Planowanie w czasie rzeczywistym

4. **Optymalizacja**:
   - Minimalizacja czasu
   - Minimalizacja energii
   - Wielokryterialna optymalizacja

## 📧 Kontakt

Dla pytań i sugestii, skontaktuj się z autorem projektu.

---

**Powodzenia w planowaniu trajektorii! 🚁✨**
