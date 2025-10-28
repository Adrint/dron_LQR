"""
Main program for drone path planning
Interactive interface for setting waypoints, obstacles, and planning paths
"""

import numpy as np
import sys
from pathlib import Path

# Add current directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from obstacles import ObstacleManager
from path_planner import create_planner
from trajectory_export import TrajectoryExporter
from visualization import PathVisualizer
import config


def print_header():
    """Print welcome header"""
    print("=" * 70)
    print("DRONE PATH PLANNING SYSTEM")
    print("=" * 70)
    print("Compatible with dron_lqr_with_orientation.py")
    print("NED Coordinate System: X=Forward, Y=Right, Z=Down (negative=altitude)")
    print("=" * 70)
    print()


def get_point_input(prompt: str, default: tuple = None) -> np.ndarray:
    """
    Get 3D point from user input
    
    Args:
        prompt: Prompt message
        default: Default values (x, y, z)
        
    Returns:
        Point as numpy array [x, y, z]
    """
    if default:
        print(f"{prompt} (domyślnie: X={default[0]}, Y={default[1]}, Z={default[2]})")
    else:
        print(prompt)
    
    x = input("  X (forward) [m]: ").strip()
    y = input("  Y (right) [m]: ").strip()
    z_agl = input("  Wysokość nad ziemią (pozytywna) [m]: ").strip()
    
    # Handle defaults
    if default:
        x = float(x) if x else default[0]
        y = float(y) if y else default[1]
        z_agl = float(z_agl) if z_agl else -default[2]  # Convert from NED
    else:
        x = float(x) if x else 0.0
        y = float(y) if y else 0.0
        z_agl = float(z_agl) if z_agl else 2.0
    
    # Convert altitude to NED (negative Z = altitude)
    z = -abs(float(z_agl))
    
    return np.array([x, y, z])


def setup_obstacles_interactive(obstacle_manager: ObstacleManager):
    """
    Interactive obstacle setup
    
    Args:
        obstacle_manager: Obstacle manager to populate
    """
    print("\n" + "="*70)
    print("KONFIGURACJA PRZESZKÓD")
    print("="*70)
    
    # Preset scenarios
    print("\nDostępne scenariusze:")
    print("1. Brak przeszkód (pusta przestrzeń)")
    print("2. Pojedynczy budynek")
    print("3. Las (drzewa)")
    print("4. Miasto (wiele budynków)")
    print("5. Własna konfiguracja")
    
    choice = input("\nWybierz scenariusz (1-5): ").strip()
    
    if choice == '1':
        print("Wybrano: Brak przeszkód")
        return
    
    elif choice == '2':
        print("Dodawanie pojedynczego budynku...")
        x = float(input("  X (pozycja) [m]: ") or "25")
        y = float(input("  Y (pozycja) [m]: ") or "0")
        width = float(input("  Szerokość X [m]: ") or "10")
        depth = float(input("  Szerokość Y [m]: ") or "10")
        height = float(input("  Wysokość [m]: ") or "15")
        
        z_center = -height / 2  # NED
        obstacle_manager.add_box(
            position=(x, y, z_center),
            size=(width, depth, height),
            name="Budynek"
        )
        print(f"✓ Dodano budynek na pozycji ({x}, {y}) o wymiarach {width}x{depth}x{height}m")
    
    elif choice == '3':
        print("Dodawanie drzew...")
        num_trees = int(input("  Liczba drzew (1-20): ") or "5")
        
        # Random placement
        np.random.seed(42)
        for i in range(num_trees):
            x = np.random.uniform(10, 40)
            y = np.random.uniform(-10, 10)
            radius = np.random.uniform(1.5, 3.0)
            height = np.random.uniform(8, 15)
            
            obstacle_manager.add_cylinder(
                position=(x, y, 0),  # Base on ground
                radius=radius,
                height=height,
                name=f"Drzewo_{i+1}"
            )
        
        print(f"✓ Dodano {num_trees} drzew")
    
    elif choice == '4':
        print("Dodawanie budynków miejskich...")
        
        # Create city layout
        buildings = [
            ((15, -5, -7.5), (8, 6, 15)),
            ((15, 8, -10), (8, 6, 20)),
            ((30, -8, -12.5), (10, 8, 25)),
            ((30, 5, -15), (12, 10, 30)),
            ((45, 0, -10), (10, 10, 20)),
        ]
        
        for i, (pos, size) in enumerate(buildings):
            obstacle_manager.add_box(pos, size, name=f"Budynek_{i+1}")
        
        print(f"✓ Dodano {len(buildings)} budynków")
    
    elif choice == '5':
        print("Własna konfiguracja - dodawanie przeszkód...")
        
        while True:
            print("\nTyp przeszkody:")
            print("1. Pudełko (budynek)")
            print("2. Cylinder (drzewo, wieża)")
            print("3. Kula")
            print("0. Zakończ dodawanie")
            
            obs_type = input("Wybierz typ (0-3): ").strip()
            
            if obs_type == '0':
                break
            
            elif obs_type == '1':
                print("Pudełko:")
                x = float(input("  X [m]: "))
                y = float(input("  Y [m]: "))
                z_agl = float(input("  Wysokość środka nad ziemią [m]: "))
                width = float(input("  Szerokość X [m]: "))
                depth = float(input("  Szerokość Y [m]: "))
                height = float(input("  Wysokość [m]: "))
                name = input("  Nazwa (opcjonalnie): ").strip()
                
                z = -z_agl
                obstacle_manager.add_box((x, y, z), (width, depth, height), name)
                print("✓ Dodano pudełko")
            
            elif obs_type == '2':
                print("Cylinder:")
                x = float(input("  X [m]: "))
                y = float(input("  Y [m]: "))
                radius = float(input("  Promień [m]: "))
                height = float(input("  Wysokość [m]: "))
                name = input("  Nazwa (opcjonalnie): ").strip()
                
                obstacle_manager.add_cylinder((x, y, 0), radius, height, name)
                print("✓ Dodano cylinder")
            
            elif obs_type == '3':
                print("Kula:")
                x = float(input("  X [m]: "))
                y = float(input("  Y [m]: "))
                z_agl = float(input("  Wysokość środka nad ziemią [m]: "))
                radius = float(input("  Promień [m]: "))
                name = input("  Nazwa (opcjonalnie): ").strip()
                
                z = -z_agl
                obstacle_manager.add_sphere((x, y, z), radius, name)
                print("✓ Dodano kulę")
    
    print(f"\nLiczba przeszkód: {len(obstacle_manager)}")


def select_algorithm() -> tuple:
    """
    Select path planning algorithm
    
    Returns:
        Tuple of (algorithm_name, parameters_dict)
    """
    print("\n" + "="*70)
    print("WYBÓR ALGORYTMU PLANOWANIA")
    print("="*70)
    
    print("\nDostępne algorytmy:")
    print("1. A* (A-star) - szybki, optymalny dla siatki")
    print("2. RRT (Rapidly-exploring Random Tree) - dobry dla złożonych przestrzeni")
    print("3. RRT* - optymalny wariant RRT (wolniejszy ale lepsze ścieżki)")
    
    choice = input("\nWybierz algorytm (1-3, domyślnie 1): ").strip()
    
    if choice == '2':
        print("\nParametry RRT:")
        max_iter = int(input(f"  Maksymalna liczba iteracji ({config.RRT_MAX_ITERATIONS}): ") or config.RRT_MAX_ITERATIONS)
        step_size = float(input(f"  Rozmiar kroku ({config.RRT_STEP_SIZE} m): ") or config.RRT_STEP_SIZE)
        
        return 'rrt', {
            'max_iterations': max_iter,
            'step_size': step_size
        }
    
    elif choice == '3':
        print("\nParametry RRT*:")
        max_iter = int(input(f"  Maksymalna liczba iteracji ({config.RRT_MAX_ITERATIONS}): ") or config.RRT_MAX_ITERATIONS)
        step_size = float(input(f"  Rozmiar kroku ({config.RRT_STEP_SIZE} m): ") or config.RRT_STEP_SIZE)
        search_radius = float(input(f"  Promień przeszukiwania ({config.RRT_STAR_SEARCH_RADIUS} m): ") or config.RRT_STAR_SEARCH_RADIUS)
        
        return 'rrt_star', {
            'max_iterations': max_iter,
            'step_size': step_size,
            'search_radius': search_radius
        }
    
    else:  # Default A*
        print("\nParametry A*:")
        grid_res = float(input(f"  Rozdzielczość siatki ({config.GRID_RESOLUTION} m): ") or config.GRID_RESOLUTION)
        
        return 'astar', {
            'grid_resolution': grid_res
        }


def main():
    """Main program"""
    print_header()
    
    # Initialize managers
    obstacle_manager = ObstacleManager()
    
    # Get start and goal points
    print("PUNKT STARTOWY (A)")
    print("-" * 70)
    start = get_point_input("Wprowadź punkt startowy:", default=(0, 0, 2))
    print(f"✓ Start: X={start[0]:.1f} m, Y={start[1]:.1f} m, Z={start[2]:.1f} m (Wysokość: {-start[2]:.1f} m AGL)")
    
    print("\nPUNKT DOCELOWY (B)")
    print("-" * 70)
    goal = get_point_input("Wprowadź punkt docelowy:", default=(50, 0, 2))
    print(f"✓ Cel: X={goal[0]:.1f} m, Y={goal[1]:.1f} m, Z={goal[2]:.1f} m (Wysokość: {-goal[2]:.1f} m AGL)")
    
    # Setup obstacles
    setup_obstacles_interactive(obstacle_manager)
    
    # Select algorithm
    algorithm, params = select_algorithm()
    
    # Safety distance
    print("\n" + "="*70)
    safety_distance = float(input(f"Odległość bezpieczeństwa od przeszkód ({config.SAFETY_DISTANCE} m): ") or config.SAFETY_DISTANCE)
    
    # Drone velocity for trajectory generation
    velocity = float(input(f"Prędkość drona ({config.DRONE_VELOCITY} m/s): ") or config.DRONE_VELOCITY)
    dt = float(input(f"Krok czasowy ({config.EXPORT_TIME_STEP} s): ") or config.EXPORT_TIME_STEP)
    
    # Create planner
    print("\n" + "="*70)
    print("PLANOWANIE TRAJEKTORII")
    print("="*70)
    print(f"Algorytm: {algorithm.upper()}")
    print(f"Odległość bezpieczeństwa: {safety_distance} m")
    print(f"Prędkość: {velocity} m/s")
    print("Planowanie...")
    
    planner = create_planner(
        algorithm=algorithm,
        obstacle_manager=obstacle_manager,
        safety_distance=safety_distance,
        **params
    )
    
    # Plan path
    path = planner.plan(start, goal)
    
    if path is None:
        print("\n❌ NIE ZNALEZIONO ŚCIEŻKI!")
        print("Możliwe przyczyny:")
        print("  - Przeszkody blokują wszystkie możliwe trasy")
        print("  - Punkt startowy lub końcowy jest w kolizji")
        print("  - Zbyt mała liczba iteracji (dla RRT/RRT*)")
        return
    
    print(f"\n✓ ZNALEZIONO ŚCIEŻKĘ!")
    print(f"Liczba punktów: {len(path)}")
    
    # Calculate path length
    path_array = np.array(path)
    distances = np.linalg.norm(np.diff(path_array, axis=0), axis=1)
    total_distance = np.sum(distances)
    print(f"Całkowita długość: {total_distance:.2f} m")
    print(f"Szacowany czas: {total_distance / velocity:.2f} s")
    
    # Export trajectory
    print("\n" + "="*70)
    print("EKSPORT TRAJEKTORII")
    print("="*70)
    
    # Create output directory
    output_dir = Path(config.GENERATED_DIR)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    exporter = TrajectoryExporter(velocity=velocity, dt=dt)
    
    # Export to multiple formats
    print("\nEksportowanie do plików...")
    
    # 1. NumPy format (.npz) - for loading in Python
    npz_file = output_dir / "trajectory_output.npz"
    exporter.export_numpy(path, str(npz_file), smooth=True)
    
    # 2. JSON format (human-readable)
    json_file = output_dir / "trajectory_output.json"
    exporter.export_json(path, str(json_file))
    
    # 3. Python module (direct replacement for trajectory.py)
    py_file = output_dir / "custom_trajectory.py"
    exporter.generate_trajectory_function(path, str(py_file), smooth=True)
    
    print("\n" + "="*70)
    print("PLIKI WYJŚCIOWE")
    print("="*70)
    print(f"1. NumPy (do wczytania):  {npz_file}")
    print(f"2. JSON (do podglądu):    {json_file}")
    print(f"3. Python (do importu):   {py_file}")
    
    print("\n" + "="*70)
    print("JAK UŻYĆ W PROJEKCIE DRONA")
    print("="*70)
    print("\nOpcja 1: Zamień trajectory.py")
    print(f"  1. Skopiuj: {py_file}")
    print(f"  2. Zastąp: D:\\Praca_Magisterska_PW\\src_28_10\\trajectory.py")
    print(f"  3. Uruchom dron_lqr_with_orientation.py")
    
    print("\nOpcja 2: Wczytaj z pliku .npz")
    print("  1. Dodaj w dron_lqr_with_orientation.py:")
    print(f"     data = np.load('{npz_file}')")
    print("     X_ref_all = data['X_ref']")
    print("     Y_ref_all = data['Y_ref']")
    print("     Z_ref_all = data['Z_ref']")
    print("     # etc...")
    
    # Visualization
    print("\n" + "="*70)
    viz_choice = input("Czy wyświetlić wizualizację? (t/n, domyślnie t): ").strip().lower()
    
    if viz_choice not in ['n', 'nie', 'no']:
        print("Generowanie wizualizacji...")
        
        visualizer = PathVisualizer(obstacle_manager)
        
        # 3D plot
        visualizer.plot_3d(path, start, goal, 
                          title=f"Trajektoria drona ({algorithm.upper()})")
        
        # 2D projections
        visualizer.plot_2d_projections(path, start, goal)
        
        visualizer.show()
    
    print("\n" + "="*70)
    print("ZAKOŃCZONO")
    print("="*70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nPrzerwano przez użytkownika.")
    except Exception as e:
        print(f"\n\n❌ BŁĄD: {e}")
        import traceback
        traceback.print_exc()
