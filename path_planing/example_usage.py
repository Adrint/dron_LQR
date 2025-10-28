"""
Example usage of path planning system (programmatic interface)
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from obstacles import ObstacleManager
from path_planner import create_planner
from trajectory_export import TrajectoryExporter
from visualization import PathVisualizer


def example_basic():
    """Basic example: Simple path with one obstacle"""
    print("="*70)
    print("PRZYKŁAD 1: Podstawowe planowanie ścieżki")
    print("="*70)
    
    # Setup
    obstacle_manager = ObstacleManager()
    
    # Add single building obstacle
    obstacle_manager.add_box(
        position=(25, 0, -7.5),  # Center at X=25, Y=0, Z=-7.5 (7.5m center height)
        size=(10, 10, 15),        # 10x10x15 meters
        name="Budynek"
    )
    
    # Define start and goal
    start = np.array([0, 0, -2])    # Start at X=0, Y=0, altitude 2m
    goal = np.array([50, 0, -2])    # Goal at X=50, Y=0, altitude 2m
    
    # Create A* planner
    planner = create_planner(
        algorithm='astar',
        obstacle_manager=obstacle_manager,
        grid_resolution=0.5,
        safety_distance=1.5
    )
    
    # Plan path
    print("\nPlanowanie ścieżki...")
    path = planner.plan(start, goal)
    
    if path:
        print(f"✓ Znaleziono ścieżkę z {len(path)} punktami")
        
        # Export
        exporter = TrajectoryExporter(velocity=2.0, dt=0.01)
        exporter.export_numpy(path, 'data/generated/example1_trajectory.npz')
        
        # Visualize
        visualizer = PathVisualizer(obstacle_manager)
        visualizer.plot_3d(path, start, goal, title="Przykład 1: Pojedynczy budynek")
        visualizer.show()
    else:
        print("❌ Nie znaleziono ścieżki")


def example_forest():
    """Example with multiple cylindrical obstacles (forest)"""
    print("\n" + "="*70)
    print("PRZYKŁAD 2: Lot przez las")
    print("="*70)
    
    obstacle_manager = ObstacleManager()
    
    # Add trees
    np.random.seed(42)
    tree_positions = [
        (15, -3, 0), (18, 4, 0), (22, -2, 0), (25, 5, 0),
        (28, -4, 0), (32, 2, 0), (35, -5, 0), (38, 3, 0)
    ]
    
    for i, (x, y, z_base) in enumerate(tree_positions):
        obstacle_manager.add_cylinder(
            position=(x, y, z_base),
            radius=2.0,
            height=12.0,
            name=f"Drzewo_{i+1}"
        )
    
    start = np.array([0, 0, -3])
    goal = np.array([45, 0, -3])
    
    # Use RRT for this scenario
    planner = create_planner(
        algorithm='rrt',
        obstacle_manager=obstacle_manager,
        safety_distance=1.0,
        max_iterations=3000,
        step_size=1.5
    )
    
    print("\nPlanowanie ścieżki przez las...")
    path = planner.plan(start, goal)
    
    if path:
        print(f"✓ Znaleziono ścieżkę z {len(path)} punktami")
        
        exporter = TrajectoryExporter(velocity=2.0, dt=0.01)
        exporter.export_numpy(path, 'data/generated/example2_forest.npz')
        
        visualizer = PathVisualizer(obstacle_manager)
        visualizer.plot_3d(path, start, goal, title="Przykład 2: Las")
        visualizer.plot_2d_projections(path, start, goal)
        visualizer.show()
    else:
        print("❌ Nie znaleziono ścieżki")


def example_city():
    """Example with complex urban environment"""
    print("\n" + "="*70)
    print("PRZYKŁAD 3: Lot przez miasto")
    print("="*70)
    
    obstacle_manager = ObstacleManager()
    
    # Add multiple buildings of varying heights
    buildings = [
        # (position, size, name)
        ((15, -5, -7.5), (8, 6, 15), "Niski_1"),
        ((15, 8, -10), (8, 6, 20), "Średni_1"),
        ((30, -8, -12.5), (10, 8, 25), "Wysoki_1"),
        ((30, 5, -15), (12, 10, 30), "Wieżowiec"),
        ((45, -3, -10), (10, 10, 20), "Średni_2"),
        ((45, 7, -8.5), (8, 8, 17), "Niski_2"),
    ]
    
    for pos, size, name in buildings:
        obstacle_manager.add_box(pos, size, name)
    
    # Add some spherical obstacles (decorative elements, balloons, etc.)
    obstacle_manager.add_sphere((25, 0, -5), 2.0, "Balon_1")
    obstacle_manager.add_sphere((40, -5, -8), 1.5, "Balon_2")
    
    start = np.array([0, 0, -5])
    goal = np.array([55, 0, -5])
    
    # Use RRT* for optimal path
    planner = create_planner(
        algorithm='rrt_star',
        obstacle_manager=obstacle_manager,
        safety_distance=2.0,
        max_iterations=5000,
        step_size=2.0,
        search_radius=5.0
    )
    
    print("\nPlanowanie optymalnej ścieżki przez miasto...")
    path = planner.plan(start, goal)
    
    if path:
        print(f"✓ Znaleziono ścieżkę z {len(path)} punktami")
        
        exporter = TrajectoryExporter(velocity=3.0, dt=0.01)
        
        # Export all formats
        exporter.export_numpy(path, 'data/generated/example3_city.npz')
        exporter.export_json(path, 'data/generated/example3_city.json')
        exporter.generate_trajectory_function(path, 'data/generated/example3_city_trajectory.py')
        
        visualizer = PathVisualizer(obstacle_manager)
        visualizer.plot_3d(path, start, goal, title="Przykład 3: Miasto")
        visualizer.plot_2d_projections(path, start, goal)
        visualizer.show()
    else:
        print("❌ Nie znaleziono ścieżki")


def example_3d_maneuvers():
    """Example with obstacles requiring 3D maneuvering"""
    print("\n" + "="*70)
    print("PRZYKŁAD 4: Złożone manewry 3D")
    print("="*70)
    
    obstacle_manager = ObstacleManager()
    
    # Create a "tunnel" with obstacles above and below
    for x in range(10, 45, 8):
        # Upper obstacles
        obstacle_manager.add_box(
            position=(x, 0, -15),
            size=(6, 12, 8),
            name=f"Górny_{x}"
        )
        # Lower obstacles
        obstacle_manager.add_box(
            position=(x + 4, 0, -3),
            size=(6, 12, 6),
            name=f"Dolny_{x}"
        )
    
    start = np.array([0, 0, -8])   # Start in middle
    goal = np.array([50, 0, -8])   # Goal in middle
    
    planner = create_planner(
        algorithm='rrt',
        obstacle_manager=obstacle_manager,
        safety_distance=1.5,
        max_iterations=5000,
        step_size=1.0
    )
    
    print("\nPlanowanie ścieżki przez przeszkody...")
    path = planner.plan(start, goal)
    
    if path:
        print(f"✓ Znaleziono ścieżkę z {len(path)} punktami")
        
        exporter = TrajectoryExporter(velocity=2.5, dt=0.01)
        exporter.export_numpy(path, 'data/generated/example4_3d.npz')
        
        visualizer = PathVisualizer(obstacle_manager)
        visualizer.plot_3d(path, start, goal, title="Przykład 4: Manewry 3D")
        visualizer.plot_2d_projections(path, start, goal)
        visualizer.show()
    else:
        print("❌ Nie znaleziono ścieżki")


def compare_algorithms():
    """Compare different algorithms on the same problem"""
    print("\n" + "="*70)
    print("PRZYKŁAD 5: Porównanie algorytmów")
    print("="*70)
    
    # Setup common scenario
    obstacle_manager = ObstacleManager()
    obstacle_manager.add_box((25, 0, -10), (12, 12, 20), "Budynek_Centralny")
    obstacle_manager.add_cylinder((15, 8, 0), 3, 15, "Wieża_1")
    obstacle_manager.add_cylinder((35, -8, 0), 3, 15, "Wieża_2")
    
    start = np.array([0, 0, -5])
    goal = np.array([50, 0, -5])
    
    algorithms = ['astar', 'rrt', 'rrt_star']
    results = {}
    
    import time
    
    for algo in algorithms:
        print(f"\n--- {algo.upper()} ---")
        
        if algo == 'astar':
            planner = create_planner(algo, obstacle_manager, grid_resolution=0.5, safety_distance=1.5)
        elif algo == 'rrt':
            planner = create_planner(algo, obstacle_manager, safety_distance=1.5, 
                                   max_iterations=3000, step_size=1.0)
        else:  # rrt_star
            planner = create_planner(algo, obstacle_manager, safety_distance=1.5,
                                   max_iterations=3000, step_size=1.0, search_radius=4.0)
        
        start_time = time.time()
        path = planner.plan(start, goal)
        elapsed_time = time.time() - start_time
        
        if path:
            path_array = np.array(path)
            distances = np.linalg.norm(np.diff(path_array, axis=0), axis=1)
            total_distance = np.sum(distances)
            
            results[algo] = {
                'path': path,
                'waypoints': len(path),
                'distance': total_distance,
                'time': elapsed_time
            }
            
            print(f"✓ Waypoints: {len(path)}")
            print(f"✓ Długość: {total_distance:.2f} m")
            print(f"✓ Czas: {elapsed_time:.3f} s")
        else:
            print("❌ Nie znaleziono ścieżki")
    
    # Print comparison
    print("\n" + "="*70)
    print("PORÓWNANIE")
    print("="*70)
    print(f"{'Algorytm':<12} {'Punkty':<10} {'Długość [m]':<15} {'Czas [s]':<10}")
    print("-" * 70)
    
    for algo, data in results.items():
        print(f"{algo.upper():<12} {data['waypoints']:<10} {data['distance']:<15.2f} {data['time']:<10.3f}")
    
    # Visualize best result
    if results:
        best_algo = min(results.items(), key=lambda x: x[1]['distance'])
        print(f"\nNajkrótsza ścieżka: {best_algo[0].upper()}")
        
        visualizer = PathVisualizer(obstacle_manager)
        visualizer.plot_3d(best_algo[1]['path'], start, goal, 
                          title=f"Najlepsza ścieżka: {best_algo[0].upper()}")
        visualizer.show()


def main():
    """Run examples"""
    print("SYSTEM PLANOWANIA TRAJEKTORII - PRZYKŁADY UŻYCIA")
    print("="*70)
    print("\nDostępne przykłady:")
    print("1. Podstawowe planowanie (jeden budynek)")
    print("2. Lot przez las (wiele cylindrów)")
    print("3. Lot przez miasto (złożone środowisko)")
    print("4. Złożone manewry 3D")
    print("5. Porównanie algorytmów")
    print("0. Uruchom wszystkie")
    
    choice = input("\nWybierz przykład (0-5): ").strip()
    
    examples = {
        '1': example_basic,
        '2': example_forest,
        '3': example_city,
        '4': example_3d_maneuvers,
        '5': compare_algorithms
    }
    
    if choice == '0':
        for func in examples.values():
            func()
            input("\nNaciśnij Enter aby kontynuować...")
    elif choice in examples:
        examples[choice]()
    else:
        print("Nieprawidłowy wybór!")


if __name__ == "__main__":
    # Create data directories
    Path('data/generated').mkdir(parents=True, exist_ok=True)
    
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nPrzerwano.")
    except Exception as e:
        print(f"\n\n❌ BŁĄD: {e}")
        import traceback
        traceback.print_exc()
