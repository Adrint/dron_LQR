"""
Quick Start - Prosty test systemu planowania trajektorii
Uruchom ten plik aby szybko przetestować system!
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import numpy as np
from obstacles import ObstacleManager
from path_planner import create_planner
from trajectory_export import TrajectoryExporter
from visualization import PathVisualizer


def quick_test():
    """Szybki test z domyślnymi parametrami"""
    
    print("="*70)
    print("QUICK START - Test Systemu Planowania Trajektorii")
    print("="*70)
    print()
    
    # 1. Stwórz przeszkody
    print("1. Tworzenie środowiska...")
    obstacles = ObstacleManager()
    
    # Dodaj budynek
    obstacles.add_box(
        position=(25, 0, -7.5),  # X=25m, Y=0m, wysokość środka 7.5m
        size=(10, 10, 15),        # 10x10x15 metrów
        name="Budynek_Centralny"
    )
    
    # Dodaj drzewa
    obstacles.add_cylinder((15, 5, 0), radius=2.0, height=12.0, name="Drzewo_1")
    obstacles.add_cylinder((35, -5, 0), radius=2.0, height=12.0, name="Drzewo_2")
    
    print(f"   ✓ Dodano {len(obstacles)} przeszkód")
    
    # 2. Zdefiniuj start i cel
    print("\n2. Definiowanie punktów...")
    start = np.array([0, 0, -2])    # Start: X=0, Y=0, wysokość 2m
    goal = np.array([50, 0, -2])    # Cel: X=50, Y=0, wysokość 2m
    print(f"   ✓ Start: X={start[0]}, Y={start[1]}, wysokość {-start[2]}m")
    print(f"   ✓ Cel:   X={goal[0]}, Y={goal[1]}, wysokość {-goal[2]}m")
    
    # 3. Zaplanuj ścieżkę
    print("\n3. Planowanie ścieżki (A*)...")
    planner = create_planner(
        algorithm='astar',
        obstacle_manager=obstacles,
        grid_resolution=0.5,
        safety_distance=1.5
    )
    
    path = planner.plan(start, goal)
    
    if path is None:
        print("   ❌ Nie znaleziono ścieżki!")
        return
    
    print(f"   ✓ Znaleziono ścieżkę z {len(path)} punktami")
    
    # Oblicz długość
    path_array = np.array(path)
    distances = np.linalg.norm(np.diff(path_array, axis=0), axis=1)
    total_distance = np.sum(distances)
    print(f"   ✓ Długość ścieżki: {total_distance:.2f} m")
    
    # 4. Eksportuj trajektorię
    print("\n4. Eksportowanie trajektorii...")
    Path('data/generated').mkdir(parents=True, exist_ok=True)
    
    exporter = TrajectoryExporter(velocity=2.0, dt=0.01)
    
    # Eksport do NumPy
    exporter.export_numpy(path, 'data/generated/quick_test.npz', smooth=True)
    print("   ✓ Zapisano: data/generated/quick_test.npz")
    
    # Eksport do Python module
    exporter.generate_trajectory_function(path, 'data/generated/quick_test_trajectory.py', smooth=True)
    print("   ✓ Zapisano: data/generated/quick_test_trajectory.py")
    
    # 5. Wizualizacja
    print("\n5. Wizualizacja...")
    visualizer = PathVisualizer(obstacles)
    
    visualizer.plot_3d(path, start, goal, title="Quick Test - Trajektoria Drona")
    visualizer.plot_2d_projections(path, start, goal)
    
    print("   ✓ Wyświetlanie wykresów...")
    
    # Podsumowanie
    print("\n" + "="*70)
    print("TEST ZAKOŃCZONY POMYŚLNIE!")
    print("="*70)
    print("\nWYGENEROWANE PLIKI:")
    print("  • data/generated/quick_test.npz")
    print("  • data/generated/quick_test_trajectory.py")
    print("\nABY UŻYĆ W PROJEKCIE DRONA:")
    print("  1. Skopiuj quick_test_trajectory.py do src_28_10/")
    print("  2. Zastąp trajectory.py lub zaimportuj jako moduł")
    print("  3. Uruchom dron_lqr_with_orientation.py")
    print("\nALBO uruchom pełny interfejs:")
    print("  python main.py")
    print("="*70)
    
    visualizer.show()


def main():
    """Main"""
    try:
        quick_test()
    except KeyboardInterrupt:
        print("\n\nPrzerwano przez użytkownika.")
    except Exception as e:
        print(f"\n\n❌ BŁĄD: {e}")
        import traceback
        traceback.print_exc()
        print("\nSpróbuj uruchomić: python main.py")


if __name__ == "__main__":
    main()
