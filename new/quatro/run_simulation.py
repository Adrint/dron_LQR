#!/usr/bin/env python3
"""
Launcher script for quadrocopter simulations
"""

import sys
import os

def print_menu():
    print("=" * 80)
    print("🚁 QUADROCOPTER SIMULATION LAUNCHER")
    print("=" * 80)
    print("\n📊 Wybierz symulację:\n")
    print("  2D Simulations (prostsze):")
    print("    [1] 2D PID  - Kontroler PID dla lotu 2D (X-Z)")
    print("    [2] 2D LQR  - Kontroler LQR dla lotu 2D")
    print()
    print("  3D Simulations (zaawansowane):")
    print("    [3] 3D PID  - Kontroler PID dla lotu 3D (X-Y-Z) ⭐ POLECANE")
    print("    [4] 3D LQR  - Kontroler LQR dla lotu 3D")
    print()
    print("    [0] Wyjście")
    print("\n" + "=" * 80)


def main():
    while True:
        print_menu()
        choice = input("\nWybierz opcję [0-4]: ").strip()
        
        if choice == '0':
            print("\n👋 Do widzenia!\n")
            sys.exit(0)
        
        elif choice == '1':
            print("\n🚀 Uruchamiam 2D PID...\n")
            os.chdir('simulations_2D')
            os.system(f'{sys.executable} dron_PID.py')
            os.chdir('..')
            input("\n\nNaciśnij Enter aby kontynuować...")
            
        elif choice == '2':
            print("\n🚀 Uruchamiam 2D LQR...\n")
            os.chdir('simulations_2D')
            os.system(f'{sys.executable} dron_LQR.py')
            os.chdir('..')
            input("\n\nNaciśnij Enter aby kontynuować...")
            
        elif choice == '3':
            print("\n🚀 Uruchamiam 3D PID...\n")
            os.chdir('simulations_3D')
            os.system(f'{sys.executable} dron_PID.py')
            os.chdir('..')
            input("\n\nNaciśnij Enter aby kontynuować...")
            
        elif choice == '4':
            print("\n🚀 Uruchamiam 3D LQR...\n")
            os.chdir('simulations_3D')
            os.system(f'{sys.executable} dron_LQR.py')
            os.chdir('..')
            input("\n\nNaciśnij Enter aby kontynuować...")
            
        else:
            print("\n❌ Nieprawidłowy wybór! Spróbuj ponownie.\n")
            input("Naciśnij Enter aby kontynuować...")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 Przerwano przez użytkownika. Do widzenia!\n")
        sys.exit(0)
