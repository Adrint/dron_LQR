"""
Skrypt do wizualizacji trajektorii 3D przed uruchomieniem symulacji.
Przydatne do sprawdzenia jak wygląda ścieżka referencji.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from trajectory import generate_3d_trajectory
from constants import Vel

# Parametry
dt = 0.01
total_time = 50.0

print("=" * 80)
print("WIZUALIZACJA TRAJEKTORII 3D")
print("=" * 80)

# Generuj trajektorię
t_ref, X_ref, Y_ref, Z_ref, Vx_ref, Vy_ref, Vz_ref = generate_3d_trajectory(
    Vel, dt, total_time
)

print(f"\nWygenerowano {len(t_ref)} punktów trajektorii")
print(f"Czas symulacji: {total_time}s")
print(f"Krok czasowy: {dt}s")
print(f"Prędkość bazowa: {Vel}m/s")

# Statystyki
print(f"\n{'='*80}")
print("STATYSTYKI TRAJEKTORII")
print(f"{'='*80}")

print(f"\nPOZYCJA:")
print(f"  X: min={X_ref.min():.2f}m, max={X_ref.max():.2f}m, zakres={X_ref.max()-X_ref.min():.2f}m")
print(f"  Y: min={Y_ref.min():.2f}m, max={Y_ref.max():.2f}m, zakres={Y_ref.max()-Y_ref.min():.2f}m")
print(f"  Z: min={Z_ref.min():.2f}m, max={Z_ref.max():.2f}m, zakres={Z_ref.max()-Z_ref.min():.2f}m")

print(f"\nPRĘDKOŚĆ:")
print(f"  Vx: min={Vx_ref.min():.2f}m/s, max={Vx_ref.max():.2f}m/s")
print(f"  Vy: min={Vy_ref.min():.2f}m/s, max={Vy_ref.max():.2f}m/s")
print(f"  Vz: min={Vz_ref.min():.2f}m/s, max={Vz_ref.max():.2f}m/s")

# Prędkość całkowita
V_total = np.sqrt(Vx_ref**2 + Vy_ref**2 + Vz_ref**2)
print(f"  V_total: min={V_total.min():.2f}m/s, max={V_total.max():.2f}m/s, średnia={V_total.mean():.2f}m/s")

# Przyspieszenia (dla oceny wymagań od kontrolera)
ax = np.gradient(Vx_ref, dt)
ay = np.gradient(Vy_ref, dt)
az = np.gradient(Vz_ref, dt)
a_total = np.sqrt(ax**2 + ay**2 + az**2)

print(f"\nPRZYSPIESZENIE (wymagania od kontrolera):")
print(f"  ax: min={ax.min():.2f}m/s², max={ax.max():.2f}m/s²")
print(f"  ay: min={ay.min():.2f}m/s², max={ay.max():.2f}m/s²")
print(f"  az: min={az.min():.2f}m/s², max={az.max():.2f}m/s²")
print(f"  a_total: max={a_total.max():.2f}m/s²")

# Całkowita długość trajektorii
distances = np.sqrt(np.diff(X_ref)**2 + np.diff(Y_ref)**2 + np.diff(Z_ref)**2)
total_distance = distances.sum()
print(f"\nCAŁKOWITA DŁUGOŚĆ TRAJEKTORII: {total_distance:.2f}m")

# Analiza manewrów
print(f"\n{'='*80}")
print("ANALIZA MANEWRÓW")
print(f"{'='*80}")

def analyze_segment(t_start, t_end, name):
    idx_start = int(t_start / dt)
    idx_end = int(t_end / dt)
    
    Y_segment = Y_ref[idx_start:idx_end]
    Z_segment = Z_ref[idx_start:idx_end]
    Vy_segment = Vy_ref[idx_start:idx_end]
    
    print(f"\n{name} ({t_start}s - {t_end}s):")
    print(f"  Y: Δ={Y_segment.max() - Y_segment.min():.2f}m")
    print(f"  Z: Δ={Z_segment.max() - Z_segment.min():.2f}m")
    print(f"  Vy: max={abs(Vy_segment).max():.2f}m/s")

analyze_segment(5, 10, "Manewer 1: S-curve")
analyze_segment(10, 15, "Manewer 2: Sharp turn")
analyze_segment(15, 22, "Manewer 3: Slalom")
analyze_segment(22, 28, "Manewer 4: Circle/arc")
analyze_segment(28, 35, "Manewer 5: Return to center")

# Tworzenie wykresów
fig = plt.figure(figsize=(18, 10))

# 1. Trajektoria 3D - główny widok
ax1 = fig.add_subplot(2, 3, 1, projection='3d')
ax1.plot(X_ref, Y_ref, Z_ref, 'b-', linewidth=2, label='Trajektoria')
ax1.scatter([X_ref[0]], [Y_ref[0]], [Z_ref[0]], c='green', s=200, marker='o', label='Start')
ax1.scatter([X_ref[-1]], [Y_ref[-1]], [Z_ref[-1]], c='red', s=200, marker='x', label='Koniec')
ax1.set_xlabel('X [m]', fontsize=10)
ax1.set_ylabel('Y [m]', fontsize=10)
ax1.set_zlabel('Z [m]', fontsize=10)
ax1.set_title('Trajektoria 3D - Widok perspektywiczny', fontsize=12, fontweight='bold')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.view_init(elev=20, azim=45)

# 2. Trajektoria 3D - widok z góry (XY)
ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(X_ref, Y_ref, 'b-', linewidth=2)
ax2.scatter([X_ref[0]], [Y_ref[0]], c='green', s=100, marker='o', label='Start')
ax2.scatter([X_ref[-1]], [Y_ref[-1]], c='red', s=100, marker='x', label='Koniec')
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Y [m]')
ax2.set_title('Widok z góry (XY)', fontsize=12, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.axis('equal')
ax2.legend()

# 3. Profil wysokości (XZ)
ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(X_ref, Z_ref, 'b-', linewidth=2)
ax3.set_xlabel('X [m]')
ax3.set_ylabel('Z [m]')
ax3.set_title('Profil wysokości (XZ)', fontsize=12, fontweight='bold')
ax3.grid(True, alpha=0.3)

# 4. Pozycje w czasie
ax4 = fig.add_subplot(2, 3, 4)
ax4.plot(t_ref, X_ref, 'r-', label='X', linewidth=2)
ax4.plot(t_ref, Y_ref, 'g-', label='Y', linewidth=2)
ax4.plot(t_ref, Z_ref, 'b-', label='Z', linewidth=2)
ax4.set_xlabel('Czas [s]')
ax4.set_ylabel('Pozycja [m]')
ax4.set_title('Pozycje vs Czas', fontsize=12, fontweight='bold')
ax4.legend()
ax4.grid(True, alpha=0.3)

# 5. Prędkości w czasie
ax5 = fig.add_subplot(2, 3, 5)
ax5.plot(t_ref, Vx_ref, 'r-', label='Vx', linewidth=2)
ax5.plot(t_ref, Vy_ref, 'g-', label='Vy', linewidth=2)
ax5.plot(t_ref, Vz_ref, 'b-', label='Vz', linewidth=2)
ax5.plot(t_ref, V_total, 'k--', label='V_total', linewidth=2)
ax5.set_xlabel('Czas [s]')
ax5.set_ylabel('Prędkość [m/s]')
ax5.set_title('Prędkości vs Czas', fontsize=12, fontweight='bold')
ax5.legend()
ax5.grid(True, alpha=0.3)

# 6. Przyspieszenia w czasie
ax6 = fig.add_subplot(2, 3, 6)
ax6.plot(t_ref, ax, 'r-', label='ax', linewidth=2, alpha=0.7)
ax6.plot(t_ref, ay, 'g-', label='ay', linewidth=2, alpha=0.7)
ax6.plot(t_ref, az, 'b-', label='az', linewidth=2, alpha=0.7)
ax6.set_xlabel('Czas [s]')
ax6.set_ylabel('Przyspieszenie [m/s²]')
ax6.set_title('Przyspieszenia vs Czas', fontsize=12, fontweight='bold')
ax6.legend()
ax6.grid(True, alpha=0.3)

plt.tight_layout()

# Drugi wykres - szczegółowa analiza manewrów
fig2 = plt.figure(figsize=(16, 10))
fig2.suptitle('Szczegółowa analiza manewrów', fontsize=14, fontweight='bold')

maneuvers = [
    (0, 5, "Start (0-5s)"),
    (5, 10, "S-curve (5-10s)"),
    (10, 15, "Sharp turn (10-15s)"),
    (15, 22, "Slalom (15-22s)"),
    (22, 28, "Circle/arc (22-28s)"),
    (28, 35, "Return (28-35s)"),
    (35, 50, "Final approach (35-50s)")
]

for idx, (t_start, t_end, name) in enumerate(maneuvers):
    ax = fig2.add_subplot(3, 3, idx + 1, projection='3d')
    
    idx_start = int(t_start / dt)
    idx_end = int(t_end / dt)
    
    X_seg = X_ref[idx_start:idx_end]
    Y_seg = Y_ref[idx_start:idx_end]
    Z_seg = Z_ref[idx_start:idx_end]
    
    ax.plot(X_seg, Y_seg, Z_seg, 'b-', linewidth=3)
    ax.scatter([X_seg[0]], [Y_seg[0]], [Z_seg[0]], c='green', s=100, marker='o')
    ax.scatter([X_seg[-1]], [Y_seg[-1]], [Z_seg[-1]], c='red', s=100, marker='x')
    
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('Y [m]', fontsize=8)
    ax.set_zlabel('Z [m]', fontsize=8)
    ax.set_title(name, fontsize=10, fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Auto-scale dla lepszej widoczności
    ax.set_xlim([X_seg.min() - 1, X_seg.max() + 1])
    ax.set_ylim([Y_seg.min() - 1, Y_seg.max() + 1])
    ax.set_zlim([Z_seg.min() - 0.5, Z_seg.max() + 0.5])

plt.tight_layout()

print(f"\n{'='*80}")
print("WYKRESY GOTOWE!")
print(f"{'='*80}")
print("\nPokaz wykresy:")
print("  - Figura 1: Przegląd ogólny trajektorii")
print("  - Figura 2: Szczegółowa analiza każdego manewru")
print("\nZamknij okna aby zakończyć.")

plt.show()
