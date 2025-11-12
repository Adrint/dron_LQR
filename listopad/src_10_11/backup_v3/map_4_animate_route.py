
"""
animate_route.py
----------------
Plik 3: animacja lotu na podstawie planned_path.npz (oraz budynków jako tło).
Wejście:
- planned_path.npz (X_ref, Y_ref, Z_ref z plan_route.py)
Uwaga: Do narysowania budynków potrzebujemy warsaw_buildings.pkl (opcjonalnie).
"""
import pickle
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import osmnx as ox
import pandas as pd

DATA_DIR = Path("data")
PATH_FILE = Path("planned_path.npz")

ox.settings.use_cache = True


def main():
    if not PATH_FILE.exists():
        print("✗ Brak planned_path.npz. Uruchom plan_route.py")
        return

    # Wczytaj ścieżkę (układ metrów lokalnych X,Y oraz NED Z)
    npz = np.load(PATH_FILE)
    X_ref = npz["X_ref"]
    Y_ref = npz["Y_ref"]
    Z_ref = npz["Z_ref"]
    # Wysokość dodatnia „w górę”
    Z_up = -Z_ref

    # Przybliżona długość wzdłuż XY
    dist = np.zeros(len(X_ref))
    for i in range(1, len(X_ref)):
        dist[i] = dist[i - 1] + np.hypot(X_ref[i] - X_ref[i - 1], Y_ref[i] - Y_ref[i - 1])

    # Spróbuj wczytać budynki (opcjonalnie – rysowane w XY jako statyczne tło)
    buildings = None
    bldg_path = DATA_DIR / "warsaw_buildings.pkl"
    if bldg_path.exists():
        try:
            buildings = pd.read_pickle(bldg_path)
        except Exception:
            buildings = None

    # Prosta animacja 1x2: XY + ZX
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # XY
    ax1.set_aspect("equal")
    ax1.set_facecolor("#f0f0f0")
    ax1.grid(True, alpha=0.3, linestyle="--")
    ax1.set_xlabel("X [m]")
    ax1.set_ylabel("Y [m]")
    ax1.set_title("Widok z góry (XY)")

    # Rysuj ślad docelowy
    ax1.plot(X_ref, Y_ref, "b-", lw=2, alpha=0.3)

    # Dynamiczny przebyty odcinek + znacznik
    traveled1, = ax1.plot([], [], "g-", lw=3, alpha=0.8, zorder=200)
    drone_pt1, = ax1.plot([], [], "ro", markersize=10, markeredgecolor="black", markeredgewidth=1.5, zorder=300)

    # ZX (profil po dystansie s)
    ax2.set_facecolor("#f9f9f9")
    ax2.grid(True, alpha=0.3, linestyle="--")
    ax2.set_xlabel("Dystans [m]")
    ax2.set_ylabel("Wysokość [m]")
    ax2.set_title("Profil (ZX)")
    ax2.plot(dist, Z_up, "b-", lw=2, alpha=0.6, zorder=10)
    drone_pt2, = ax2.plot([], [], "ro", markersize=8, markeredgecolor="black", markeredgewidth=1.2, zorder=20)

    # Skalowanie osi
    ax1.set_xlim(min(X_ref) - 30, max(X_ref) + 30)
    ax1.set_ylim(min(Y_ref) - 30, max(Y_ref) + 30)
    ax2.set_xlim(0, dist[-1] if len(dist) else 1.0)
    ax2.set_ylim(bottom=0, top=(float(np.max(Z_up)) if len(Z_up) else 0.0) * 1.15 + 1e-6)

    # Ustawienia animacji
    fps = 30
    speed_multiplier = 5.0  # większa wartość -> szybsza animacja
    total_time = (dist[-1] / 5.0) / speed_multiplier if len(dist) else 1.0
    total_frames = max(1, int(total_time * fps))

    t = np.linspace(0.0, 1.0, total_frames)
    drone_x = np.interp(t, np.linspace(0.0, 1.0, len(X_ref)), X_ref)
    drone_y = np.interp(t, np.linspace(0.0, 1.0, len(Y_ref)), Y_ref)
    drone_s = np.interp(t, np.linspace(0.0, 1.0, len(dist)), dist)
    drone_z = np.interp(t, np.linspace(0.0, 1.0, len(Z_up)), Z_up)

    info_text = fig.text(0.5, 0.02, "", ha="center", fontsize=11, family="monospace",
                         bbox=dict(boxstyle="round", facecolor="white", alpha=0.9))

    def init():
        traveled1.set_data([], [])
        drone_pt1.set_data([], [])
        drone_pt2.set_data([], [])
        return traveled1, drone_pt1, drone_pt2, info_text

    def update(frame):
        dx, dy = drone_x[frame], drone_y[frame]
        ds, dz = drone_s[frame], drone_z[frame]

        traveled1.set_data(drone_x[:frame + 1], drone_y[:frame + 1])
        drone_pt1.set_data([dx], [dy])
        drone_pt2.set_data([ds], [dz])

        info_text.set_text(f"Postęp: {frame / total_frames * 100:.1f}% | s={ds:.1f} m | z={dz:.1f} m")
        return traveled1, drone_pt1, drone_pt2, info_text

    anim = animation.FuncAnimation(fig, update, init_func=init,
                                   frames=total_frames, interval=int(1000 / fps),
                                   blit=False, repeat=True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
