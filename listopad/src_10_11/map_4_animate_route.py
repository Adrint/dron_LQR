"""
Moduł wizualizacji dla symulatora drona
Jedna figura 2x2:
  • ax1 (YX – top view) + ax2 (ZX – profil) - z animacjami budynków jak w map_vis.py
  • ax3 (trajektoria 3D) + ax4 (orientacja 3D) - bez zmian

Zachowuje kompatybilny interfejs: DroneVisualizer.update_plots(...)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from config import config
from control_rhs import rotation_matrix_zyx


# ===== helpers (kontur drona 2D) =====
def aa_mdl(X, Y, teta, c):
    xs0 = np.array([-0.1 * c, +0.1 * c, +0.1 * c, -0.1 * c, -0.1 * c])
    ys0 = np.array([-0.1 * c, -0.1 * c, +0.1 * c, +0.1 * c, -0.1 * c])

    xs = np.zeros(5)
    ys = np.zeros(5)

    ct = np.cos(teta);
    st = np.sin(teta)
    for i in range(5):
        xs[i] = X + xs0[i] * ct - ys0[i] * st
        ys[i] = Y + xs0[i] * st + ys0[i] * ct

    return xs, ys


class DroneVisualizer:
    """
    Jedna figura 2x2:
      • ax1 (YX – widok z góry) + ax2 (ZX – profil) - z animacjami budynków
      • ax3 (trajektoria 3D) + ax4 (orientacja 3D)

    Interfejs update_plots pozostaje zgodny z resztą systemu.
    """

    def __init__(self, X_ref_all, Y_terr_all, Z_terr_all, Y_ref_all, Z_ref_all,
                 buildings_data=None):
        """
        buildings_data (optional): słownik z:
            'shapes': lista dict z 'x', 'y', 'height' dla ax1 (widok z góry)
            'profile': lista dict z 'x_min', 'x_max', 'height' dla ax2 (profil)
        """
        # referencje (NED)
        self.X_ref_all = np.asarray(X_ref_all, dtype=float)
        self.Y_terr_all = np.asarray(Y_terr_all, dtype=float)
        self.Z_terr_all = np.asarray(Z_terr_all, dtype=float)
        self.Y_ref_all = np.asarray(Y_ref_all, dtype=float)
        self.Z_ref_all = np.asarray(Z_ref_all, dtype=float)

        # wymiary drona
        self.body_length = config.body_length
        self.body_width = config.body_width
        self.body_height = config.body_height
        self.motor_arm = config.motor_arm

        # przechowywanie artystów
        self._artists = {}

        # dane budynków (opcjonalne)
        self.buildings_data = buildings_data
        self.building_shapes = []
        self.building_profile = []

        if buildings_data:
            self.building_shapes = buildings_data.get('shapes', [])
            self.building_profile = buildings_data.get('profile', [])

        plt.ion()
        self._init_figures()
        self._init_ax1()
        self._init_ax2()
        self._init_ax3()
        self._init_ax4()
        plt.pause(0.001)

    # ---------- tworzenie figur (JEDNA FIGURA 2x2)
    def _init_figures(self):
        """Jedna figura z gridem 2x2"""
        self.fig = plt.figure(figsize=(18, 14))
        gs = self.fig.add_gridspec(2, 2, wspace=0.25, hspace=0.3)

        self.ax1 = self.fig.add_subplot(gs[0, 0])  # YX (top view)
        self.ax2 = self.fig.add_subplot(gs[0, 1])  # ZX (profil)
        self.ax3 = self.fig.add_subplot(gs[1, 0], projection='3d')  # 3D trajectory
        self.ax4 = self.fig.add_subplot(gs[1, 1], projection='3d')  # 3D orientation

    # ---------- AX1 YX (z animacją budynków)
    def _init_ax1(self):
        ax = self.ax1
        ax.set_aspect("equal")
        ax.set_facecolor("#f0f0f0")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.set_xlabel("X [m]", fontsize=11, fontweight='bold')
        ax.set_ylabel("Y [m]", fontsize=11, fontweight='bold')
        ax.set_title("Widok z góry (XY) – animacja", fontweight="bold", fontsize=13)

        # Referencyjna trasa (półprzezroczysta)
        ax.plot(self.X_ref_all, self.Y_ref_all, "r-", lw=1.6, alpha=0.35, label="Trasa referencyjna")

        # Artyści do animacji
        self._artists["ax1_traveled"], = ax.plot([], [], "g-", lw=3, alpha=0.8, label="Przebyta trasa")
        self._artists["ax1_future"], = ax.plot([], [], "b-", lw=2, alpha=0.55, label="Nadchodząca trasa")
        self._artists["ax1_drone"], = ax.plot([], [], "ro", ms=12, mec="black", mew=2, zorder=1000, label="Dron")
        self._artists["ax1_contour"], = ax.plot([], [], "k-", lw=2)

        ax.legend(fontsize=10, loc="upper right", framealpha=0.9)

    # ---------- AX2 ZX (z budynkami na profilu)
    def _init_ax2(self):
        ax = self.ax2
        ax.set_facecolor("#f9f9f9")
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.set_xlabel("X [m]", fontsize=11, fontweight='bold')
        ax.set_ylabel("Z [m NED]", fontsize=11, fontweight='bold')
        ax.set_title("Profil wysokościowy (XZ) – animacja", fontweight="bold", fontsize=13)

        # Referencja i teren
        ax.plot(self.X_ref_all, self.Z_ref_all, "r-", lw=2.5, alpha=0.8, label="Trasa referencyjna (NED)")
        ax.plot(self.X_ref_all, self.Z_terr_all, "g-", lw=2, alpha=0.7, label="Teren (NED)")

        # Budynki na profilu (statyczne prostokąty)
        if self.building_profile:
            print(f"  [AX2] Rysowanie {len(self.building_profile)} budynków na profilu...")
            for bldg in self.building_profile:
                x_min = bldg['x_min']
                x_max = bldg['x_max']
                height = bldg['height']
                width = x_max - x_min

                # NED: budynek od 0 do -height (bo ujemne Z to w górę)
                rect = patches.Rectangle(
                    (x_min, 0),
                    width, -height,  # ujemna wysokość bo NED
                    facecolor='orange',
                    edgecolor='darkgray',
                    alpha=0.6,
                    linewidth=1,
                    zorder=50
                )
                ax.add_patch(rect)

        # Artyści do animacji
        self._artists["ax2_path"], = ax.plot([], [], "b-", lw=2.5, alpha=0.9, label="Tor lotu (NED)")
        self._artists["ax2_drone"], = ax.plot([], [], "ro", ms=10, mec="black", mew=1.5, zorder=200, label="Dron")
        self._artists["ax2_contour"], = ax.plot([], [], "k-", lw=2)

        ax.invert_yaxis()
        ax.legend(fontsize=10, loc="upper right", framealpha=0.9)

        # Pasek informacji
        self._artists["info_text"] = self.fig.text(
            0.5, 0.48, "", ha="center", fontsize=11, family="monospace",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.95, edgecolor='black', linewidth=1.5)
        )

    # ---------- AX3 3D traj
    def _init_ax3(self):
        ax = self.ax3
        ax.plot(self.X_ref_all, self.Y_ref_all, self.Z_ref_all, "r-", lw=2, alpha=0.7, label="Referencja")
        ax.plot(self.X_ref_all, self.Y_terr_all, self.Z_terr_all, "g-", lw=1.5, alpha=0.7, label="Teren")
        self._artists["ax3_path"], = ax.plot([], [], [], "b-", lw=2.5, alpha=0.9, label="Tor lotu")
        self._artists["ax3_pt"] = ax.scatter([], [], [], color="red", s=60, edgecolors='black', linewidths=1.5)
        ax.set_xlabel("X [m]", fontsize=11, fontweight='bold')
        ax.set_ylabel("Y [m]", fontsize=11, fontweight='bold')
        ax.set_zlabel("Z [m NED]", fontsize=11, fontweight='bold')
        ax.set_title("Trajektoria 3D", fontweight='bold', fontsize=13)
        ax.legend(fontsize=10, framealpha=0.9)
        ax.grid(True, alpha=0.25)
        ax.invert_zaxis()

    # ---------- AX4 orientacja
    def _init_ax4(self):
        self.ax4.set_title("Orientacja 3D Quadcoptera", fontweight='bold', fontsize=13)
        self.ax4.invert_zaxis()

    # ---------- kontury
    def _contour_xy(self, X_pos, Y_pos, roll_phi):
        xsy, ys = aa_mdl(X_pos, Y_pos, -roll_phi, 5)
        return xsy[:5], ys[:5]

    def _contour_xz(self, X_pos, Z_pos, pitch_theta):
        xs, zs = aa_mdl(X_pos, Z_pos, -pitch_theta, 5)
        return xs[:5], zs[:5]

    # ---------- PUBLIC: update (wywoływane z pętli symulacji)
    def update_plots(self, x, t, i, yp_array, T1, T2, T3, T4, limits_exceeded):
        X_pos, Y_pos, Z_pos = float(x[6]), float(x[7]), float(x[8])
        phi, theta = float(x[9]), float(x[10])

        # ========== AX1 (XY - widok z góry z budynkami) ==========
        self._artists["ax1_traveled"].set_data(yp_array[:i + 1, 6], yp_array[:i + 1, 7])

        # Look-ahead (80m)
        look_ahead_m = 80.0
        if len(self.X_ref_all) > 1:
            step_m = np.mean(np.diff(self.X_ref_all))
            step_m = max(step_m, 1.0)
        else:
            step_m = 1.0
        n_look = int(look_ahead_m / step_m)
        j1 = max(0, i)
        j2 = min(len(self.X_ref_all) - 1, i + n_look)
        self._artists["ax1_future"].set_data(self.X_ref_all[j1:j2], self.Y_ref_all[j1:j2])

        self._artists["ax1_drone"].set_data([X_pos], [Y_pos])

        cx, cy = self._contour_xy(X_pos, Y_pos, phi)
        self._artists["ax1_contour"].set_data(cx, cy)

        # Usuń stare budynki (patch'e)
        for patch in self.ax1.patches[:]:
            patch.remove()

        # Rysuj budynki w zasięgu (jak w map_vis.py)
        view_range = 100.0
        if self.building_shapes:
            for bldg in self.building_shapes:
                bldg_x = np.array(bldg['x'])
                bldg_y = np.array(bldg['y'])

                # Sprawdź odległość środka budynku od drona
                dist_to_drone = np.hypot(np.mean(bldg_x) - X_pos, np.mean(bldg_y) - Y_pos)

                # Rysuj tylko budynki w zasięgu view_range * 1.5
                if dist_to_drone < view_range * 1.5:
                    poly = plt.Polygon(list(zip(bldg_x, bldg_y)),
                                       facecolor='#808080',
                                       edgecolor='#404040',
                                       alpha=0.7,
                                       linewidth=1,
                                       zorder=50)
                    self.ax1.add_patch(poly)

        # Okno podążające
        self.ax1.set_xlim(X_pos - view_range, X_pos + view_range)
        self.ax1.set_ylim(Y_pos - view_range, Y_pos + view_range)

        # ========== AX2 (XZ - profil) ==========
        self._artists["ax2_path"].set_data(yp_array[:i + 1, 6], yp_array[:i + 1, 8])
        self._artists["ax2_drone"].set_data([X_pos], [Z_pos])

        cx2, cz2 = self._contour_xz(X_pos, Z_pos, theta)
        self._artists["ax2_contour"].set_data(cx2, cz2)

        # Okno podążające
        view_half_width = 80.0
        x_min = max(0, X_pos - view_half_width)
        x_max = min(self.X_ref_all[-1] if len(self.X_ref_all) else X_pos + view_half_width,
                    X_pos + view_half_width)
        if x_max <= x_min:
            x_max = x_min + 1.0
        self.ax2.set_xlim(x_min, x_max)

        # Pasek informacji
        progress = 100.0 * i / max(1, len(self.X_ref_all) - 1)
        self._artists["info_text"].set_text(
            f"Postęp: {progress:5.1f}% | X: {X_pos:7.1f} m | Z(NED): {Z_pos:7.1f} m | AGL: {-Z_pos:6.1f} m"
        )

        # ========== AX3 (Trajektoria 3D) ==========
        self._artists["ax3_path"].set_data_3d(yp_array[:i + 1, 6], yp_array[:i + 1, 7], yp_array[:i + 1, 8])
        self._artists["ax3_pt"]._offsets3d = ([X_pos], [Y_pos], [Z_pos])

        # ========== AX4 (Orientacja 3D) ==========
        self._draw_drone_orientation(x, T1, T2, T3, T4)

        # Odśwież całą figurę
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    # ---------- rysowanie orientacji (identyczne jak w visualization.py)
    def _draw_drone_orientation(self, x, T1, T2, T3, T4):
        ax = self.ax4
        ax.clear()

        xbm = np.floor(-self.motor_arm);
        xbp = np.ceil(self.motor_arm)
        ybm = np.floor(-self.motor_arm);
        ybp = np.ceil(self.motor_arm)
        zbm = np.floor(-self.motor_arm);
        zbp = np.ceil(self.motor_arm)
        ax.set_xlim(xbm, xbp);
        ax.set_ylim(ybm, ybp);
        ax.set_zlim(zbm, zbp)

        ax.set_xlabel('X [m]', fontsize=11, weight='bold')
        ax.set_ylabel('Y [m]', fontsize=11, weight='bold')
        ax.set_zlabel('Z [m]', fontsize=11, weight='bold')
        ax.set_title('Orientacja 3D Quadcoptera', fontweight='bold', fontsize=13)
        ax.invert_zaxis()

        phi, theta, psi = float(x[9]), float(x[10]), float(x[11])
        R = rotation_matrix_zyx(phi, theta, psi)

        # Korpus (sześcian)
        rx = self.body_length / 2;
        ry = self.body_width / 2;
        rz = self.body_height / 2
        vertices_body = [
            np.array([-rx, -ry, -rz]), np.array([rx, -ry, -rz]),
            np.array([rx, ry, -rz]), np.array([-rx, ry, -rz]),
            np.array([-rx, -ry, rz]), np.array([rx, -ry, rz]),
            np.array([rx, ry, rz]), np.array([-rx, ry, rz])
        ]
        vertices_world = [R @ v for v in vertices_body]
        faces = [
            [vertices_world[0], vertices_world[1], vertices_world[2], vertices_world[3]],
            [vertices_world[4], vertices_world[5], vertices_world[6], vertices_world[7]],
            [vertices_world[0], vertices_world[1], vertices_world[5], vertices_world[4]],
            [vertices_world[2], vertices_world[3], vertices_world[7], vertices_world[6]],
            [vertices_world[0], vertices_world[3], vertices_world[7], vertices_world[4]],
            [vertices_world[1], vertices_world[2], vertices_world[6], vertices_world[5]]
        ]
        cube = Poly3DCollection(faces, alpha=0.8, facecolor='royalblue', edgecolor='navy', linewidth=2)
        ax.add_collection3d(cube)

        # Ramiona X
        ang = np.pi / 4
        motors_body = [
            np.array([self.motor_arm * np.cos(ang), self.motor_arm * np.sin(ang), 0]),
            np.array([self.motor_arm * np.cos(ang), -self.motor_arm * np.sin(ang), 0]),
            np.array([-self.motor_arm * np.cos(ang), self.motor_arm * np.sin(ang), 0]),
            np.array([-self.motor_arm * np.cos(ang), -self.motor_arm * np.sin(ang), 0]),
        ]
        motors_world = [R @ m for m in motors_body]
        ax.plot([motors_world[0][0], motors_world[3][0]],
                [motors_world[0][1], motors_world[3][1]],
                [motors_world[0][2], motors_world[3][2]], 'k-', lw=3, alpha=0.9)
        ax.plot([motors_world[1][0], motors_world[2][0]],
                [motors_world[1][1], motors_world[2][1]],
                [motors_world[1][2], motors_world[2][2]], 'k-', lw=3, alpha=0.9)

        # Silniki
        colors = ['red', 'blue', 'green', 'orange']
        for j, m in enumerate(motors_world):
            ax.scatter([m[0]], [m[1]], [m[2]],
                       color=colors[j], s=200,
                       edgecolors='black', linewidths=2, depthshade=True, zorder=10)

        # Osie ciała
        axis_len = self.motor_arm * 1.6
        xb = R @ np.array([axis_len, 0, 0])
        yb = R @ np.array([0, axis_len, 0])
        zb = R @ np.array([0, 0, axis_len])
        ax.quiver(0, 0, 0, xb[0], xb[1], xb[2], color='red', linewidth=3, alpha=0.9)
        ax.quiver(0, 0, 0, yb[0], yb[1], yb[2], color='green', linewidth=3, alpha=0.9)
        ax.quiver(0, 0, 0, zb[0], zb[1], zb[2], color='blue', linewidth=3, alpha=0.9)

        # Legenda
        legend_elements = [
            Line2D([0], [0], color='none', label='═══ OSIE ═══'),
            Line2D([0], [0], color='red', lw=2, label="X' (ciało)"),
            Line2D([0], [0], color='green', lw=2, label="Y' (ciało)"),
            Line2D([0], [0], color='blue', lw=2, label="Z' (ciało)"),
        ]

        ax.legend(handles=legend_elements,
                  loc='upper left',
                  bbox_to_anchor=(1.02, 1.0),
                  fontsize=9,
                  framealpha=0.95,
                  edgecolor='black',
                  fancybox=True,
                  shadow=True,
                  borderaxespad=0,
                  prop={'family': 'monospace', 'size': 9})

    def close(self):
        plt.ioff()
        plt.show()