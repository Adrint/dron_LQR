"""
map_5_visualization.py

Moduł wizualizacji orientacji drona w 3D.
Lewy panel: wykres 3D drona.
Prawy panel: tabelka THRUST / ANGLES / RATES.
Przyciski sterowania: Stop/Wznów i Zakończ umieszczone bezpośrednio na wykresie
"""

import math
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from config import config
from control_rhs import rotation_matrix_zyx


def rotate_point(point, R):
    """Zastosuj macierz obrotu R do wektora 3D."""
    return R @ np.array(point)


def draw_angle_arc_3d(ax, center, radius, start_vec, end_vec, normal, color, label):
    """
    Rysuje łuk pokazujący kąt w 3D z poprawnym znakiem (kierunkiem obrotu).
    """
    start_vec = start_vec / np.linalg.norm(start_vec)
    end_vec = end_vec / np.linalg.norm(end_vec)
    normal = normal / np.linalg.norm(normal)

    # Magnituda kąta
    angle_magnitude = np.arccos(np.clip(np.dot(start_vec, end_vec), -1.0, 1.0))

    # Kierunek obrotu: (start x end) · normal
    cross_prod = np.cross(start_vec, end_vec)
    direction = np.dot(cross_prod, normal)
    if direction < 0:
        angle_magnitude = -angle_magnitude

    # Punkty łuku (Rodrigues)
    n_points = 20
    angles = np.linspace(0, angle_magnitude, n_points)
    arc_points = []
    for a in angles:
        v = (start_vec * np.cos(a)
             + np.cross(normal, start_vec) * np.sin(a)
             + normal * np.dot(normal, start_vec) * (1 - np.cos(a)))
        arc_points.append(center + radius * v)

    arc_points = np.array(arc_points)
    ax.plot(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
            color=color, linewidth=2, alpha=0.8)

    # Etykieta w połowie łuku
    if len(arc_points) > 0:
        mid_point = arc_points[len(arc_points) // 2]
        ax.text(mid_point[0], mid_point[1], mid_point[2], label,
                fontsize=11, color=color, weight='bold')


class DroneVisualizer:
    """
    Wizualizacja orientacji quadcoptera w 3D.

    Figura ma 1 wiersz i 2 kolumny:
    - ax3d   : lewa kolumna – wykres 3D drona
    - ax_panel: prawa kolumna – wycentrowana tabelka (THRUST / ANGLES / RATES)

    Przyciski sterowania symulacją umieszczone na dole okna
    """

    def __init__(self, simulation_controller=None):
        # Wymiary z configu
        self.body_length = config.body_length
        self.body_width = config.body_width
        self.body_height = config.body_height
        self.motor_arm = config.motor_arm

        # Ograniczenia osi (body frame wokół [0,0,0])
        self.x_boundary_minus = -self.motor_arm
        self.x_boundary_plus = +self.motor_arm
        self.y_boundary_minus = -self.motor_arm
        self.y_boundary_plus = +self.motor_arm
        self.z_boundary_minus = -self.motor_arm
        self.z_boundary_plus = +self.motor_arm

        self.fig = None
        self.ax3d = None      # lewy – 3D
        self.ax_panel = None  # prawy – tabelka

        # Przyciski sterowania
        self.pause_button = None
        self.stop_button = None
        self.status_text = None

        # Referencja do kontrolera symulacji
        self.sim_controller = simulation_controller

        self._init_plot()

    def _init_plot(self):
        """Inicjalizacja figury i dwóch osi (3D + panel tabelki) + przyciski."""
        plt.ion()
        self.fig = plt.figure(figsize=(8, 5.2))  # Zwiększona wysokość dla przycisków

        # Główny grid dla wykresów
        gs = self.fig.add_gridspec(2, 2,
                                   height_ratios=[9, 1],  # Główne wykresy + przyciski
                                   width_ratios=[2.0, 1.0],
                                   hspace=0.3, wspace=0.3)

        # Lewa kolumna – wykres 3D
        self.ax3d = self.fig.add_subplot(gs[0, 0], projection='3d')

        # Prawa kolumna – panel tekstowy (tabelka)
        self.ax_panel = self.fig.add_subplot(gs[0, 1])
        self.ax_panel.axis('off')        # brak osi
        self.ax_panel.set_xlim(0, 1)
        self.ax_panel.set_ylim(0, 1)

        # Ustawienia osi 3D
        self.ax3d.set_xlim(self.x_boundary_minus, self.x_boundary_plus)
        self.ax3d.set_ylim(self.y_boundary_minus, self.y_boundary_plus)
        self.ax3d.set_zlim(self.z_boundary_minus, self.z_boundary_plus)

        self.ax3d.set_xticks(np.arange(self.x_boundary_minus,
                                       self.x_boundary_plus + 0.001, 0.5))
        self.ax3d.set_yticks(np.arange(self.y_boundary_minus,
                                       self.y_boundary_plus + 0.001, 0.5))
        self.ax3d.set_zticks(np.arange(self.z_boundary_minus,
                                       self.z_boundary_plus + 0.001, 0.5))

        self.ax3d.set_xlabel('X [m]')
        self.ax3d.set_ylabel('Y [m]')
        self.ax3d.set_zlabel('Z [m]')
        self.ax3d.set_title('Quadcopter 3D Orientation')

        # Lekko skośny widok
        self.ax3d.view_init(elev=20, azim=-45)
        self.ax3d.invert_zaxis()

        # ========== PRZYCISKI STEROWANIA ==========

        # Tekst statusu (nad przyciskami)
        ax_status = self.fig.add_subplot(gs[1, :])
        ax_status.axis('off')
        self.status_text = ax_status.text(
            0.5, 0.75, 'Symulacja w toku...',
            ha='center', va='center',
            fontsize=11, fontweight='bold',
            transform=ax_status.transAxes
        )

        # Przycisk Stop/Wznów
        ax_pause = plt.axes([0.25, 0.02, 0.2, 0.05])  # [left, bottom, width, height]
        self.pause_button = Button(ax_pause, '⏸ Stop', color='lightgray', hovercolor='gray')
        self.pause_button.on_clicked(self._on_pause_clicked)

        # Przycisk Zakończ
        ax_stop = plt.axes([0.55, 0.02, 0.2, 0.05])
        self.stop_button = Button(ax_stop, '⏹ Zakończ', color='lightcoral', hovercolor='red')
        self.stop_button.on_clicked(self._on_stop_clicked)

        plt.tight_layout()
        plt.draw()
        plt.pause(0.1)

    def _on_pause_clicked(self, event):
        """Obsługa kliknięcia przycisku Stop/Wznów."""
        if self.sim_controller:
            self.sim_controller.toggle_pause()

            if self.sim_controller.is_paused():
                self.pause_button.label.set_text('▶ Wznów')
                self.status_text.set_text('⏸ Symulacja zatrzymana')
                self.status_text.set_color('orange')
            else:
                self.pause_button.label.set_text('⏸ Stop')
                self.status_text.set_text('▶ Symulacja wznowiona')
                self.status_text.set_color('green')

            # WAŻNE: Wymuś odświeżenie GUI
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def _on_stop_clicked(self, event):
        """Obsługa kliknięcia przycisku Zakończ."""
        if self.sim_controller:
            self.sim_controller.stop_simulation()
            self.status_text.set_text('⏹ Symulacja zakończona')
            self.status_text.set_color('red')

            # WAŻNE: Wymuś odświeżenie GUI
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def update_plots(self, x, t, i, T1, T2, T3, T4):
        """
        Aktualizacja orientacji drona + panelu z tabelką.

        Parameters
        ----------
        x : ndarray
            Wektor stanu (phi, theta, psi w x[9:12])
        t : float
            Czas (nieużywany, ale zostawiony dla kompatybilności)
        i : int
            Krok (nieużywany, ale zostawiony dla kompatybilności)
        T1..T4 : float
            Ciągi silników [N]
        """
        phi = x[9]
        theta = x[10]
        psi = x[11]

        phi_deg = math.degrees(phi)
        theta_deg = math.degrees(theta)
        psi_deg = math.degrees(psi)

        self._draw_drone_orientation(x, T1, T2, T3, T4, phi_deg, theta_deg, psi_deg)

        plt.draw()
        plt.pause(0.001)

    def _draw_drone_orientation(self, x, T1, T2, T3, T4, phi_deg, theta_deg, psi_deg):
        """
        Rysuje drona, osie oraz panel z tabelką (THRUST / ANGLES / RATES).
        Lewa kolumna – ax3d, prawa – ax_panel.
        """
        # Czyścimy obie osie (ale nie przyciski!)
        self.ax3d.clear()
        self.ax_panel.clear()
        self.ax_panel.axis('off')
        self.ax_panel.set_xlim(0, 1)
        self.ax_panel.set_ylim(0, 1)

        # ===== Lewy panel: wykres 3D =====
        self.ax3d.set_xlim(self.x_boundary_minus, self.x_boundary_plus)
        self.ax3d.set_ylim(self.y_boundary_minus, self.y_boundary_plus)
        self.ax3d.set_zlim(self.z_boundary_minus, self.z_boundary_plus)

        self.ax3d.set_xlabel('X [m]', fontsize=11, weight='bold')
        self.ax3d.set_ylabel('Y [m]', fontsize=11, weight='bold')
        self.ax3d.set_zlabel('Z [m]', fontsize=11, weight='bold')

        self.ax3d.set_xticks(np.arange(self.x_boundary_minus,
                                       self.x_boundary_plus + 0.001, 0.5))
        self.ax3d.set_yticks(np.arange(self.y_boundary_minus,
                                       self.y_boundary_plus + 0.001, 0.5))
        self.ax3d.set_zticks(np.arange(self.z_boundary_minus,
                                       self.z_boundary_plus + 0.001, 0.5))

        # Oś świata
        axis_len = self.x_boundary_plus + 0.3
        self.ax3d.plot([0, axis_len], [0, 0], [0, 0],
                       color='black', linewidth=3, alpha=0.7)
        self.ax3d.plot([0, 0], [0, axis_len], [0, 0],
                       color='black', linewidth=3, alpha=0.7)
        self.ax3d.plot([0, 0], [0, 0], [0, axis_len],
                       color='black', linewidth=3, alpha=0.7)

        self.ax3d.text(axis_len, 0, 0, 'X', fontsize=14,
                       color='black', weight='bold')
        self.ax3d.text(0, axis_len, 0, 'Y', fontsize=14,
                       color='black', weight='bold')
        self.ax3d.text(0, 0, axis_len, 'Z', fontsize=14,
                       color='black', weight='bold')

        # Pełna macierz obrotu body->world
        phi, theta, psi = x[9], x[10], x[11]
        R = rotation_matrix_zyx(phi, theta, psi)

        # Położenia silników w body frame (X-config)
        motor_arm = self.motor_arm
        angle_offset = np.pi / 4  # 45°
        motors_body = [
            np.array([motor_arm * np.cos(angle_offset),
                      motor_arm * np.sin(angle_offset), 0]),
            np.array([motor_arm * np.cos(angle_offset),
                      -motor_arm * np.sin(angle_offset), 0]),
            np.array([-motor_arm * np.cos(angle_offset),
                      motor_arm * np.sin(angle_offset), 0]),
            np.array([-motor_arm * np.cos(angle_offset),
                      -motor_arm * np.sin(angle_offset), 0])
        ]
        motors_world = [rotate_point(m, R) for m in motors_body]

        # Ramiona X
        self.ax3d.plot(
            [motors_world[0][0], motors_world[3][0]],
            [motors_world[0][1], motors_world[3][1]],
            [motors_world[0][2], motors_world[3][2]],
            'k-', linewidth=3, alpha=0.9
        )
        self.ax3d.plot(
            [motors_world[1][0], motors_world[2][0]],
            [motors_world[1][1], motors_world[2][1]],
            [motors_world[1][2], motors_world[2][2]],
            'k-', linewidth=3, alpha=0.9
        )

        # Korpus (prostopadłościan)
        rx = self.body_length / 2
        ry = self.body_width / 2
        rz = self.body_height / 2

        vertices_body = [
            np.array([-rx, -ry, -rz]), np.array([rx, -ry, -rz]),
            np.array([rx, ry, -rz]), np.array([-rx, ry, -rz]),
            np.array([-rx, -ry, rz]), np.array([rx, -ry, rz]),
            np.array([rx, ry, rz]), np.array([-rx, ry, rz])
        ]
        vertices_world = [rotate_point(v, R) for v in vertices_body]

        faces = [
            [vertices_world[0], vertices_world[1], vertices_world[2], vertices_world[3]],
            [vertices_world[4], vertices_world[5], vertices_world[6], vertices_world[7]],
            [vertices_world[0], vertices_world[1], vertices_world[5], vertices_world[4]],
            [vertices_world[2], vertices_world[3], vertices_world[7], vertices_world[6]],
            [vertices_world[0], vertices_world[3], vertices_world[7], vertices_world[4]],
            [vertices_world[1], vertices_world[2], vertices_world[6], vertices_world[5]]
        ]

        cube = Poly3DCollection(
            faces, alpha=0.8,
            facecolor='royalblue',
            edgecolor='navy',
            linewidth=2
        )
        self.ax3d.add_collection3d(cube)

        # Silniki
        thrusts = [T1, T2, T3, T4]
        motor_colors = ['red', 'blue', 'green', 'orange']

        for j, (m_pos, T) in enumerate(zip(motors_world, thrusts)):
            self.ax3d.scatter(
                [m_pos[0]], [m_pos[1]], [m_pos[2]],
                color=motor_colors[j],
                s=200,
                edgecolors='black',
                linewidths=2,
                depthshade=True,
                zorder=10
            )

        # Osie body (X', Y', Z')
        axis_body_len = self.motor_arm * 1.8

        # X_body (przód) – czerwony
        x_body_dir = rotate_point([axis_body_len, 0, 0], R)
        self.ax3d.quiver(
            0, 0, 0,
            x_body_dir[0], x_body_dir[1], x_body_dir[2],
            color='red',
            arrow_length_ratio=0.2,
            linewidth=3,
            linestyle='--',
            alpha=0.9
        )
        self.ax3d.text(
            x_body_dir[0] * 1.1, x_body_dir[1] * 1.1, x_body_dir[2] * 1.1,
            'X\'', fontsize=12, color='red', weight='bold'
        )

        # Y_body (prawo) – zielony
        y_body_dir = rotate_point([0, axis_body_len, 0], R)
        self.ax3d.quiver(
            0, 0, 0,
            y_body_dir[0], y_body_dir[1], y_body_dir[2],
            color='green',
            arrow_length_ratio=0.2,
            linewidth=3,
            linestyle='--',
            alpha=0.9
        )
        self.ax3d.text(
            y_body_dir[0] * 1.1, y_body_dir[1] * 1.1, y_body_dir[2] * 1.1,
            'Y\'', fontsize=12, color='green', weight='bold'
        )

        # Z_body (dół) – niebieski
        z_body_dir = rotate_point([0, 0, axis_body_len], R)
        self.ax3d.quiver(
            0, 0, 0,
            z_body_dir[0], z_body_dir[1], z_body_dir[2],
            color='blue',
            arrow_length_ratio=0.2,
            linewidth=3,
            linestyle='--',
            alpha=0.9
        )
        self.ax3d.text(
            z_body_dir[0] * 1.1, z_body_dir[1] * 1.1, z_body_dir[2] * 1.1,
            'Z\'', fontsize=12, color='blue', weight='bold'
        )

        # Łuki kątów (opcjonalne – przy większych kątach)
        angle_radius = axis_body_len

        if abs(theta_deg) > 1:
            draw_angle_arc_3d(
                self.ax3d, np.array([0, 0, 0]), angle_radius,
                np.array([1, 0, 0]),
                rotate_point([1, 0, 0], R),
                np.array([0, 1, 0]),
                'green', f'θ'
            )

        if abs(phi_deg) > 1:
            draw_angle_arc_3d(
                self.ax3d, np.array([0, 0, 0]), angle_radius,
                np.array([0, 1, 0]),
                rotate_point([0, 1, 0], R),
                np.array([1, 0, 0]),
                'red', f'φ'
            )

        if abs(psi_deg) > 1:
            draw_angle_arc_3d(
                self.ax3d, np.array([0, 0, 0]), angle_radius,
                np.array([1, 0, 0]),
                rotate_point([1, 0, 0], R),
                np.array([0, 0, 1]),
                'blue', f'ψ'
            )

        # ===== Prawy panel: tabelka jako legenda na osobnym ax =====
        legend_elements = [
            # === THRUST ===
            Line2D([0], [0], color='none', label='╔═══ THRUST ════╗'),
            Patch(facecolor=motor_colors[0], edgecolor='black', label=f'║ M1:    {T1:5.1f}N ║'),
            Patch(facecolor=motor_colors[1], edgecolor='black', label=f'║ M2:    {T2:5.1f}N ║'),
            Patch(facecolor=motor_colors[2], edgecolor='black', label=f'║ M3:    {T3:5.1f}N ║'),
            Patch(facecolor=motor_colors[3], edgecolor='black', label=f'║ M4:    {T4:5.1f}N ║'),
            # === ANGLES ===
            Line2D([0], [0], color='none', label='╠═══ ANGLES ════╣'),
            Line2D([0], [0], color='none', label=f'║ φ:  {phi_deg:7.2f}°  ║'),
            Line2D([0], [0], color='none', label=f'║ θ:  {theta_deg:7.2f}°  ║'),
            Line2D([0], [0], color='none', label=f'║ ψ:  {psi_deg:7.2f}°  ║'),
            # === RATES ===
            Line2D([0], [0], color='none', label='╠═══ RATES ═════╣'),
            Line2D([0], [0], color='none', label=f'║ p: {math.degrees(x[3]):7.2f}°/s ║'),
            Line2D([0], [0], color='none', label=f'║ q: {math.degrees(x[4]):7.2f}°/s ║'),
            Line2D([0], [0], color='none', label=f'║ r: {math.degrees(x[5]):7.2f}°/s ║'),
            Line2D([0], [0], color='none', label='╚═══════════════╝')
        ]

        # Legenda wycentrowana w ax_panel – zostawiamy domyślny handlelength,
        # żeby kolorowe kwadraty Patch (M1–M4) były dobrze widoczne.
        self.ax_panel.legend(
            handles=legend_elements,
            loc='center',
            fontsize=9,
            framealpha=0.95,
            edgecolor='black',
            fancybox=True,
            shadow=True,
            borderaxespad=0,
            prop={'family': 'monospace', 'size': 9}
        )

        self.ax3d.view_init(elev=20, azim=-45)
        self.ax3d.invert_zaxis()

        self.fig.tight_layout()

    def close(self):
        """Zakończenie trybu interaktywnego."""
        plt.ioff()
        plt.show()
