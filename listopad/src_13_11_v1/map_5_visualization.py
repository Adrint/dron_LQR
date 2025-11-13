"""
Moduł wizualizacji dla symulatora drona
Zawiera klasę DroneVisualizer do obsługi wszystkich wykresów
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from config import config
from geo_utils import estimate_building_height
from control_rhs import rotation_matrix_zyx


def aa_mdl(X, Y, teta, c):
    """
    Calculate drone model coordinates for visualization

    Parameters:
    -----------
    X : float
        X position of drone
    Y : float
        Y position of drone
    teta : float
        Pitch angle (radians)
    c : float
        Scale factor

    Returns:
    --------
    xs : ndarray
        X coordinates of drone outline
    ys : ndarray
        Y coordinates of drone outline
    """
    xs0 = np.array([-0.1 * c, +0.1 * c, +0.1 * c, -0.1 * c, -0.1 * c])
    ys0 = np.array([-0.1 * c, -0.1 * c, +0.1 * c, +0.1 * c, -0.1 * c])

    xs = np.zeros(5)
    ys = np.zeros(5)

    for i in range(5):
        xs[i] = X + xs0[i] * np.cos(teta) - ys0[i] * np.sin(teta)
        ys[i] = Y + xs0[i] * np.sin(teta) + ys0[i] * np.cos(teta)

    return xs, ys


def rotate_point(point, R):
    """Apply rotation matrix to a 3D point."""
    return R @ np.array(point)


def draw_angle_arc_3d(ax, center, radius, start_vec, end_vec, normal, color, label):
    """Draw an arc showing an angle in 3D space with proper sign handling."""
    # Normalize vectors
    start_vec = start_vec / np.linalg.norm(start_vec)
    end_vec = end_vec / np.linalg.norm(end_vec)
    normal = normal / np.linalg.norm(normal)

    # Calculate angle magnitude (always positive from arccos)
    angle_magnitude = np.arccos(np.clip(np.dot(start_vec, end_vec), -1.0, 1.0))

    # Determine the sign of rotation using cross product
    # cross(start, end) · normal tells us the direction
    cross_prod = np.cross(start_vec, end_vec)
    direction = np.dot(cross_prod, normal)

    # If direction is negative, we rotate in opposite direction
    # This means we go the "long way" around the circle
    if direction < 0:
        angle_magnitude = -angle_magnitude

    # Generate arc points
    n_points = 20
    angles = np.linspace(0, angle_magnitude, n_points)

    # Create arc in local plane using Rodrigues' rotation formula
    arc_points = []
    for a in angles:
        v = start_vec * np.cos(a) + np.cross(normal, start_vec) * np.sin(a) + \
            normal * np.dot(normal, start_vec) * (1 - np.cos(a))
        arc_points.append(center + radius * v)

    arc_points = np.array(arc_points)
    ax.plot(arc_points[:, 0], arc_points[:, 1], arc_points[:, 2],
            color=color, linewidth=2, alpha=0.8)

    # Add label at mid-arc
    if len(arc_points) > 0:
        mid_point = arc_points[len(arc_points) // 2]
        ax.text(mid_point[0], mid_point[1], mid_point[2], label,
                fontsize=11, color=color, weight='bold')


# ============================================================================
# DRONE VISUALIZER CLASS
# ============================================================================

class DroneVisualizer:
    """
    Klasa obsługująca wszystkie wykresy symulatora drona
    """

    def __init__(self, s_ref_all, X_ref_all, Y_terr_all, Z_terr_all, Y_ref_all, Z_ref_all):
        """
        Inicjalizacja wizualizatora

        Parameters:
        -----------
        s_ref_all : array
            Dystans wzdłuż trasy (oś X na wykresach)
        X_ref_all : array
            Współrzędna X (dla wykresów 3D)
        Y_terr_all, Z_terr_all, Y_ref_all, Z_ref_all : array
            Referencyjne trajektorie
        """

        self.body_length = config.body_length
        self.body_width = config.body_width
        self.body_height = config.body_height
        self.motor_arm = config.motor_arm

        self.s_ref_all = s_ref_all  # Dystans zamiast X_ref_all
        self.X_ref_all = X_ref_all  # Współrzędna X dla wykresów 3D (zostawiona, jeśli gdzieś potrzebna)
        self.Y_terr_all = Y_terr_all
        self.Z_terr_all = Z_terr_all
        self.Y_ref_all = Y_ref_all
        self.Z_ref_all = Z_ref_all

        # Calculate boundaries
        self.x_boundary_minus = np.floor(-self.motor_arm)
        self.x_boundary_plus = np.ceil(self.motor_arm)
        self.y_boundary_minus = np.floor(-self.motor_arm)
        self.y_boundary_plus = np.ceil(self.motor_arm)
        self.z_boundary_minus = np.floor(-self.motor_arm)
        self.z_boundary_plus = np.ceil(self.motor_arm)

        # Initialize plots
        self._init_plots()

    def _init_plots(self):
        """Inicjalizacja figur matplotlib"""
        plt.ion()
        self.fig = plt.figure(1, figsize=(14, 12))

        # ===================================================================
        # Subplot 1: Trajektoria 3D – oś X = dystans s
        # ===================================================================
        self.ax1 = plt.subplot(1, 2, 1, projection='3d')

        # Dystans referencyjny – wymuszenie nieujemnych wartości
        s_ref_nonneg = np.maximum(0.0, np.asarray(self.s_ref_all))
        self.ax1.plot(s_ref_nonneg, self.Y_ref_all, self.Z_ref_all, 'r-',
                      label='Reference', linewidth=2)
        self.ax1.set_xlabel('Dystans s [m]')
        self.ax1.set_ylabel('Y [m]')
        self.ax1.set_zlabel('Z [m] (NED)')
        self.ax1.set_title('Trajektoria 3D (s, Y, Z)', fontsize=10)
        self.ax1.legend(fontsize=10)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.invert_zaxis()

        # ===================================================================
        # Subplot 2: Drone Orientation (prawy) - 3D VIEW - BEZ ZMIAN
        # ===================================================================
        self.ax2 = plt.subplot(1, 2, 2, projection='3d')
        self.ax2.set_xlim(self.x_boundary_minus, self.x_boundary_plus)
        self.ax2.set_ylim(self.y_boundary_minus, self.y_boundary_plus)
        self.ax2.set_zlim(self.z_boundary_minus, self.z_boundary_plus)
        self.ax2.set_xticks(np.arange(self.x_boundary_minus, self.x_boundary_plus, 0.5))
        self.ax2.set_yticks(np.arange(self.y_boundary_minus, self.y_boundary_plus, 0.5))
        self.ax2.set_zticks(np.arange(self.z_boundary_minus, self.z_boundary_plus, 0.5))
        self.ax2.set_xlabel('X [m]')
        self.ax2.set_ylabel('Y [m]')
        self.ax2.set_zlabel('Z [m]')
        self.ax2.set_title('Quadcopter 3D Orientation (Full Rotation)')
        self.ax2.view_init(elev=20, azim=-45)
        self.ax2.invert_zaxis()

        plt.tight_layout()
        plt.draw()
        plt.pause(1)

        print(f"X boundary: [{self.x_boundary_minus}, {self.x_boundary_plus}]")
        print(f"Y boundary: [{self.y_boundary_minus}, {self.y_boundary_plus}]")

    def update_plots(self, x, t, i, yp_array, T1, T2, T3, T4, limits_exceeded):
        """
        Aktualizacja wszystkich wykresów

        Parameters:
        -----------
        x : array
            Wektor stanu drona
        t : float
            Aktualny czas
        i : int
            Indeks iteracji
        yp_array : array
            Historia stanów
        T1, T2, T3, T4 : float
            Ciągi silników
        limits_exceeded : bool
            Czy przekroczono limity
        """
        # Extract state
        # X_pos globalny – jeśli byłby używany do wykresów, pilnujemy nieujemności
        X_pos = max(0.0, x[6])   # X nie może być ujemne na wykresach odległości
        Y_pos = x[7]
        Z_pos = x[8]
        phi = x[9]
        theta = x[10]
        psi = x[11]

        V = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
        phi_deg = math.degrees(phi)
        theta_deg = math.degrees(theta)
        psi_deg = math.degrees(psi)

        # Get drone model coordinates (aktualnie nieużywane w rysowaniu, ale zostawione)
        xs, zs = aa_mdl(X_pos, Z_pos, -theta, 5)
        xsy, ys = aa_mdl(X_pos, Y_pos, -phi, 5)
        alt = -Z_pos

        # ====================================================================
        # UPDATE SUBPLOT 1: TRAJEKTORIA 3D – oś X = dystans
        # ====================================================================
        self.ax1.clear()

        # Referencyjna trajektoria vs dystans (gwarancja nieujemnych s)
        s_ref_nonneg = np.maximum(0.0, np.asarray(self.s_ref_all))
        self.ax1.plot(s_ref_nonneg, self.Y_ref_all, self.Z_ref_all, 'r-',
                      label='Reference', linewidth=2)

        # Ścieżka drona – dystans wyliczany z historii pozycji (zawsze >= 0)
        if yp_array.shape[0] > 1:
            # używamy XY do liczenia dystansu w rzucie na ziemię
            xy_positions = yp_array[:, 6:8]  # kolumny X, Y
            deltas = np.diff(xy_positions, axis=0)
            segment_lengths = np.linalg.norm(deltas, axis=1)
            s_path = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        else:
            s_path = np.array([0.0])

        # bezpiecznie ucinamy s_path do długości yp_array (na wszelki wypadek)
        if s_path.shape[0] > yp_array.shape[0]:
            s_path = s_path[:yp_array.shape[0]]

        self.ax1.plot(s_path, yp_array[:, 7], yp_array[:, 8], 'b-',
                      label='Ścieżka drona', alpha=0.7, linewidth=2)

        # Aktualny punkt drona na trajektorii
        idx = min(i, len(s_path) - 1)
        self.ax1.scatter([s_path[idx]], [yp_array[idx, 7]], [yp_array[idx, 8]],
                         color='blue', s=100)

        self.ax1.set_xlabel('Dystans s [m]')
        self.ax1.set_ylabel('Y [m]')
        self.ax1.set_zlabel('Z [m] (NED)')
        self.ax1.set_title('Trajektoria 3D (s, Y, Z)', fontsize=10)
        self.ax1.legend(fontsize=9)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.invert_zaxis()

        # ====================================================================
        # UPDATE SUBPLOT 2: QUADCOPTER 3D ORIENTATION
        # ====================================================================
        self._draw_drone_orientation(x, T1, T2, T3, T4, phi_deg, theta_deg, psi_deg)

        plt.draw()
        plt.pause(0.001)

    def _draw_drone_orientation(self, x, T1, T2, T3, T4, phi_deg, theta_deg, psi_deg):
        """Rysowanie orientacji drona w 3D"""
        self.ax2.clear()
        self.ax2.set_xlim(self.x_boundary_minus, self.x_boundary_plus)
        self.ax2.set_ylim(self.y_boundary_minus, self.y_boundary_plus)
        self.ax2.set_zlim(self.z_boundary_minus, self.z_boundary_plus)

        self.ax2.set_xlabel('X [m]', fontsize=11, weight='bold')
        self.ax2.set_ylabel('Y [m]', fontsize=11, weight='bold')
        self.ax2.set_zlabel('Z [m]', fontsize=11, weight='bold')

        self.ax2.set_xticks(np.arange(self.x_boundary_minus, self.x_boundary_plus, 0.5))
        self.ax2.set_yticks(np.arange(self.y_boundary_minus, self.y_boundary_plus, 0.5))
        self.ax2.set_zticks(np.arange(self.z_boundary_minus, self.z_boundary_plus, 0.5))

        # Draw world coordinate axes
        axis_len = self.x_boundary_plus + 0.5
        self.ax2.plot([0, axis_len], [0, 0], [0, 0], color='black', linewidth=3, alpha=0.7, label='X (world)')
        self.ax2.plot([0, 0], [0, axis_len], [0, 0], color='black', linewidth=3, alpha=0.7, label='Y (world)')
        self.ax2.plot([0, 0], [0, 0], [0, axis_len], color='black', linewidth=3, alpha=0.7, label='Z (world)')

        self.ax2.text(axis_len, 0, 0, 'X', fontsize=14, color='black', weight='bold')
        self.ax2.text(0, axis_len, 0, 'Y', fontsize=14, color='black', weight='bold')
        self.ax2.text(0, 0, axis_len, 'Z', fontsize=14, color='black', weight='bold')

        # Get full rotation matrix
        phi, theta, psi = x[9], x[10], x[11]
        R = rotation_matrix_zyx(phi, theta, psi)

        # Drone dimensions
        motor_arm = self.motor_arm

        # Motor positions in body frame (X configuration)
        angle_offset = np.pi / 4  # 45 degrees
        motors_body = [
            np.array([motor_arm * np.cos(angle_offset), motor_arm * np.sin(angle_offset), 0]),
            np.array([motor_arm * np.cos(angle_offset), -motor_arm * np.sin(angle_offset), 0]),
            np.array([-motor_arm * np.cos(angle_offset), motor_arm * np.sin(angle_offset), 0]),
            np.array([-motor_arm * np.cos(angle_offset), -motor_arm * np.sin(angle_offset), 0])
        ]

        # Rotate motors to world frame
        motors_world = [rotate_point(m, R) for m in motors_body]

        # Draw drone arms (X shape)
        self.ax2.plot([motors_world[0][0], motors_world[3][0]],
                      [motors_world[0][1], motors_world[3][1]],
                      [motors_world[0][2], motors_world[3][2]],
                      'k-', linewidth=3, alpha=0.9)
        self.ax2.plot([motors_world[1][0], motors_world[2][0]],
                      [motors_world[1][1], motors_world[2][1]],
                      [motors_world[1][2], motors_world[2][2]],
                      'k-', linewidth=3, alpha=0.9)

        # Draw center body (cube with real dimensions)
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

        cube = Poly3DCollection(faces, alpha=0.8, facecolor='royalblue', edgecolor='navy', linewidth=2)
        self.ax2.add_collection3d(cube)

        # Draw motors
        thrusts = [T1, T2, T3, T4]
        motor_colors = ['red', 'blue', 'green', 'orange']

        for j, (m_pos, T) in enumerate(zip(motors_world, thrusts)):
            self.ax2.scatter([m_pos[0]], [m_pos[1]], [m_pos[2]],
                             color=motor_colors[j], s=200,
                             edgecolors='black', linewidths=2, depthshade=True, zorder=10)

        # Draw body axes (rotated)
        axis_body_len = self.motor_arm * 1.8

        # X_body (forward) - RED
        x_body_dir = rotate_point([axis_body_len, 0, 0], R)
        self.ax2.quiver(0, 0, 0, x_body_dir[0], x_body_dir[1], x_body_dir[2],
                        color='red', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
        self.ax2.text(x_body_dir[0] * 1.1, x_body_dir[1] * 1.1, x_body_dir[2] * 1.1,
                      'X\'', fontsize=12, color='red', weight='bold')

        # Y_body (right) - GREEN
        y_body_dir = rotate_point([0, axis_body_len, 0], R)
        self.ax2.quiver(0, 0, 0, y_body_dir[0], y_body_dir[1], y_body_dir[2],
                        color='green', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
        self.ax2.text(y_body_dir[0] * 1.1, y_body_dir[1] * 1.1, y_body_dir[2] * 1.1,
                      'Y\'', fontsize=12, color='green', weight='bold')

        # Z_body (down) - BLUE
        z_body_dir = rotate_point([0, 0, axis_body_len], R)
        self.ax2.quiver(0, 0, 0, z_body_dir[0], z_body_dir[1], z_body_dir[2],
                        color='blue', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
        self.ax2.text(z_body_dir[0] * 1.1, z_body_dir[1] * 1.1, z_body_dir[2] * 1.1,
                      'Z\'', fontsize=12, color='blue', weight='bold')

        # Draw body angles
        angle_radius = axis_body_len

        # Theta (pitch): kąt między początkowym a obróconym X (forward) wokół osi Y
        if abs(theta_deg) > 1:
            draw_angle_arc_3d(self.ax2, np.array([0, 0, 0]), angle_radius,
                              np.array([1, 0, 0]), rotate_point([1, 0, 0], R),
                              np.array([0, 1, 0]), 'green', f'θ={-1 * theta_deg:.1f}°')

        # Phi (roll): kąt między początkowym a obróconym Y (right) wokół osi X
        if abs(phi_deg) > 1:
            draw_angle_arc_3d(self.ax2, np.array([0, 0, 0]), angle_radius,
                              np.array([0, 1, 0]), rotate_point([0, 1, 0], R),
                              np.array([1, 0, 0]), 'red', f'φ={phi_deg:.1f}°')

        # Psi (yaw): kąt między początkowym a obróconym X (forward) wokół osi Z
        if abs(psi_deg) > 1:
            draw_angle_arc_3d(self.ax2, np.array([0, 0, 0]), angle_radius,
                              np.array([1, 0, 0]), rotate_point([1, 0, 0], R),
                              np.array([0, 0, 1]), 'blue', f'ψ={psi_deg:.1f}°')

        # Legend with motor info
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

        self.ax2.legend(handles=legend_elements,
                        loc='upper left',
                        bbox_to_anchor=(1.02, 1.0),
                        fontsize=9,
                        framealpha=0.95,
                        edgecolor='black',
                        fancybox=True,
                        shadow=True,
                        borderaxespad=0,
                        prop={'family': 'monospace', 'size': 9})
        self.ax2.invert_zaxis()

    def close(self):
        """Zamknięcie trybu interaktywnego"""
        plt.ioff()
        plt.show()
