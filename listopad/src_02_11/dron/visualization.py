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
from model_draw import aa_mdl


# ============================================================================
# HELPER FUNCTIONS: 3D ROTATION AND VISUALIZATION
# ============================================================================

def rotation_matrix_zyx(phi, theta, psi):
    """
    Compute full 3D rotation matrix using ZYX Euler angles (yaw-pitch-roll).
    Uses AEROSPACE CONVENTION for NED coordinate system.

    phi: roll angle (rotation around X)
    theta: pitch angle (rotation around Y)
        - theta > 0: nose up
        - theta < 0: nose down
    psi: yaw angle (rotation around Z)

    Returns: 3x3 rotation matrix from body frame to NED inertial frame
    """
    # Roll (X-axis rotation) - standard right-hand rule
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    Rx = np.array([
        [1, 0, 0],
        [0, c_phi, -s_phi],
        [0, s_phi, c_phi]
    ])

    # Pitch (Y-axis rotation) - AEROSPACE CONVENTION (reversed from math convention)
    # In aerospace/NED: positive theta rotates nose up
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    Ry = np.array([
        [c_theta, 0, -s_theta],  # Note: -sin for aerospace convention
        [0, 1, 0],
        [s_theta, 0, c_theta]  # Note: +sin for aerospace convention
    ])

    # Yaw (Z-axis rotation) - standard right-hand rule
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    Rz = np.array([
        [c_psi, -s_psi, 0],
        [s_psi, c_psi, 0],
        [0, 0, 1]
    ])

    # Combined rotation: R = Rz * Ry * Rx (ZYX Euler angles)
    return Rz @ Ry @ Rx


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
    ax.plot(arc_points[:, 1], arc_points[:, 0], arc_points[:, 2],
            color=color, linewidth=2, alpha=0.8)

    # Add label at mid-arc
    if len(arc_points) > 0:
        mid_point = arc_points[len(arc_points) // 2]
        ax.text(mid_point[1], mid_point[0], mid_point[2], label,
                fontsize=11, color=color, weight='bold')


# ============================================================================
# DRONE VISUALIZER CLASS
# ============================================================================

class DroneVisualizer:
    """
    Klasa obsługująca wszystkie wykresy symulatora drona
    """

    def __init__(self, X_ref_all, Y_terr_all, Z_terr_all, Y_ref_all, Z_ref_all,
                 motor_arm_x, motor_arm_y):
        """
        Inicjalizacja wizualizatora

        Parameters:
        -----------
        X_ref_all, Y_terr_all, Z_terr_all, Y_ref_all, Z_ref_all : array
            Referencyjne trajektorie
        motor_arm_x, motor_arm_y : float
            Wymiary ramion drona
        """
        self.X_ref_all = X_ref_all
        self.Y_terr_all = Y_terr_all
        self.Z_terr_all = Z_terr_all
        self.Y_ref_all = Y_ref_all
        self.Z_ref_all = Z_ref_all
        self.motor_arm_x = motor_arm_x
        self.motor_arm_y = motor_arm_y

        # Calculate boundaries
        self.x_boundary_minus = np.floor(-motor_arm_x)
        self.x_boundary_plus = np.ceil(motor_arm_x)
        self.y_boundary_minus = np.floor(-motor_arm_y)
        self.y_boundary_plus = np.ceil(motor_arm_y)

        # Initialize plots
        self._init_plots()

    def _init_plots(self):
        """Inicjalizacja figur matplotlib"""
        plt.ion()
        self.fig = plt.figure(1, figsize=(20, 12))

        # Subplot 1: Z(X) Trajectory (top left)
        self.ax1 = plt.subplot(2, 2, 1)
        self.ax1.plot(self.X_ref_all, self.Z_ref_all, 'r', label='Reference', linewidth=2)
        self.ax1.plot(self.X_ref_all, self.Z_terr_all, 'g', label='Terrain', linewidth=2)
        self.ax1.axhline(y=0, color='brown', linestyle='--', linewidth=1.5, label='Ground', alpha=0.7)
        self.ax1.legend(fontsize=11)
        self.ax1.axis([0, 50.0, -25.0, 0.5])
        self.ax1.set_xlabel('X [m]')
        self.ax1.set_ylabel('Z [m]')
        self.ax1.set_title('Trajektoria Z(X)', fontsize=10)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.invert_yaxis()

        # Subplot 2: Y(X) Trajectory (top right)
        self.ax2 = plt.subplot(2, 2, 2)
        self.ax2.plot(self.X_ref_all, self.Y_ref_all, 'r', label='Reference Y', linewidth=2)
        self.ax2.plot(self.X_ref_all, self.Y_terr_all, 'g', label='Terrain Y', linewidth=2)
        self.ax2.legend(fontsize=11)
        self.ax2.set_xlabel('X [m]')
        self.ax2.set_ylabel('Y [m]')
        self.ax2.set_title('Trajektoria Y(X)', fontsize=10)
        self.ax2.grid(True, alpha=0.3)

        # Subplot 3: 3D Trajectory (bottom left)
        self.ax3 = plt.subplot(2, 2, 3, projection='3d')
        self.ax3.plot(self.X_ref_all, self.Y_ref_all, self.Z_ref_all, 'r', label='Reference', linewidth=2)
        self.ax3.plot(self.X_ref_all, self.Y_terr_all, self.Z_terr_all, 'g', label='Terrain', linewidth=2)
        self.ax3.set_xlabel('X [m]')
        self.ax3.set_ylabel('Y [m]')
        self.ax3.set_zlabel('Z [m]')
        self.ax3.set_title('Trajektoria 3D (X, Y, Z)', fontsize=10)
        self.ax3.legend(fontsize=10)
        self.ax3.grid(True, alpha=0.3)
        self.ax3.invert_zaxis()

        # Subplot 4: Drone Orientation (bottom right) - 3D VIEW
        self.ax4 = plt.subplot(2, 2, 4, projection='3d')
        self.ax4.set_xlim(self.x_boundary_minus, self.x_boundary_plus)
        self.ax4.set_ylim(self.y_boundary_minus, self.y_boundary_plus)
        self.ax4.set_zlim(self.y_boundary_minus, self.y_boundary_plus)
        self.ax4.set_xticks(np.arange(self.x_boundary_minus, self.x_boundary_plus, 0.5))
        self.ax4.set_yticks(np.arange(self.y_boundary_minus, self.y_boundary_plus, 0.5))
        self.ax4.set_zticks(np.arange(self.y_boundary_minus, self.y_boundary_plus, 0.5))
        self.ax4.set_xlabel('Y [m]')
        self.ax4.set_ylabel('X [m]')
        self.ax4.set_zlabel('Z [m]')
        self.ax4.set_title('Quadcopter 3D Orientation (Full Rotation)')
        self.ax4.view_init(elev=20, azim=45)

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
        X_pos = x[6]
        Y_pos = x[7]
        Z_pos = x[8]
        phi = x[9]
        theta = x[10]
        psi = x[11]

        V = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
        phi_deg = math.degrees(phi)
        theta_deg = math.degrees(theta)
        psi_deg = math.degrees(psi)
        alt = -Z_pos

        # Get drone model coordinates
        xs, zs = aa_mdl(X_pos, Z_pos, theta, 5)
        xsy, ys = aa_mdl(X_pos, Y_pos, phi, 5)

        # ====================================================================
        # UPDATE SUBPLOT 1: Z(X) TRAJECTORY
        # ====================================================================
        self.ax1.clear()
        self.ax1.plot(self.X_ref_all, self.Z_ref_all, 'r', label='Reference', linewidth=2)
        self.ax1.plot(self.X_ref_all, self.Z_terr_all, 'g', label='Terrain', linewidth=2)
        self.ax1.axhline(y=0, color='brown', linestyle='--', linewidth=1.5, label='Ground', alpha=0.7)

        # Plot flight path
        self.ax1.plot(yp_array[:, 6], yp_array[:, 8], 'b', label='Flight path', alpha=0.7)
        self.ax1.plot(yp_array[i, 6], yp_array[i, 8], 'bo', markersize=8)

        # Plot drone model
        self.ax1.plot(xs[:5], zs[:5], 'k', linewidth=3)

        # Add limit violation indicator
        limit_indicator = " ⚠️ LIMITS!" if limits_exceeded else ""
        box_color = 'red' if limits_exceeded else 'lightgreen'

        txt = (f't={t:7.3f}s | V={V:7.3f}m/s{limit_indicator}\n'
               f'x={X_pos:7.3f}m y={Y_pos:7.3f}m z={alt:7.3f}m')

        self.ax1.text(0.98, 0.98, txt,
                      transform=self.ax1.transAxes,
                      fontsize=10,
                      verticalalignment='top',
                      horizontalalignment='right',
                      family='monospace',
                      weight='bold',
                      bbox=dict(boxstyle='round,pad=0.5',
                                facecolor=box_color,
                                edgecolor='black',
                                linewidth=2,
                                alpha=0.9))
        self.ax1.set_title('Trajektoria Z(X)', fontsize=10)
        self.ax1.legend(fontsize=9)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axis([max(0, X_pos - 10), X_pos + 10, -25.0, 0.5])
        self.ax1.invert_yaxis()
        self.ax1.set_xlabel('X [m]')
        self.ax1.set_ylabel('Z [m]')

        # ====================================================================
        # UPDATE SUBPLOT 2: Y(X) TRAJECTORY
        # ====================================================================
        self.ax2.clear()
        self.ax2.plot(self.X_ref_all, self.Y_ref_all, 'r', label='Reference Y', linewidth=2)
        self.ax2.plot(self.X_ref_all, self.Y_terr_all, 'g', label='Terrain Y', linewidth=2)
        self.ax2.plot(yp_array[:, 6], yp_array[:, 7], 'b', label='Flight path Y', alpha=0.7)
        self.ax2.plot(yp_array[i, 6], yp_array[i, 7], 'bo', markersize=8)

        # Plot drone model
        self.ax2.plot(xsy[:5], ys[:5], 'k', linewidth=3)
        self.ax2.set_xlabel('X [m]')
        self.ax2.set_ylabel('Y [m]')
        self.ax2.set_title('Trajektoria Y(X)', fontsize=10)
        self.ax2.legend(fontsize=9)
        self.ax2.grid(True, alpha=0.3)
        self.ax2.axis([max(0, X_pos - 10), X_pos + 10, -10, 10])

        # ====================================================================
        # UPDATE SUBPLOT 3: 3D TRAJECTORY
        # ====================================================================
        self.ax3.clear()
        self.ax3.plot(self.X_ref_all, self.Y_ref_all, self.Z_ref_all, 'r', label='Reference', linewidth=2)
        self.ax3.plot(self.X_ref_all, self.Y_terr_all, self.Z_terr_all, 'g', label='Terrain', linewidth=2)
        self.ax3.plot(yp_array[:, 6], yp_array[:, 7], yp_array[:, 8], 'b', label='Flight path', alpha=0.7, linewidth=2)
        self.ax3.scatter([yp_array[i, 6]], [yp_array[i, 7]], [yp_array[i, 8]], color='blue', s=100)

        self.ax3.set_xlabel('X [m]')
        self.ax3.set_ylabel('Y [m]')
        self.ax3.set_zlabel('Z [m]')
        self.ax3.set_title('Trajektoria 3D (X, Y, Z)', fontsize=10)
        self.ax3.legend(fontsize=9)
        self.ax3.grid(True, alpha=0.3)
        self.ax3.invert_zaxis()

        # ====================================================================
        # UPDATE SUBPLOT 4: QUADCOPTER 3D ORIENTATION
        # ====================================================================
        self._draw_drone_orientation(x, T1, T2, T3, T4, phi_deg, theta_deg, psi_deg)

        plt.draw()
        plt.pause(0.001)

    def _draw_drone_orientation(self, x, T1, T2, T3, T4, phi_deg, theta_deg, psi_deg):
        """Rysowanie orientacji drona w 3D"""
        self.ax4.clear()
        self.ax4.set_xlim(self.x_boundary_minus, self.x_boundary_plus)
        self.ax4.set_ylim(self.y_boundary_minus, self.y_boundary_plus)
        self.ax4.set_zlim(self.y_boundary_minus, self.y_boundary_plus)

        self.ax4.set_xlabel('Y [m]', fontsize=11, weight='bold')
        self.ax4.set_ylabel('X [m]', fontsize=11, weight='bold')
        self.ax4.set_zlabel('Z [m]', fontsize=11, weight='bold')

        self.ax4.set_xticks(np.arange(self.x_boundary_minus, self.x_boundary_plus, 0.5))
        self.ax4.set_yticks(np.arange(self.y_boundary_minus, self.y_boundary_plus, 0.5))
        self.ax4.set_zticks(np.arange(self.x_boundary_minus, self.x_boundary_plus, 0.5))

        self.ax4.view_init(elev=20, azim=45)

        # Draw world coordinate axes
        axis_len = self.x_boundary_plus + 0.5
        self.ax4.plot([0, 0], [0, axis_len], [0, 0], color='black', linewidth=3, alpha=0.7, label='X (world)')
        self.ax4.plot([0, axis_len], [0, 0], [0, 0], color='black', linewidth=3, alpha=0.7, label='Y (world)')
        self.ax4.plot([0, 0], [0, 0], [0, axis_len], color='black', linewidth=3, alpha=0.7, label='Z (world)')

        self.ax4.text(0, axis_len, 0, 'X', fontsize=14, color='black', weight='bold')
        self.ax4.text(axis_len, 0, 0, 'Y', fontsize=14, color='black', weight='bold')
        self.ax4.text(0, 0, axis_len, 'Z', fontsize=14, color='black', weight='bold')

        # Get full rotation matrix
        phi, theta, psi = x[9], x[10], x[11]
        R = rotation_matrix_zyx(phi, theta, psi)

        # Drone dimensions
        arm_length = self.motor_arm_x

        # Motor positions in body frame (X configuration)
        angle_offset = np.pi / 4  # 45 degrees
        motors_body = [
            np.array([arm_length * np.cos(angle_offset), arm_length * np.sin(angle_offset), 0]),
            np.array([arm_length * np.cos(angle_offset), -arm_length * np.sin(angle_offset), 0]),
            np.array([-arm_length * np.cos(angle_offset), arm_length * np.sin(angle_offset), 0]),
            np.array([-arm_length * np.cos(angle_offset), -arm_length * np.sin(angle_offset), 0])
        ]

        # Rotate motors to world frame
        motors_world = [rotate_point(m, R) for m in motors_body]

        # Draw drone arms (X shape)
        self.ax4.plot([motors_world[0][1], motors_world[3][1]],
                      [motors_world[0][0], motors_world[3][0]],
                      [motors_world[0][2], motors_world[3][2]],
                      'k-', linewidth=3, alpha=0.9)
        self.ax4.plot([motors_world[1][1], motors_world[2][1]],
                      [motors_world[1][0], motors_world[2][0]],
                      [motors_world[1][2], motors_world[2][2]],
                      'k-', linewidth=3, alpha=0.9)

        # Draw center body (cube)
        r = self.motor_arm_x / 8
        vertices_body = [
            np.array([-r, -r, -r]), np.array([r, -r, -r]), np.array([r, r, -r]), np.array([-r, r, -r]),
            np.array([-r, -r, r]), np.array([r, -r, r]), np.array([r, r, r]), np.array([-r, r, r])
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
        self.ax4.add_collection3d(cube)

        # Draw motors
        thrusts = [T1, T2, T3, T4]
        motor_colors = ['red', 'blue', 'green', 'orange']

        for j, (m_pos, T) in enumerate(zip(motors_world, thrusts)):
            self.ax4.scatter([m_pos[1]], [m_pos[0]], [m_pos[2]],
                             color=motor_colors[j], s=200,
                             edgecolors='black', linewidths=2, depthshade=True, zorder=10)

        # Draw body axes (rotated)
        axis_body_len = self.motor_arm_x * 1.8

        # X_body (forward) - RED
        x_body_dir = rotate_point([axis_body_len, 0, 0], R)
        self.ax4.quiver(0, 0, 0, x_body_dir[1], x_body_dir[0], x_body_dir[2],
                        color='red', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
        self.ax4.text(x_body_dir[1] * 1.1, x_body_dir[0] * 1.1, x_body_dir[2] * 1.1,
                      'X\'', fontsize=12, color='red', weight='bold')

        # Y_body (right) - GREEN
        y_body_dir = rotate_point([0, axis_body_len, 0], R)
        self.ax4.quiver(0, 0, 0, y_body_dir[1], y_body_dir[0], y_body_dir[2],
                        color='green', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
        self.ax4.text(y_body_dir[1] * 1.1, y_body_dir[0] * 1.1, y_body_dir[2] * 1.1,
                      'Y\'', fontsize=12, color='green', weight='bold')

        # Z_body (down) - BLUE
        z_body_dir = rotate_point([0, 0, axis_body_len], R)
        self.ax4.quiver(0, 0, 0, z_body_dir[1], z_body_dir[0], z_body_dir[2],
                        color='blue', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
        self.ax4.text(z_body_dir[1] * 1.1, z_body_dir[0] * 1.1, z_body_dir[2] * 1.1,
                      'Z\'', fontsize=12, color='blue', weight='bold')

        # Draw body angles
        angle_radius = axis_body_len

        if abs(theta_deg) > 1:
            draw_angle_arc_3d(self.ax4, np.array([0, 0, 0]), angle_radius,
                              np.array([1, 0, 0]), rotate_point([0, 1, 0], R),
                              np.array([0, 1, 0]), 'green', f'θ={theta_deg:.1f}°')

        if abs(phi_deg) > 1:
            draw_angle_arc_3d(self.ax4, np.array([0, 0, 0]), angle_radius,
                              np.array([0, 1, 0]), rotate_point([1, 0, 0], R),
                              np.array([1, 0, 0]), 'red', f'φ={phi_deg:.1f}°')

        if abs(psi_deg) > 1:
            draw_angle_arc_3d(self.ax4, np.array([0, 0, 0]), angle_radius,
                              np.array([1, 0, 0]), rotate_point([0, 0, 1], R),
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

        self.ax4.legend(handles=legend_elements,
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
        """Zamknięcie trybu interaktywnego"""
        plt.ioff()
        plt.show()