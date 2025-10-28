"""
Visualization module for path planning
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from typing import List, Optional

from obstacles import ObstacleManager, BoxObstacle, CylinderObstacle, SphereObstacle
import config


class PathVisualizer:
    """Visualize path planning results"""
    
    def __init__(self, obstacle_manager: ObstacleManager):
        """
        Args:
            obstacle_manager: Manager containing obstacles
        """
        self.obstacle_manager = obstacle_manager
        
    def plot_3d(self, path: Optional[List[np.ndarray]] = None,
                start: Optional[np.ndarray] = None,
                goal: Optional[np.ndarray] = None,
                title: str = "3D Path Planning"):
        """
        Create 3D visualization of path and obstacles
        
        Args:
            path: List of waypoints (if available) - expects NED coordinates
            start: Start position - NED coordinates
            goal: Goal position - NED coordinates
            title: Plot title

        Note: Internally uses NED (Z negative = altitude), but displays Z as positive for clarity
        """
        fig = plt.figure(figsize=config.FIGURE_SIZE)
        ax = fig.add_subplot(111, projection='3d')

        # Convert NED to display coordinates (Z_display = -Z_ned for intuitive visualization)
        def to_display(point):
            """Convert NED coordinates to display coordinates"""
            if point is None:
                return None
            p = np.array(point)
            return np.array([p[0], p[1], -p[2]])  # Flip Z for display

        # Plot ground plane at Z=0
        self._plot_ground_plane(ax)

        # Plot obstacles (converted to display coords)
        self._plot_obstacles_3d(ax)

        # Plot path
        if path is not None and len(path) > 0:
            path_array = np.array([to_display(p) for p in path])
            ax.plot(path_array[:, 0], path_array[:, 1], path_array[:, 2],
                   color=config.COLOR_PATH, linewidth=3, label='Planned Path', marker='o', markersize=3)

        # Plot start and goal
        if start is not None:
            start_disp = to_display(start)
            ax.scatter(*start_disp, color=config.COLOR_START, s=200, marker='o',
                      label='Start', edgecolors='black', linewidths=2, zorder=10)

        if goal is not None:
            goal_disp = to_display(goal)
            ax.scatter(*goal_disp, color=config.COLOR_GOAL, s=200, marker='*',
                      label='Goal', edgecolors='black', linewidths=2, zorder=10)

        # Labels and formatting
        ax.set_xlabel('X [m]', fontsize=12)
        ax.set_ylabel('Y [m]', fontsize=12)
        ax.set_zlabel('Z [m]', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)

        # Equal aspect ratio
        self._set_axes_equal_3d(ax)

        plt.tight_layout()

    def plot_2d_projections(self, path: Optional[List[np.ndarray]] = None,
                           start: Optional[np.ndarray] = None,
                           goal: Optional[np.ndarray] = None):
        """
        Create 2D projections (XY, XZ, YZ views)

        Args:
            path: List of waypoints
            start: Start position
            goal: Goal position
        """
        fig, axes = plt.subplots(2, 2, figsize=config.FIGURE_SIZE)

        # XY view (top view)
        ax_xy = axes[0, 0]
        self._plot_obstacles_2d(ax_xy, 'xy')
        if path is not None:
            path_array = np.array(path)
            ax_xy.plot(path_array[:, 0], path_array[:, 1],
                      color=config.COLOR_PATH, linewidth=2, label='Path', marker='o', markersize=4)
        if start is not None:
            ax_xy.scatter(start[0], start[1], color=config.COLOR_START, s=150,
                         marker='o', label='Start', edgecolors='black', linewidths=2, zorder=5)
        if goal is not None:
            ax_xy.scatter(goal[0], goal[1], color=config.COLOR_GOAL, s=150,
                         marker='*', label='Goal', edgecolors='black', linewidths=2, zorder=5)
        ax_xy.set_xlabel('X [m]')
        ax_xy.set_ylabel('Y [m]')
        ax_xy.set_title('Top View (XY)')
        ax_xy.legend()
        ax_xy.grid(True, alpha=0.3)
        ax_xy.set_aspect('equal')

        # XZ view (side view)
        ax_xz = axes[0, 1]
        self._plot_obstacles_2d(ax_xz, 'xz')

        # Add ground line at Z=0
        xlim = ax_xz.get_xlim() if path is not None else [0, 50]
        ax_xz.axhline(y=0, color='brown', linestyle='--', linewidth=2, alpha=0.7, zorder=1)

        if path is not None:
            path_array = np.array(path)
            # Convert Z to display coordinates (positive = altitude)
            z_display = -path_array[:, 2]
            ax_xz.plot(path_array[:, 0], z_display,
                      color=config.COLOR_PATH, linewidth=2, label='Path', marker='o', markersize=4)
        if start is not None:
            ax_xz.scatter(start[0], -start[2], color=config.COLOR_START, s=150,
                         marker='o', label='Start', edgecolors='black', linewidths=2, zorder=5)
        if goal is not None:
            ax_xz.scatter(goal[0], -goal[2], color=config.COLOR_GOAL, s=150,
                         marker='*', label='Goal', edgecolors='black', linewidths=2, zorder=5)
        ax_xz.set_xlabel('X [m]')
        ax_xz.set_ylabel('Z [m]')
        ax_xz.set_title('Side View (XZ)')
        ax_xz.legend()
        ax_xz.grid(True, alpha=0.3)
        ax_xz.set_aspect('equal')

        # YZ view (front view)
        ax_yz = axes[1, 0]
        self._plot_obstacles_2d(ax_yz, 'yz')

        # Add ground line at Z=0
        ylim = ax_yz.get_xlim() if path is not None else [-10, 10]
        ax_yz.axhline(y=0, color='brown', linestyle='--', linewidth=2, alpha=0.7, zorder=1)

        if path is not None:
            path_array = np.array(path)
            # Convert Z to display coordinates (positive = altitude)
            z_display = -path_array[:, 2]
            ax_yz.plot(path_array[:, 1], z_display,
                      color=config.COLOR_PATH, linewidth=2, label='Path', marker='o', markersize=4)
        if start is not None:
            ax_yz.scatter(start[1], -start[2], color=config.COLOR_START, s=150,
                         marker='o', label='Start', edgecolors='black', linewidths=2, zorder=5)
        if goal is not None:
            ax_yz.scatter(goal[1], -goal[2], color=config.COLOR_GOAL, s=150,
                         marker='*', label='Goal', edgecolors='black', linewidths=2, zorder=5)
        ax_yz.set_xlabel('Y [m]')
        ax_yz.set_ylabel('Z [m]')
        ax_yz.set_title('Front View (YZ)')
        ax_yz.legend()
        ax_yz.grid(True, alpha=0.3)
        ax_yz.set_aspect('equal')

        # Path statistics
        ax_stats = axes[1, 1]
        ax_stats.axis('off')

        if path is not None and len(path) > 1:
            path_array = np.array(path)

            # Calculate statistics
            distances = np.linalg.norm(np.diff(path_array, axis=0), axis=1)
            total_distance = np.sum(distances)

            x_range = path_array[:, 0].max() - path_array[:, 0].min()
            y_range = path_array[:, 1].max() - path_array[:, 1].min()
            z_range = path_array[:, 2].max() - path_array[:, 2].min()

            stats_text = f"""
PATH STATISTICS

Waypoints: {len(path)}
Total Distance: {total_distance:.2f} m

Position Ranges:
  X: {path_array[:, 0].min():.2f} to {path_array[:, 0].max():.2f} m ({x_range:.2f} m)
  Y: {path_array[:, 1].min():.2f} to {path_array[:, 1].max():.2f} m ({y_range:.2f} m)
  Z: {path_array[:, 2].min():.2f} to {path_array[:, 2].max():.2f} m ({z_range:.2f} m)

Altitude (AGL):
  Min: {-path_array[:, 2].max():.2f} m
  Max: {-path_array[:, 2].min():.2f} m

Obstacles: {len(self.obstacle_manager)}
            """
            ax_stats.text(0.1, 0.5, stats_text, fontsize=11, family='monospace',
                         verticalalignment='center')

        plt.tight_layout()

    def _plot_ground_plane(self, ax):
        """Plot ground plane at Z=0 for reference"""
        # Get bounds from obstacles or use defaults
        if len(self.obstacle_manager) > 0:
            obs_min, obs_max = self.obstacle_manager.get_all_bounds()
            x_range = [obs_min[0] - 5, obs_max[0] + 5]
            y_range = [obs_min[1] - 5, obs_max[1] + 5]
        else:
            x_range = [-10, 60]
            y_range = [-20, 20]

        # Create ground mesh
        xx, yy = np.meshgrid(
            np.linspace(x_range[0], x_range[1], 10),
            np.linspace(y_range[0], y_range[1], 10)
        )
        zz = np.zeros_like(xx)

        # Plot ground plane
        ax.plot_surface(xx, yy, zz, alpha=0.2, color='tan',
                       edgecolor='brown', linewidth=0.5, label='Ground')

        # Add grid on ground
        ax.contour(xx, yy, zz, levels=[0], colors='brown', alpha=0.3, linewidths=1)

    def _plot_obstacles_3d(self, ax):
        """Plot obstacles in 3D (with Z converted to display coordinates)"""
        for obstacle in self.obstacle_manager.obstacles:
            if isinstance(obstacle, BoxObstacle):
                self._plot_box_3d(ax, obstacle)
            elif isinstance(obstacle, CylinderObstacle):
                self._plot_cylinder_3d(ax, obstacle)
            elif isinstance(obstacle, SphereObstacle):
                self._plot_sphere_3d(ax, obstacle)

    def _plot_box_3d(self, ax, box: BoxObstacle):
        """Plot box obstacle in 3D (converts NED Z to display Z)"""
        # Convert NED to display coordinates
        center = np.array([box.position[0], box.position[1], -box.position[2]])
        size = box.size

        # Define vertices (in display coordinates)
        r = size / 2
        vertices = [
            center + r * np.array([x, y, z])
            for x in [-1, 1] for y in [-1, 1] for z in [-1, 1]
        ]

        # Define faces (6 faces of a box)
        faces = [
            [vertices[0], vertices[1], vertices[3], vertices[2]],  # Bottom
            [vertices[4], vertices[5], vertices[7], vertices[6]],  # Top
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back
            [vertices[0], vertices[2], vertices[6], vertices[4]],  # Left
            [vertices[1], vertices[3], vertices[7], vertices[5]],  # Right
        ]

        collection = Poly3DCollection(faces, alpha=0.3, facecolor=config.COLOR_OBSTACLE,
                                     edgecolor='black', linewidths=1)
        ax.add_collection3d(collection)

        # Add label
        if box.name:
            ax.text(center[0], center[1], center[2], box.name, fontsize=9)

    def _plot_cylinder_3d(self, ax, cylinder: CylinderObstacle):
        """Plot cylinder obstacle in 3D (converts NED Z to display Z)"""
        # Convert NED to display coordinates
        # In NED: z_base is on ground (0), extends downward (negative)
        # In display: base at ground (0), extends upward (positive)
        z_base_display = -cylinder.position[2]  # Convert NED to display
        z_top_display = z_base_display + cylinder.height  # Top is higher

        # Create cylinder mesh
        theta = np.linspace(0, 2*np.pi, 20)
        z = np.linspace(z_base_display, z_top_display, 10)
        Theta, Z = np.meshgrid(theta, z)

        X = cylinder.position[0] + cylinder.radius * np.cos(Theta)
        Y = cylinder.position[1] + cylinder.radius * np.sin(Theta)

        ax.plot_surface(X, Y, Z, alpha=0.3, color=config.COLOR_OBSTACLE)

        if cylinder.name:
            ax.text(cylinder.position[0], cylinder.position[1],
                   z_base_display + cylinder.height/2, cylinder.name, fontsize=9)

    def _plot_sphere_3d(self, ax, sphere: SphereObstacle):
        """Plot sphere obstacle in 3D (converts NED Z to display Z)"""
        # Convert NED to display coordinates
        center_display = np.array([sphere.position[0], sphere.position[1], -sphere.position[2]])

        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = center_display[0] + sphere.radius * np.outer(np.cos(u), np.sin(v))
        y = center_display[1] + sphere.radius * np.outer(np.sin(u), np.sin(v))
        z = center_display[2] + sphere.radius * np.outer(np.ones(np.size(u)), np.cos(v))

        ax.plot_surface(x, y, z, alpha=0.3, color=config.COLOR_OBSTACLE)

        if sphere.name:
            ax.text(center_display[0], center_display[1], center_display[2],
                   sphere.name, fontsize=9)

    def _plot_obstacles_2d(self, ax, plane: str):
        """Plot obstacles in 2D projection"""
        for obstacle in self.obstacle_manager.obstacles:
            if isinstance(obstacle, BoxObstacle):
                self._plot_box_2d(ax, obstacle, plane)
            elif isinstance(obstacle, CylinderObstacle):
                self._plot_cylinder_2d(ax, obstacle, plane)
            elif isinstance(obstacle, SphereObstacle):
                self._plot_sphere_2d(ax, obstacle, plane)

    def _plot_box_2d(self, ax, box: BoxObstacle, plane: str):
        """Plot box in 2D projection (converts Z to display for xz/yz planes)"""
        center = box.position
        size = box.size

        if plane == 'xy':
            x, y = center[0], center[1]
            w, h = size[0], size[1]
        elif plane == 'xz':
            # Convert Z to display coordinates
            x = center[0]
            y = -center[2]  # Display Z (positive = altitude)
            w, h = size[0], size[2]
        else:  # yz
            # Convert Z to display coordinates
            x = center[1]
            y = -center[2]  # Display Z (positive = altitude)
            w, h = size[1], size[2]

        rect = plt.Rectangle((x - w/2, y - h/2), w, h,
                            fill=True, facecolor=config.COLOR_OBSTACLE,
                            edgecolor='black', alpha=0.5, linewidth=1.5)
        ax.add_patch(rect)

    def _plot_cylinder_2d(self, ax, cylinder: CylinderObstacle, plane: str):
        """Plot cylinder in 2D projection (converts Z to display for xz/yz planes)"""
        if plane == 'xy':
            circle = plt.Circle((cylinder.position[0], cylinder.position[1]),
                              cylinder.radius, fill=True, facecolor=config.COLOR_OBSTACLE,
                              edgecolor='black', alpha=0.5, linewidth=1.5)
            ax.add_patch(circle)
        elif plane == 'xz':
            x = cylinder.position[0]
            # Convert Z to display: base at ground (0), extends upward
            z_base_display = -cylinder.position[2]  # Ground in display coords
            z_top_display = z_base_display + cylinder.height
            rect = plt.Rectangle((x - cylinder.radius, z_base_display),
                                cylinder.radius * 2, cylinder.height,
                                fill=True, facecolor=config.COLOR_OBSTACLE,
                                edgecolor='black', alpha=0.5, linewidth=1.5)
            ax.add_patch(rect)
        else:  # yz
            y = cylinder.position[1]
            # Convert Z to display: base at ground (0), extends upward
            z_base_display = -cylinder.position[2]  # Ground in display coords
            z_top_display = z_base_display + cylinder.height
            rect = plt.Rectangle((y - cylinder.radius, z_base_display),
                                cylinder.radius * 2, cylinder.height,
                                fill=True, facecolor=config.COLOR_OBSTACLE,
                                edgecolor='black', alpha=0.5, linewidth=1.5)
            ax.add_patch(rect)

    def _plot_sphere_2d(self, ax, sphere: SphereObstacle, plane: str):
        """Plot sphere in 2D projection (as circle, converts Z to display for xz/yz)"""
        if plane == 'xy':
            circle = plt.Circle((sphere.position[0], sphere.position[1]),
                              sphere.radius, fill=True, facecolor=config.COLOR_OBSTACLE,
                              edgecolor='black', alpha=0.5, linewidth=1.5)
        elif plane == 'xz':
            # Convert Z to display coordinates
            circle = plt.Circle((sphere.position[0], -sphere.position[2]),
                              sphere.radius, fill=True, facecolor=config.COLOR_OBSTACLE,
                              edgecolor='black', alpha=0.5, linewidth=1.5)
        else:  # yz
            # Convert Z to display coordinates
            circle = plt.Circle((sphere.position[1], -sphere.position[2]),
                              sphere.radius, fill=True, facecolor=config.COLOR_OBSTACLE,
                              edgecolor='black', alpha=0.5, linewidth=1.5)
        ax.add_patch(circle)

    @staticmethod
    def _set_axes_equal_3d(ax):
        """Set equal aspect ratio for 3D plot"""
        limits = np.array([
            ax.get_xlim3d(),
            ax.get_ylim3d(),
            ax.get_zlim3d(),
        ])

        center = np.mean(limits, axis=1)
        radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))

        ax.set_xlim3d([center[0] - radius, center[0] + radius])
        ax.set_ylim3d([center[1] - radius, center[1] + radius])
        ax.set_zlim3d([center[2] - radius, center[2] + radius])

    def show(self):
        """Display all plots"""
        plt.show()