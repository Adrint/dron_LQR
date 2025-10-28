"""
Trajectory export module - generates trajectory in format compatible with dron_lqr_with_orientation.py
"""

import numpy as np
import json
from typing import List, Tuple
from pathlib import Path


class TrajectoryExporter:
    """Export planned path to format compatible with drone controller"""
    
    def __init__(self, velocity: float = 2.0, dt: float = 0.01):
        """
        Args:
            velocity: Drone velocity (m/s)
            dt: Time step (s) - must match controller's dt
        """
        self.velocity = velocity
        self.dt = dt
    
    def smooth_path(self, waypoints: List[np.ndarray], 
                    smoothing_factor: float = 0.5) -> List[np.ndarray]:
        """
        Smooth path using moving average
        
        Args:
            waypoints: List of waypoint positions
            smoothing_factor: Smoothing strength (0 = no smoothing, 1 = max smoothing)
            
        Returns:
            Smoothed waypoints
        """
        if len(waypoints) < 3 or smoothing_factor <= 0:
            return waypoints
        
        waypoints = np.array(waypoints)
        smoothed = waypoints.copy()
        
        # Apply simple moving average smoothing
        window = max(3, int(5 * smoothing_factor))
        if window % 2 == 0:
            window += 1  # Make odd
        
        for i in range(1, len(waypoints) - 1):
            start = max(0, i - window // 2)
            end = min(len(waypoints), i + window // 2 + 1)
            smoothed[i] = np.mean(waypoints[start:end], axis=0)
        
        return [wp for wp in smoothed]
    
    def interpolate_path(self, waypoints: List[np.ndarray]) -> Tuple[np.ndarray, ...]:
        """
        Interpolate waypoints to create smooth trajectory at constant velocity
        
        Args:
            waypoints: List of waypoint positions [X, Y, Z] in NED
            
        Returns:
            Tuple of (X_ref, Y_ref, Z_ref, alpha, beta) arrays
            - X_ref: Forward positions
            - Y_ref: Lateral positions  
            - Z_ref: Vertical positions (NED: negative = altitude)
            - alpha: Pitch angles (radians)
            - beta: Roll angles (radians)
        """
        waypoints = np.array(waypoints)
        
        # Calculate distances between waypoints
        distances = [0.0]
        for i in range(1, len(waypoints)):
            dist = np.linalg.norm(waypoints[i] - waypoints[i-1])
            distances.append(distances[-1] + dist)
        
        total_distance = distances[-1]
        total_time = total_distance / self.velocity
        num_points = int(total_time / self.dt) + 1
        
        # Interpolate positions
        t_samples = np.linspace(0, total_distance, num_points)
        
        X_ref = np.interp(t_samples, distances, waypoints[:, 0])
        Y_ref = np.interp(t_samples, distances, waypoints[:, 1])
        Z_ref = np.interp(t_samples, distances, waypoints[:, 2])
        
        # Calculate angles (slope of trajectory)
        dx = self.velocity * self.dt
        
        # Alpha: pitch angle (slope in X-Z plane)
        dZ_dX = np.gradient(Z_ref, dx)
        alpha = np.arctan(dZ_dX)
        
        # Beta: roll angle (slope in X-Y plane)
        dY_dX = np.gradient(Y_ref, dx)
        beta = np.arctan(dY_dX)
        
        return X_ref, Y_ref, Z_ref, alpha, beta
    
    def export_numpy(self, waypoints: List[np.ndarray], 
                     output_file: str = 'trajectory_output.npz',
                     smooth: bool = True) -> dict:
        """
        Export trajectory to NumPy format (compatible with trajectory.py)
        
        Args:
            waypoints: List of waypoint positions
            output_file: Output file path
            smooth: Whether to smooth the path
            
        Returns:
            Dictionary containing trajectory data
        """
        if smooth:
            waypoints = self.smooth_path(waypoints, smoothing_factor=0.3)
        
        # Interpolate to create dense trajectory
        X_ref, Y_ref, Z_ref, alpha, beta = self.interpolate_path(waypoints)
        
        # For compatibility with trajectory.py, also include terrain data
        # (In this case, terrain = reference - offset)
        Z_terr = Z_ref + 2.0  # Assuming 2m flight height above "terrain"
        Y_terr = Y_ref.copy()
        
        # Save to file
        trajectory_data = {
            'X_ref': X_ref,
            'Y_ref': Y_ref,
            'Z_ref': Z_ref,
            'Y_terr': Y_terr,
            'Z_terr': Z_terr,
            'alpha': alpha,
            'beta': beta,
            'velocity': self.velocity,
            'dt': self.dt,
            'waypoints': np.array(waypoints)
        }
        
        np.savez(output_file, **trajectory_data)
        print(f"Trajectory exported to: {output_file}")
        print(f"  - Number of waypoints: {len(waypoints)}")
        print(f"  - Number of reference points: {len(X_ref)}")
        print(f"  - Total distance: {X_ref[-1] - X_ref[0]:.2f} m")
        print(f"  - Duration: {len(X_ref) * self.dt:.2f} s")
        
        return trajectory_data
    
    def export_json(self, waypoints: List[np.ndarray],
                    output_file: str = 'trajectory_output.json') -> dict:
        """
        Export trajectory to JSON format (human-readable)
        
        Args:
            waypoints: List of waypoint positions
            output_file: Output file path
            
        Returns:
            Dictionary containing trajectory data
        """
        trajectory_data = {
            'waypoints': [
                {'x': float(wp[0]), 'y': float(wp[1]), 'z': float(wp[2])}
                for wp in waypoints
            ],
            'velocity': self.velocity,
            'dt': self.dt,
            'num_waypoints': len(waypoints),
            'coordinate_system': 'NED'
        }
        
        with open(output_file, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        
        print(f"Trajectory exported to JSON: {output_file}")
        return trajectory_data
    
    def generate_trajectory_function(self, waypoints: List[np.ndarray],
                                    output_file: str = 'custom_trajectory.py',
                                    smooth: bool = True):
        """
        Generate a Python file with trajectory function compatible with trajectory.py
        
        Args:
            waypoints: List of waypoint positions
            output_file: Output Python file
            smooth: Whether to smooth the path
        """
        if smooth:
            waypoints = self.smooth_path(waypoints, smoothing_factor=0.3)
        
        X_ref, Y_ref, Z_ref, alpha, beta = self.interpolate_path(waypoints)
        Z_terr = Z_ref + 2.0
        Y_terr = Y_ref.copy()
        
        # Generate Python code
        code = f'''"""
Custom trajectory generated by path planning system
Compatible with dron_lqr_with_orientation.py

Generated with:
- Velocity: {self.velocity} m/s
- Time step: {self.dt} s
- Number of waypoints: {len(waypoints)}
"""

import numpy as np


def generate_reference_profile(Vel={self.velocity}, dt={self.dt}, X_max=None):
    """
    Load pre-computed trajectory from path planning
    
    Parameters match trajectory.py for compatibility
    """
    # Pre-computed trajectory data
    X_ref = np.array({X_ref.tolist()})
    Y_terr = np.array({Y_terr.tolist()})
    Z_terr = np.array({Z_terr.tolist()})
    Y_ref = np.array({Y_ref.tolist()})
    Z_ref = np.array({Z_ref.tolist()})
    alpha = np.array({alpha.tolist()})
    beta = np.array({beta.tolist()})
    
    # If X_max specified, truncate trajectory
    if X_max is not None:
        idx = np.where(X_ref <= X_max)[0]
        if len(idx) > 0:
            last_idx = idx[-1] + 1
            X_ref = X_ref[:last_idx]
            Y_terr = Y_terr[:last_idx]
            Z_terr = Z_terr[:last_idx]
            Y_ref = Y_ref[:last_idx]
            Z_ref = Z_ref[:last_idx]
            alpha = alpha[:last_idx]
            beta = beta[:last_idx]
    
    return X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta


def generate_reference_profile_2d(Vel={self.velocity}, dt={self.dt}, X_max=None):
    """2D version for backward compatibility"""
    X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta = generate_reference_profile(Vel, dt, X_max)
    return X_ref, Z_terr, Z_ref, alpha


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta = generate_reference_profile()
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # X-Z plane
    axes[0, 0].plot(X_ref, Z_ref, 'r-', label='Flight path')
    axes[0, 0].plot(X_ref, Z_terr, 'g--', label='Terrain', alpha=0.5)
    axes[0, 0].set_xlabel('X [m]')
    axes[0, 0].set_ylabel('Z [m] (NED)')
    axes[0, 0].set_title('Side View (X-Z)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].invert_yaxis()
    
    # X-Y plane
    axes[0, 1].plot(X_ref, Y_ref, 'r-', label='Flight path')
    axes[0, 1].set_xlabel('X [m]')
    axes[0, 1].set_ylabel('Y [m] (NED)')
    axes[0, 1].set_title('Top View (X-Y)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # 3D view
    ax3d = fig.add_subplot(2, 2, 3, projection='3d')
    ax3d.plot(X_ref, Y_ref, Z_ref, 'r-', linewidth=2)
    ax3d.set_xlabel('X [m]')
    ax3d.set_ylabel('Y [m]')
    ax3d.set_zlabel('Z [m]')
    ax3d.set_title('3D Trajectory')
    ax3d.invert_zaxis()
    
    # Angles
    axes[1, 1].plot(X_ref, np.degrees(alpha), label='Alpha (pitch)')
    axes[1, 1].plot(X_ref, np.degrees(beta), label='Beta (roll)')
    axes[1, 1].set_xlabel('X [m]')
    axes[1, 1].set_ylabel('Angle [deg]')
    axes[1, 1].set_title('Trajectory Angles')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.show()
    
    print(f"Trajectory points: {{len(X_ref)}}")
    print(f"X range: {{X_ref[0]:.2f}} to {{X_ref[-1]:.2f}} m")
    print(f"Y range: {{Y_ref.min():.2f}} to {{Y_ref.max():.2f}} m")
    print(f"Z range: {{Z_ref.min():.2f}} to {{Z_ref.max():.2f}} m")
'''
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(code)
        
        print(f"Trajectory function generated: {output_file}")
        print(f"  You can now use: from {Path(output_file).stem} import generate_reference_profile")


def load_trajectory(file_path: str) -> dict:
    """
    Load trajectory from saved file
    
    Args:
        file_path: Path to .npz file
        
    Returns:
        Dictionary with trajectory data
    """
    data = np.load(file_path)
    return {key: data[key] for key in data.files}
