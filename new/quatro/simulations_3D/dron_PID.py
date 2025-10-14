"""
Quadrocopter 3D Simulation - PID Controller
Clean and organized implementation with proper 3D visualization
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from common.integrator import rk45_simple
from common.trajectory import generate_3d_trajectory
from common.constants import (z0, Vel, MASS, g, T_max, T_min, MOTOR_ARM_LENGTH)


class QuadcopterPIDController:
    """PID controller for 3D trajectory tracking"""
    
    def __init__(self):
        # Altitude control
        self.kp_z = 25.0
        self.kd_z = 12.0
        
        # Horizontal position control
        self.kp_x = 3.0
        self.kd_x = 4.0
        self.kp_y = 3.0
        self.kd_y = 4.0
        
        # Attitude control
        self.kp_att = 6.0
        self.kd_att = 3.0
        
        # Yaw control
        self.kp_yaw = 3.0
        self.kd_yaw = 1.5
        
    def compute_thrust(self, state, ref_pos, ref_vel):
        """Compute motor thrusts for 3D trajectory tracking"""
        # Extract state
        vx, vy, vz = state[0:3]
        p, q, r = state[3:6]
        X, Y, Z = state[6:9]
        phi, theta, psi = state[9:12]
        
        x_ref, y_ref, z_ref = ref_pos
        vx_ref, vy_ref, vz_ref = ref_vel
        
        # === ALTITUDE CONTROL ===
        e_z = z_ref - Z
        e_vz = vz_ref - vz
        T_total = MASS * g + self.kp_z * e_z + self.kd_z * e_vz
        T_total = np.clip(T_total, MASS * g * 0.3, MASS * g * 2.5)
        
        # === HORIZONTAL POSITION CONTROL ===
        e_x = x_ref - X
        e_y = y_ref - Y
        e_vx = vx_ref - vx
        e_vy = vy_ref - vy
        
        # Desired angles
        theta_des = np.clip(self.kp_x * e_x + self.kd_x * e_vx, -0.4, 0.4)
        phi_des = np.clip(self.kp_y * e_y + self.kd_y * e_vy, -0.4, 0.4)
        
        # Desired yaw
        if abs(vx) > 0.5 or abs(vy) > 0.5:
            psi_des = np.arctan2(vy, vx)
        else:
            psi_des = 0.0
        
        # === ATTITUDE CONTROL ===
        e_phi = phi_des - phi
        e_theta = theta_des - theta
        e_psi = np.arctan2(np.sin(psi_des - psi), np.cos(psi_des - psi))
        
        M_roll = self.kp_att * e_phi - self.kd_att * p
        M_pitch = self.kp_att * e_theta - self.kd_att * q
        M_yaw = self.kp_yaw * e_psi - self.kd_yaw * r
        
        M_roll = np.clip(M_roll, -15.0, 15.0)
        M_pitch = np.clip(M_pitch, -15.0, 15.0)
        M_yaw = np.clip(M_yaw, -3.0, 3.0)
        
        # === MOTOR MIXING (X-configuration) ===
        arm = MOTOR_ARM_LENGTH / np.sqrt(2)
        k_yaw = 0.1
        T_avg = T_total / 4.0
        
        dT_roll = M_roll / (2 * arm)
        dT_pitch = M_pitch / (2 * arm)
        dT_yaw = M_yaw / (2 * k_yaw)
        
        T1 = T_avg - dT_roll - dT_pitch + dT_yaw  # Front-right
        T2 = T_avg - dT_roll + dT_pitch - dT_yaw  # Front-left
        T3 = T_avg + dT_roll - dT_pitch - dT_yaw  # Back-left
        T4 = T_avg + dT_roll + dT_pitch + dT_yaw  # Back-right
        
        thrusts = np.array([T1, T2, T3, T4])
        thrusts = np.clip(thrusts, T_min, T_max)
        
        return thrusts


def main():
    print("=" * 80)
    print("QUADROCOPTER 3D SIMULATION - PID CONTROLLER")
    print("=" * 80)
    
    # Simulation parameters
    dt = 0.01
    total_time = 50.0
    
    # Initial state: [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
    
    # Initial control
    T_hover = MASS * g / 4.0
    u = np.array([T_hover, T_hover, T_hover, T_hover])
    
    # Generate 3D trajectory
    print("\nGenerating 3D trajectory...")
    t_traj, X_traj, Y_traj, Z_traj, Z_terr, Vx_traj, Vy_traj, Vz_traj = generate_3d_trajectory(
        Vel, dt, total_time
    )
    print(f"Generated {len(t_traj)} trajectory points")
    print(f"  X range: {X_traj.min():.1f} to {X_traj.max():.1f} m")
    print(f"  Y range: {Y_traj.min():.1f} to {Y_traj.max():.1f} m")
    print(f"  Z range: {Z_traj.min():.1f} to {Z_traj.max():.1f} m")
    
    # Initialize controller
    controller = QuadcopterPIDController()
    
    # Storage
    tp, yp, up = [], [], []
    
    # Setup plots
    plt.ion()
    fig = plt.figure(figsize=(18, 10))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Trajectory')
    ax1.view_init(elev=25, azim=45)  # Fixed view angle
    
    # Z(X) profile
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Z [m]')
    ax2.set_title('Altitude Profile Z(X)')
    ax2.grid(True, alpha=0.3)
    
    # Y(X) profile
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.set_xlabel('X [m]')
    ax3.set_ylabel('Y [m]')
    ax3.set_title('Lateral Profile Y(X)')
    ax3.grid(True, alpha=0.3)
    
    # Time histories
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Position [m]')
    ax4.set_title('Positions vs Time')
    ax4.grid(True, alpha=0.3)
    
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Angle [°]')
    ax5.set_title('Attitude Angles')
    ax5.grid(True, alpha=0.3)
    
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Thrust [N]')
    ax6.set_title('Motor Thrusts')
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.pause(0.1)
    
    # Main simulation loop
    print("\nStarting simulation...")
    print("-" * 80)
    
    for i in range(len(t_traj) - 1):
        t = t_traj[i]
        
        # Get reference
        x_ref = X_traj[i]
        y_ref = Y_traj[i]
        z_ref = Z_traj[i]
        vx_ref = Vx_traj[i]
        vy_ref = Vy_traj[i]
        vz_ref = Vz_traj[i]
        
        # Store data
        tp.append(t)
        yp.append(x.copy())
        up.append(u.copy())
        
        # Compute control
        ref_pos = np.array([x_ref, y_ref, z_ref])
        ref_vel = np.array([vx_ref, vy_ref, vz_ref])
        u = controller.compute_thrust(x, ref_pos, ref_vel)
        
        # Print progress
        if i % 1000 == 0:
            e_pos = np.sqrt((x[6]-x_ref)**2 + (x[7]-y_ref)**2 + (x[8]-z_ref)**2)
            print(f"t={t:6.2f}s | Pos: X={x[6]:6.2f} Y={x[7]:6.2f} Z={x[8]:6.2f} | Error: {e_pos:5.3f}m")
        
        # Integrate
        x = rk45_simple(x, t, dt, u, 0.0, 0.0, 0.0)
        
        # Update plots
        if i % 100 == 0:
            yp_array = np.array(yp)
            up_array = np.array(up)
            
            # 3D trajectory
            ax1.clear()
            ax1.plot(X_traj, Y_traj, Z_traj, 'r--', label='Reference', linewidth=2, alpha=0.5)
            ax1.plot(yp_array[:, 6], yp_array[:, 7], yp_array[:, 8], 'b-', label='Actual', linewidth=2)
            ax1.scatter([x[6]], [x[7]], [x[8]], c='blue', s=100, marker='o')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Y [m]')
            ax1.set_zlabel('Z [m]')
            ax1.set_title(f'3D Trajectory (t={t:.1f}s)')
            ax1.legend()
            ax1.view_init(elev=25, azim=45)  # Fixed view
            
            # Z(X) profile
            ax2.clear()
            ax2.fill_between(X_traj, 0, Z_terr, color='brown', alpha=0.3, label='Terrain')
            ax2.plot(X_traj, Z_traj, 'r--', label='Reference', linewidth=2)
            ax2.plot(yp_array[:, 6], yp_array[:, 8], 'b-', label='Actual', linewidth=2)
            ax2.scatter([x[6]], [x[8]], c='blue', s=100, marker='o')
            ax2.set_xlabel('X [m]')
            ax2.set_ylabel('Z [m]')
            ax2.set_title('Altitude Profile Z(X)')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            ax2.set_xlim([0, 50])
            ax2.set_ylim([0, 12])
            
            # Y(X) profile
            ax3.clear()
            ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            ax3.plot(X_traj, Y_traj, 'r--', label='Reference', linewidth=2)
            ax3.plot(yp_array[:, 6], yp_array[:, 7], 'b-', label='Actual', linewidth=2)
            ax3.scatter([x[6]], [x[7]], c='blue', s=100, marker='o')
            ax3.set_xlabel('X [m]')
            ax3.set_ylabel('Y [m]')
            ax3.set_title('Lateral Profile Y(X)')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            ax3.set_xlim([0, 50])
            ax3.set_ylim([-5, 5])
            
            # Positions vs time
            ax4.clear()
            ax4.plot(tp, yp_array[:, 6], 'r-', label='X', linewidth=2)
            ax4.plot(tp, yp_array[:, 7], 'g-', label='Y', linewidth=2)
            ax4.plot(tp, yp_array[:, 8], 'b-', label='Z', linewidth=2)
            ax4.set_xlabel('Time [s]')
            ax4.set_ylabel('Position [m]')
            ax4.set_title('Positions vs Time')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
            ax4.set_xlim([0, total_time])
            
            # Attitude angles
            ax5.clear()
            ax5.plot(tp, np.degrees(yp_array[:, 9]), 'r-', label='φ (roll)', linewidth=2)
            ax5.plot(tp, np.degrees(yp_array[:, 10]), 'g-', label='θ (pitch)', linewidth=2)
            ax5.plot(tp, np.degrees(yp_array[:, 11]), 'b-', label='ψ (yaw)', linewidth=2)
            ax5.set_xlabel('Time [s]')
            ax5.set_ylabel('Angle [°]')
            ax5.set_title('Attitude Angles')
            ax5.legend()
            ax5.grid(True, alpha=0.3)
            ax5.set_xlim([0, total_time])
            
            # Motor thrusts
            ax6.clear()
            ax6.plot(tp, up_array[:, 0], label='T1', linewidth=1)
            ax6.plot(tp, up_array[:, 1], label='T2', linewidth=1)
            ax6.plot(tp, up_array[:, 2], label='T3', linewidth=1)
            ax6.plot(tp, up_array[:, 3], label='T4', linewidth=1)
            ax6.axhline(y=MASS*g/4, color='k', linestyle='--', alpha=0.5, label='Hover')
            ax6.set_xlabel('Time [s]')
            ax6.set_ylabel('Thrust [N]')
            ax6.set_title('Motor Thrusts')
            ax6.legend()
            ax6.grid(True, alpha=0.3)
            ax6.set_xlim([0, total_time])
            
            plt.draw()
            plt.pause(0.001)
    
    # Final statistics
    print("\n" + "=" * 80)
    print("SIMULATION COMPLETED")
    print("=" * 80)
    
    yp_array = np.array(yp)
    e_x = X_traj[:len(tp)] - yp_array[:, 6]
    e_y = Y_traj[:len(tp)] - yp_array[:, 7]
    e_z = Z_traj[:len(tp)] - yp_array[:, 8]
    e_pos = np.sqrt(e_x**2 + e_y**2 + e_z**2)
    
    print(f"\nTracking Errors:")
    print(f"  X: mean={np.mean(np.abs(e_x)):.3f}m, max={np.max(np.abs(e_x)):.3f}m")
    print(f"  Y: mean={np.mean(np.abs(e_y)):.3f}m, max={np.max(np.abs(e_y)):.3f}m")
    print(f"  Z: mean={np.mean(np.abs(e_z)):.3f}m, max={np.max(np.abs(e_z)):.3f}m")
    print(f"  3D: mean={np.mean(e_pos):.3f}m, max={np.max(e_pos):.3f}m")
    
    print(f"\nAttitude:")
    print(f"  Max roll: {math.degrees(np.max(np.abs(yp_array[:, 9]))):.1f}°")
    print(f"  Max pitch: {math.degrees(np.max(np.abs(yp_array[:, 10]))):.1f}°")
    
    print("\n" + "=" * 80)
    
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
