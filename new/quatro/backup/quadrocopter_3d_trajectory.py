import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from integrator import rk45_simple
from trajectory import generate_3d_trajectory
from constants import (z0, Vel, MASS, g, T_max, T_min, MOTOR_ARM_LENGTH, IX, IY, IZ)

# ============================================================================
# SIMPLIFIED PID-LIKE CONTROLLER (3D version)
# ============================================================================

class Quad3DController:
    """3D Controller for quadrocopter with lateral maneuvers"""
    
    def __init__(self):
        # Altitude control gains
        self.kp_z = 25.0    # Position gain (increased for 3D)
        self.kd_z = 12.0    # Velocity gain
        
        # Horizontal position gains (X and Y)
        self.kp_x = 3.0     # Forward position
        self.kd_x = 4.0     # Forward velocity
        
        self.kp_y = 3.0     # Lateral position (same as forward)
        self.kd_y = 4.0     # Lateral velocity
        
        # Attitude gains
        self.kp_att = 6.0   # Angle gain (increased for aggressive maneuvers)
        self.kd_att = 3.0   # Rate gain
        
        # Yaw gain (keep heading aligned with velocity)
        self.kp_yaw = 3.0
        self.kd_yaw = 1.5
        
    def compute_thrust(self, state, ref_pos, ref_vel):
        """
        Compute motor thrusts for 3D trajectory tracking
        
        Args:
            state: current state [vx,vy,vz,p,q,r,X,Y,Z,phi,theta,psi]
            ref_pos: reference position [x_ref, y_ref, z_ref]
            ref_vel: reference velocity [vx_ref, vy_ref, vz_ref]
        """
        # Extract state
        vx, vy, vz = state[0:3]
        p, q, r = state[3:6]
        X, Y, Z = state[6:9]
        phi, theta, psi = state[9:12]
        
        x_ref, y_ref, z_ref = ref_pos
        vx_ref, vy_ref, vz_ref = ref_vel
        
        # ====== ALTITUDE CONTROL ======
        e_z = z_ref - Z
        e_vz = vz_ref - vz
        
        # Total thrust command
        T_total = MASS * g + self.kp_z * e_z + self.kd_z * e_vz
        T_total = np.clip(T_total, MASS * g * 0.3, MASS * g * 2.5)
        
        # ====== HORIZONTAL POSITION CONTROL (3D!) ======
        # Position errors
        e_x = x_ref - X
        e_y = y_ref - Y
        
        # Velocity errors
        e_vx = vx_ref - vx
        e_vy = vy_ref - vy
        
        # Desired angles for position tracking
        # IMPORTANT: In body frame (X-configuration)
        # - To move forward (+X): pitch forward (theta < 0 in NED, but theta > 0 in our convention)
        # - To move right (+Y): roll right (phi > 0)
        
        theta_des = np.clip(
            self.kp_x * e_x + self.kd_x * e_vx,
            -0.4, 0.4  # ±23° max
        )
        
        phi_des = np.clip(
            self.kp_y * e_y + self.kd_y * e_vy,
            -0.4, 0.4  # ±23° max
        )
        
        # Desired yaw - align with velocity direction
        if abs(vx) > 0.5 or abs(vy) > 0.5:
            psi_des = np.arctan2(vy, vx)
        else:
            psi_des = 0.0
        
        # ====== ATTITUDE CONTROL ======
        # Angle errors
        e_phi = phi_des - phi
        e_theta = theta_des - theta
        e_psi = psi_des - psi
        
        # Wrap yaw error to [-pi, pi]
        e_psi = np.arctan2(np.sin(e_psi), np.cos(e_psi))
        
        # Desired moments
        M_roll = self.kp_att * e_phi - self.kd_att * p
        M_pitch = self.kp_att * e_theta - self.kd_att * q
        M_yaw = self.kp_yaw * e_psi - self.kd_yaw * r
        
        # Limit moments
        M_roll = np.clip(M_roll, -15.0, 15.0)
        M_pitch = np.clip(M_pitch, -15.0, 15.0)
        M_yaw = np.clip(M_yaw, -3.0, 3.0)
        
        # ====== MOTOR MIXING (X-configuration) ======
        arm = MOTOR_ARM_LENGTH / np.sqrt(2)
        k_yaw = 0.1
        
        T_avg = T_total / 4.0
        
        # Convert moments to thrust differentials
        dT_roll = M_roll / (2 * arm)
        dT_pitch = M_pitch / (2 * arm)
        dT_yaw = M_yaw / (2 * k_yaw)
        
        # Individual motor thrusts (X-configuration)
        #     1(FR)      2(FL)
        #        \      /
        #         \    /
        #          \  /
        #    ------()------
        #          /  \
        #         /    \
        #        /      \
        #     4(BR)      3(BL)
        
        T1 = T_avg - dT_roll - dT_pitch + dT_yaw  # Front-right
        T2 = T_avg - dT_roll + dT_pitch - dT_yaw  # Front-left
        T3 = T_avg + dT_roll - dT_pitch - dT_yaw  # Back-left
        T4 = T_avg + dT_roll + dT_pitch + dT_yaw  # Back-right
        
        thrusts = np.array([T1, T2, T3, T4])
        thrusts = np.clip(thrusts, T_min, T_max)
        
        return thrusts

# ============================================================================
# CONFIGURATION
# ============================================================================

dt = 0.01
t = 0.0
total_time = 50.0

# ============================================================================
# INITIAL STATE
# ============================================================================

x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
T_hover = MASS * g / 4.0
u = np.array([T_hover, T_hover, T_hover, T_hover])

# ============================================================================
# GENERATE 3D REFERENCE TRAJECTORY
# ============================================================================

print("=" * 80)
print("Generating 3D trajectory with lateral maneuvers...")
print("=" * 80)

t_traj, X_traj, Y_traj, Z_traj, Vx_traj, Vy_traj, Vz_traj = generate_3d_trajectory(
    Vel, dt, total_time
)

print(f"Trajectory generated: {len(t_traj)} points")
print(f"X range: {X_traj.min():.1f} to {X_traj.max():.1f} m")
print(f"Y range: {Y_traj.min():.1f} to {Y_traj.max():.1f} m")
print(f"Z range: {Z_traj.min():.1f} to {Z_traj.max():.1f} m")

# Storage
tp = []
yp = []
up = []

# ============================================================================
# CONTROLLER
# ============================================================================

controller = Quad3DController()

# ============================================================================
# PLOTTING SETUP - 3D!
# ============================================================================

plt.ion()
fig = plt.figure(figsize=(16, 8))

# 3D trajectory view
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot(X_traj, Y_traj, Z_traj, 'r--', label='Reference', linewidth=2, alpha=0.7)
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_zlabel('Z [m]')
ax1.set_xlim([0, 50])
ax1.set_ylim([-10, 10])
ax1.set_zlim([0, 15])
ax1.legend()
ax1.view_init(elev=20, azim=45)
ax1.grid(True, alpha=0.3)

# Time history plots
ax2 = fig.add_subplot(322)
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('X position [m]')
ax2.grid(True, alpha=0.3)

ax3 = fig.add_subplot(324)
ax3.set_xlabel('Time [s]')
ax3.set_ylabel('Y position [m]')
ax3.grid(True, alpha=0.3)

ax4 = fig.add_subplot(326)
ax4.set_xlabel('Time [s]')
ax4.set_ylabel('Z altitude [m]')
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.draw()
plt.pause(0.5)

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================

print("\n" + "=" * 80)
print("QUADROCOPTER 3D SIMULATION - PID CONTROLLER")
print("=" * 80)

step_count = 0

for i in range(len(t_traj)):
    if i >= len(t_traj) - 1:
        break
    
    t = t_traj[i]
    
    # Get reference from trajectory
    x_ref = X_traj[i]
    y_ref = Y_traj[i]
    z_ref = Z_traj[i]
    
    vx_ref = Vx_traj[i]
    vy_ref = Vy_traj[i]
    vz_ref = Vz_traj[i]
    
    # === Store data ===
    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())
    
    # === Control ===
    ref_pos = np.array([x_ref, y_ref, z_ref])
    ref_vel = np.array([vx_ref, vy_ref, vz_ref])
    u = controller.compute_thrust(x, ref_pos, ref_vel)
    
    # === Diagnostics ===
    if step_count % 100 == 0:
        print(f"\nt={t:7.3f}s")
        print(f"Position: X={x[6]:7.2f} Y={x[7]:7.2f} Z={x[8]:7.2f}")
        print(f"Reference: X={x_ref:7.2f} Y={y_ref:7.2f} Z={z_ref:7.2f}")
        print(f"Velocity: vx={x[0]:7.3f} vy={x[1]:7.3f} vz={x[2]:7.3f}")
        print(f"Attitude: phi={math.degrees(x[9]):7.2f}° theta={math.degrees(x[10]):7.2f}° psi={math.degrees(x[11]):7.2f}°")
        print(f"Thrusts: T1={u[0]:7.1f}N T2={u[1]:7.1f}N T3={u[2]:7.1f}N T4={u[3]:7.1f}N (sum={u.sum():7.1f}N)")
        
        # Error metrics
        e_pos = np.sqrt((x[6]-x_ref)**2 + (x[7]-y_ref)**2 + (x[8]-z_ref)**2)
        print(f"Position error: {e_pos:7.3f}m")
    
    # === Integrate ===
    x = rk45_simple(x, t, dt, u, 0.0, 0.0, 0.0)
    
    # === Visualization (every 50 steps for performance) ===
    if step_count % 50 == 0:
        V = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
        
        yp_array = np.array(yp)
        
        # Update 3D trajectory
        ax1.clear()
        ax1.plot(X_traj, Y_traj, Z_traj, 'r--', label='Reference', linewidth=2, alpha=0.5)
        ax1.plot(yp_array[:, 6], yp_array[:, 7], yp_array[:, 8], 
                'b-', label='Flight path', linewidth=2)
        ax1.scatter([x[6]], [x[7]], [x[8]], 
                   c='blue', s=200, marker='o', label='Drone', edgecolors='black', linewidth=2)
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_zlabel('Z [m]')
        ax1.set_xlim([0, 50])
        ax1.set_ylim([-10, 10])
        ax1.set_zlim([0, 15])
        ax1.legend()
        ax1.view_init(elev=20, azim=45 + t * 0.5)  # Slowly rotating view
        ax1.grid(True, alpha=0.3)
        ax1.set_title(f't={t:7.3f}s | V={V:5.2f}m/s')
        
        # Update X position plot
        ax2.clear()
        ax2.plot(tp, yp_array[:, 6], 'b-', label='Actual X', linewidth=2)
        ax2.plot(t_traj[:len(tp)], X_traj[:len(tp)], 'r--', label='Reference', alpha=0.7)
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('X position [m]')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.set_xlim([0, total_time])
        
        # Update Y position plot
        ax3.clear()
        ax3.plot(tp, yp_array[:, 7], 'b-', label='Actual Y', linewidth=2)
        ax3.plot(t_traj[:len(tp)], Y_traj[:len(tp)], 'r--', label='Reference', alpha=0.7)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Y position [m]')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        ax3.set_xlim([0, total_time])
        
        # Update Z altitude plot
        ax4.clear()
        ax4.plot(tp, yp_array[:, 8], 'b-', label='Actual Z', linewidth=2)
        ax4.plot(t_traj[:len(tp)], Z_traj[:len(tp)], 'r--', label='Reference', alpha=0.7)
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Z altitude [m]')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        ax4.set_xlim([0, total_time])
        
        plt.draw()
        plt.pause(0.001)
    
    step_count += 1

# ============================================================================
# FINAL STATISTICS
# ============================================================================

print("\n" + "=" * 80)
print("SIMULATION COMPLETED!")
print("=" * 80)

yp_array = np.array(yp)

# Calculate tracking errors
e_x = X_traj[:len(tp)] - yp_array[:, 6]
e_y = Y_traj[:len(tp)] - yp_array[:, 7]
e_z = Z_traj[:len(tp)] - yp_array[:, 8]

e_pos_3d = np.sqrt(e_x**2 + e_y**2 + e_z**2)

print(f"\nTracking Performance:")
print(f"  X error: mean={np.mean(np.abs(e_x)):.3f}m, max={np.max(np.abs(e_x)):.3f}m")
print(f"  Y error: mean={np.mean(np.abs(e_y)):.3f}m, max={np.max(np.abs(e_y)):.3f}m")
print(f"  Z error: mean={np.mean(np.abs(e_z)):.3f}m, max={np.max(np.abs(e_z)):.3f}m")
print(f"  3D position error: mean={np.mean(e_pos_3d):.3f}m, max={np.max(e_pos_3d):.3f}m")

# Angle statistics
print(f"\nAttitude Statistics:")
print(f"  Roll (phi): max={math.degrees(np.max(np.abs(yp_array[:, 9]))):.1f}°")
print(f"  Pitch (theta): max={math.degrees(np.max(np.abs(yp_array[:, 10]))):.1f}°")
print(f"  Yaw (psi): range=[{math.degrees(np.min(yp_array[:, 11])):.1f}°, {math.degrees(np.max(yp_array[:, 11])):.1f}°]")

print("\n" + "=" * 80)

plt.ioff()
plt.show()
