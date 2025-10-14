import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from integrator import rk45_simple
from linearization import linearize_system_simple
from lqr_controller import design_lqr
from trajectory import generate_3d_trajectory
from constants import (z0, Vel, MASS, g, T_max, T_min, MOTOR_ARM_LENGTH, IX, IY, IZ)

# ============================================================================
# CONFIGURATION
# ============================================================================

n = 12  # State dimension
m = 4   # Control inputs (4 motors)

dt = 0.01
t = 0.0
total_time = 50.0

# ============================================================================
# INITIAL STATE
# ============================================================================

x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
T_hover = MASS * g / 4.0
u = np.array([T_hover, T_hover, T_hover, T_hover])

# Initialize K matrix
K = None

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
print("QUADROCOPTER 3D SIMULATION - STABILIZED LQR CONTROLLER")
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
    
    # ========================================================================
    # LQR TUNING - Conservative for 3D maneuvers
    # ========================================================================
    
    # High control penalty for stability
    R = np.eye(m) * 15.0  # Even higher for 3D maneuvers
    
    Q = np.eye(n)
    # Velocities - moderate weight
    Q[0, 0] = 3.0    # vx (increased for trajectory tracking)
    Q[1, 1] = 3.0    # vy (IMPORTANT for lateral maneuvers!)
    Q[2, 2] = 5.0    # vz
    
    # Angular rates - very low weight
    Q[3, 3] = 0.1    # p
    Q[4, 4] = 0.1    # q
    Q[5, 5] = 0.1    # r
    
    # Position - moderate weight
    Q[6, 6] = 2.0    # X position
    Q[7, 7] = 2.0    # Y position (IMPORTANT!)
    Q[8, 8] = 50.0   # Z position - critical!
    
    # Angles - HIGH weight to prevent flipping
    Q[9, 9] = 25.0   # phi (roll) - critical for Y control
    Q[10, 10] = 25.0 # theta (pitch) - critical for X control
    Q[11, 11] = 5.0  # psi (yaw)
    
    # ========================================================================
    # ERROR CALCULATION
    # ========================================================================
    
    e = np.zeros(n)
    e[0] = x[0] - vx_ref
    e[1] = x[1] - vy_ref
    e[2] = x[2] - vz_ref
    e[3] = x[3] - 0.0  # p
    e[4] = x[4] - 0.0  # q
    e[5] = x[5] - 0.0  # r
    e[6] = x[6] - x_ref
    e[7] = x[7] - y_ref
    e[8] = x[8] - z_ref
    e[9] = x[9] - 0.0  # phi
    e[10] = x[10] - 0.0  # theta
    e[11] = x[11] - 0.0  # psi
    
    # ========================================================================
    # SAFETY CHECK - Reset if angles too large
    # ========================================================================
    
    max_angle = math.radians(50)  # 50 degrees max for 3D maneuvers
    if abs(x[9]) > max_angle or abs(x[10]) > max_angle:
        print(f"\n⚠️  WARNING: Large angles detected at t={t:.3f}s")
        print(f"   phi={math.degrees(x[9]):.1f}°, theta={math.degrees(x[10]):.1f}°")
        print("   Increasing angle penalties...")
        Q[9, 9] = 100.0
        Q[10, 10] = 100.0
    
    # ========================================================================
    # DIAGNOSTICS (every 100 steps)
    # ========================================================================
    
    if step_count % 100 == 0:
        T_sum = u.sum()
        print(f"\nt={t:7.3f}s")
        print(f"Position: X={x[6]:7.2f} Y={x[7]:7.2f} Z={x[8]:7.2f}")
        print(f"Reference: X={x_ref:7.2f} Y={y_ref:7.2f} Z={z_ref:7.2f}")
        print(f"Velocity: vx={x[0]:7.3f} vy={x[1]:7.3f} vz={x[2]:7.3f}")
        print(f"Attitude: phi={math.degrees(x[9]):7.2f}° theta={math.degrees(x[10]):7.2f}° psi={math.degrees(x[11]):7.2f}°")
        print(f"Thrusts: T1={u[0]:7.1f}N T2={u[1]:7.1f}N T3={u[2]:7.1f}N T4={u[3]:7.1f}N (sum={T_sum:7.1f}N)")
        
        # Error metrics
        e_pos = np.sqrt(e[6]**2 + e[7]**2 + e[8]**2)
        print(f"Position error: {e_pos:7.3f}m (ΔX={e[6]:.2f} ΔY={e[7]:.2f} ΔZ={e[8]:.2f})")
    
    # ========================================================================
    # LQR CONTROLLER (recompute every 10 steps)
    # ========================================================================
    
    if step_count % 10 == 0:
        try:
            A, B = linearize_system_simple(x, t, u, n, m)
            K_new, P = design_lqr(A, B, Q, R)
            
            # Only update if successful
            if K_new is not None and not np.any(np.isnan(K_new)):
                K = K_new
            else:
                print("⚠️  LQR returned NaN, keeping previous gain")
        except Exception as ex:
            print(f"⚠️  LQR failed: {ex}, keeping previous gain")
            if K is None:
                # Emergency: use zero gain
                K = np.zeros((m, n))
    
    # Compute control perturbations
    if K is not None:
        u_pert = -K @ e
        
        # CRITICAL: Limit perturbations
        max_pert = 60.0  # Maximum 60N perturbation per motor (higher for 3D)
        u_pert = np.clip(u_pert, -max_pert, max_pert)
    else:
        u_pert = np.zeros(m)
    
    # Compute desired thrusts with baseline
    T_baseline = MASS * g / 4.0
    T1_desired = T_baseline + u_pert[0]
    T2_desired = T_baseline + u_pert[1]
    T3_desired = T_baseline + u_pert[2]
    T4_desired = T_baseline + u_pert[3]
    
    # Apply physical limits
    u = np.array([
        np.clip(T1_desired, T_min, T_max),
        np.clip(T2_desired, T_min, T_max),
        np.clip(T3_desired, T_min, T_max),
        np.clip(T4_desired, T_min, T_max)
    ])
    
    # ========================================================================
    # INTEGRATE STATE
    # ========================================================================
    
    x = rk45_simple(x, t, dt, u, 0.0, 0.0, 0.0)
    
    # ========================================================================
    # VISUALIZATION (every 50 steps)
    # ========================================================================
    
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
        ax1.set_title(f't={t:7.3f}s | V={V:5.2f}m/s | LQR')
        
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

# Control effort
up_array = np.array(up)
print(f"\nControl Statistics:")
print(f"  Mean total thrust: {np.mean(np.sum(up_array, axis=1)):.1f}N")
print(f"  Max single motor: {np.max(up_array):.1f}N")
print(f"  Min single motor: {np.min(up_array):.1f}N")

print("\n" + "=" * 80)

plt.ioff()
plt.show()
