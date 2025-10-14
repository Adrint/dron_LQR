import numpy as np
import math
import matplotlib.pyplot as plt
from integrator import rk45_simple
from trajectory import generate_reference_profile
from constants import (z0, Vel, MASS, g, T_max, T_min, MOTOR_ARM_LENGTH, IX, IY, IZ)

# ============================================================================
# SIMPLIFIED PID-LIKE CONTROLLER (More stable than pure LQR)
# ============================================================================

class SimpleQuadController:
    """Simplified controller based on bicopter principles but adapted for quad"""
    
    def __init__(self):
        # Altitude control gains
        self.kp_z = 20.0    # Position gain
        self.kd_z = 10.0    # Velocity gain
        
        # Horizontal position gains  
        self.kp_xy = 2.0
        self.kd_xy = 3.0
        
        # Attitude gains
        self.kp_att = 5.0   # Angle gain
        self.kd_att = 2.0   # Rate gain
        
        # Yaw gain
        self.kp_yaw = 2.0
        self.kd_yaw = 1.0
        
    def compute_thrust(self, state, ref_pos):
        """
        Compute motor thrusts using simplified control law
        Similar to bicopter but for quadrocopter
        """
        # Extract state
        vx, vy, vz = state[0:3]
        p, q, r = state[3:6]
        X, Y, Z = state[6:9]
        phi, theta, psi = state[9:12]
        
        x_ref, y_ref, z_ref = ref_pos
        
        # ====== ALTITUDE CONTROL (most important!) ======
        e_z = z_ref - Z
        e_vz = 0.0 - vz  # Want zero vertical velocity at setpoint
        
        # Thrust command (baseline + correction)
        T_total = MASS * g + self.kp_z * e_z + self.kd_z * e_vz
        T_total = np.clip(T_total, MASS * g * 0.3, MASS * g * 2.5)
        
        # ====== HORIZONTAL POSITION CONTROL ======
        # Position error
        e_x = x_ref - X
        e_y = y_ref - Y
        
        # Desired angles (small angles assumption)
        # To move forward (+X): pitch forward (theta > 0)
        # To move right (+Y): roll right (phi > 0)
        theta_des = np.clip(self.kp_xy * e_x - self.kd_xy * vx, -0.3, 0.3)  # ±17°
        phi_des = np.clip(self.kp_xy * e_y - self.kd_xy * vy, -0.3, 0.3)
        psi_des = 0.0  # Keep yaw at zero
        
        # ====== ATTITUDE CONTROL ======
        # Angle errors
        e_phi = phi_des - phi
        e_theta = theta_des - theta
        e_psi = psi_des - psi
        
        # Desired moments
        M_roll = self.kp_att * e_phi - self.kd_att * p
        M_pitch = self.kp_att * e_theta - self.kd_att * q
        M_yaw = self.kp_yaw * e_psi - self.kd_yaw * r
        
        # Limit moments
        M_roll = np.clip(M_roll, -10.0, 10.0)
        M_pitch = np.clip(M_pitch, -10.0, 10.0)
        M_yaw = np.clip(M_yaw, -2.0, 2.0)
        
        # ====== MOTOR MIXING (X-configuration) ======
        arm = MOTOR_ARM_LENGTH / np.sqrt(2)
        k_yaw = 0.1
        
        T_avg = T_total / 4.0
        
        # Convert moments to thrust differentials
        dT_roll = M_roll / (2 * arm)
        dT_pitch = M_pitch / (2 * arm)
        dT_yaw = M_yaw / (2 * k_yaw)
        
        # Individual motor thrusts
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

# ============================================================================
# INITIAL STATE
# ============================================================================

x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
T_hover = MASS * g / 4.0
u = np.array([T_hover, T_hover, T_hover, T_hover])

# ============================================================================
# REFERENCE TRAJECTORY
# ============================================================================

X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)

# Storage
tp = []
yp = []
up = []

# ============================================================================
# CONTROLLER
# ============================================================================

controller = SimpleQuadController()

# ============================================================================
# PLOTTING SETUP
# ============================================================================

plt.ion()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

ax1.plot(X_ref_all, Z_ref_all, 'r-', label='Reference', linewidth=2, alpha=0.7)
ax1.plot(X_ref_all, Z_terr_all, 'g--', label='Terrain', linewidth=2, alpha=0.7)
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Z [m]')
ax1.set_xlim([0, 50])
ax1.set_ylim([0, 15])
ax1.grid(True, alpha=0.3)
ax1.legend()

ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Altitude Z [m]')
ax2.grid(True, alpha=0.3)

plt.draw()
plt.pause(0.5)

x_ref = 0.0
y_ref = 0.0

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================

print("=" * 80)
print("QUADROCOPTER SIMULATION - SIMPLE PID CONTROLLER")
print("(Based on working bicopter approach)")
print("=" * 80)

for i in range(100000):
    if t >= 50.0:
        break

    X = x[6]

    # === Find reference point ===
    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]

    # === Reference velocity ===
    Vx_ref = Vel * np.cos(alfa)
    x_ref += Vx_ref * dt
    y_ref = 0.0

    # === Store data ===
    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    # === Control ===
    ref_pos = np.array([x_ref, y_ref, z_ref])
    u = controller.compute_thrust(x, ref_pos)

    # === Diagnostics ===
    if i % 100 == 0:
        print(f"\nt={t:7.3f}s")
        print(f"Position: X={x[6]:7.2f} Y={x[7]:7.2f} Z={x[8]:7.2f} (ref: Z={z_ref:7.2f})")
        print(f"Velocity: vx={x[0]:7.3f} vy={x[1]:7.3f} vz={x[2]:7.3f}")
        print(f"Attitude: phi={math.degrees(x[9]):7.2f}° theta={math.degrees(x[10]):7.2f}° psi={math.degrees(x[11]):7.2f}°")
        print(f"Thrusts: T1={u[0]:7.1f}N T2={u[1]:7.1f}N T3={u[2]:7.1f}N T4={u[3]:7.1f}N (sum={u.sum():7.1f}N)")
        print(f"Error: ΔZ={z_ref - x[8]:7.3f}m")

    # === Integrate ===
    x = rk45_simple(x, t, dt, u, 0.0, 0.0, 0.0)

    # === Visualization ===
    if i % 50 == 0:
        V = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
        alt = x[8]

        ax1.clear()
        ax1.plot(X_ref_all, Z_ref_all, 'r-', label='Reference', linewidth=2, alpha=0.7)
        ax1.plot(X_ref_all, Z_terr_all, 'g--', label='Terrain', linewidth=2, alpha=0.7)

        yp_array = np.array(yp)
        ax1.plot(yp_array[:, 6], yp_array[:, 8], 'b-', label='Flight path', linewidth=1.5)
        ax1.scatter([x[6]], [x[8]], c='blue', s=150, marker='o', label='Drone')

        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Z [m]')
        ax1.set_xlim([0, 50])
        ax1.set_ylim([0, 15])
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        ax1.set_title(f't={t:7.3f}s | V={V:5.2f}m/s | alt={alt:6.2f}m')

        ax2.clear()
        if len(yp_array) > 0:
            ax2.plot(tp, yp_array[:, 8], 'b-', label='Altitude', linewidth=1.5)
            ax2.axhline(y=z_ref, color='r', linestyle='--', alpha=0.5, label='Reference')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Z [m]')
        ax2.set_xlim([0, 10])
        ax2.set_ylim([0, 10])
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        plt.draw()
        plt.pause(0.001)

    t = t + dt

print("\n" + "=" * 80)
print("Simulation completed!")
print("=" * 80)

plt.ioff()
plt.show()
