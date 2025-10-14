import numpy as np
import math
import matplotlib.pyplot as plt
from integrator import rk45_simple
from linearization import linearize_system_simple
from lqr_controller import design_lqr
from trajectory import generate_reference_profile
from constants import (z0, Vel, MASS, g, T_max, T_min, MOTOR_ARM_LENGTH, IX, IY, IZ)

# ============================================================================
# CONFIGURATION
# ============================================================================

# State dimension
n = 12  # [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
m = 4   # Four motor thrusts

# Integration parameters
dt = 0.01
t = 0.0

# ============================================================================
# INITIAL STATE
# ============================================================================

# State: [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])

# Control input - hover equilibrium
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
print("QUADROCOPTER SIMULATION - SIMPLE LQR (Based on Working Bicopter)")
print("=" * 80)

for i in range(100000):
    if t >= 10.0:
        break

    X = x[6]  # Current horizontal position

    # === 1. Find reference point ===
    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]

    # === 2. Reference velocity ===
    Vx_ref = Vel * np.cos(alfa)
    Vz_ref = Vel * np.sin(alfa)
    Vy_ref = 0.0

    # === 3. Update reference position ===
    x_ref += Vx_ref * dt
    y_ref += 0.0  # No lateral movement

    # === 4. Collect data ===
    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    # ========================================================================
    # LQR TUNING - Conservative gains similar to bicopter
    # ========================================================================

    R = np.eye(m) * 1.0  # Control penalty

    Q = np.eye(n)
    Q[0, 0] = 10.0   # vx
    Q[1, 1] = 10.0   # vy
    Q[2, 2] = 10.0   # vz
    Q[3, 3] = 1.0    # p (roll rate)
    Q[4, 4] = 1.0    # q (pitch rate)
    Q[5, 5] = 1.0    # r (yaw rate)
    Q[6, 6] = 5.0    # X position
    Q[7, 7] = 5.0    # Y position
    Q[8, 8] = 100.0  # Z position - MOST IMPORTANT!
    Q[9, 9] = 5.0    # phi (roll)
    Q[10, 10] = 5.0  # theta (pitch)
    Q[11, 11] = 2.0  # psi (yaw)

    # ========================================================================
    # ERROR CALCULATION
    # ========================================================================

    e = np.zeros(n)
    e[0] = x[0] - Vx_ref
    e[1] = x[1] - Vy_ref
    e[2] = x[2] - Vz_ref
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
    # DIAGNOSTICS (every 100 steps)
    # ========================================================================

    if i % 100 == 0:
        T_sum = u.sum()
        print(f"\nt={t:7.3f}s")
        print(f"Position: X={x[6]:7.2f} Y={x[7]:7.2f} Z={x[8]:7.2f} (ref: Z={z_ref:7.2f})")
        print(f"Velocity: vx={x[0]:7.3f} vy={x[1]:7.3f} vz={x[2]:7.3f}")
        print(f"Attitude: phi={math.degrees(x[9]):7.2f}° theta={math.degrees(x[10]):7.2f}° psi={math.degrees(x[11]):7.2f}°")
        print(f"Thrusts: T1={u[0]:7.1f}N T2={u[1]:7.1f}N T3={u[2]:7.1f}N T4={u[3]:7.1f}N (sum={T_sum:7.1f}N)")
        print(f"Errors: ΔZ={e[8]:7.3f}m Δvz={e[2]:7.3f}m/s")

    # ========================================================================
    # LQR CONTROLLER (recompute every 10 steps for efficiency)
    # ========================================================================

    if i % 10 == 0:
        A, B = linearize_system_simple(x, t, u, n, m)
        K, P = design_lqr(A, B, Q, R)

    # Compute control perturbations
    u_pert = -K @ e

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

    if i % 50 == 0:
        V = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
        alt = x[8]

        # Plot trajectory
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

        # Plot altitude over time
        ax2.clear()
        ax2.plot(tp, yp_array[:, 8], 'b-', label='Altitude', linewidth=1.5)
        ax2.axhline(y=z_ref, color='r', linestyle='--', alpha=0.5, label='Reference')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Z [m]')
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
