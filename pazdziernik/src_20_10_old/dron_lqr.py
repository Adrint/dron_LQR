import numpy as np
import math
import matplotlib.pyplot as plt
from rk45 import aa_rk45
from matrices import aa_matrices_AB
from model_draw import aa_mdl
from lqr import lqr_m
from trajectory import generate_reference_profile
from trajectory import generate_terrain_profile, generate_reference_from_terrain_segmentwise

# Global variables
from constants import (z0, Vel, X_turb_1, X_turb_2, c_turb, MASS, g, T_max, T_min, TAU)

# ============================================================================
# CONFIGURATION
# ============================================================================

# Choose model: n=6 (simple) or n=8 (with thrust dynamics)
n = 6  # Recommended: start with n=6
m = 2  # dimension of control input

# Integration parameters
dt = 0.01
t = 0.0

# ============================================================================
# INITIAL STATE
# ============================================================================

if n == 6:
    # State: [vx, vz, omega, X, Z, theta]
    x = np.array([0.0, 0.0, 0.0, 0.0, z0, 0.0])
elif n == 8:
    # State: [vx, vz, omega, X, Z, theta, Thrust_1, Thrust_2]
    x = np.array([0.0, 0.0, 0.0, 0.0, z0, 0.0, MASS * g / 2, MASS * g / 2])
else:
    raise ValueError("n must be 6 or 8")

# Control input
u = np.array([MASS * g / 2.0, MASS * g / 2.0])

# ============================================================================
# REFERENCE TRAJECTORY
# ============================================================================

#old
X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)

#new
#X_terr_all, Z_terr_all = generate_terrain_profile(Vel, dt, X_max=50)
#X_ref_all, Z_ref_all, alpha_all = generate_reference_from_terrain_segmentwise(X_terr_all, Z_terr_all, offset=2.0)


# Storage for plotting
tp = []
yp = []
up = []

# Terrain visualization data
terrain_x = []
terrain_z = []
ref_z = []

# ============================================================================
# PLOTTING SETUP
# ============================================================================

plt.ion()
fig = plt.figure(1, figsize=(11, 6.4))
plt.plot(X_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
plt.plot(X_ref_all, Z_terr_all, 'g', label='Terrain', linewidth=2)

plt.legend(fontsize=11)
plt.axis([0, 50.0, 0.0, 25.0])
plt.xlabel('X [m]')
plt.ylabel('Z [m]')
plt.title('Drone Flight Simulation')
plt.grid(True, alpha=0.3)
plt.draw()
plt.pause(1)

x_ref = 0.0
theta_ref = np.radians(-4)
# ============================================================================
# PHYSICAL THRUST LIMITS
# ============================================================================

T_baseline = MASS * g / 2.0  # Thrust baseline for equilibrium
T_max_absolute = T_max  # Maximum physical thrust
T_min_absolute = T_min  # Minimum physical thrust

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================

for i in range(100000):
    if t >= 100.0:  # Increased from 5.0 to 10.0 seconds
        break

    X = x[3]  # Current horizontal position

    # === 1. Find reference point ===
    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]

    # === 2. Reference velocity components ===
    Vx_ref = Vel * np.cos(alfa)
    Vz_ref = Vel * np.sin(alfa)

    # === 3. Update reference position ===
    x_ref += Vel * np.cos(alfa) * dt
    vx_ref = np.cos(x[5]) * Vx_ref + np.sin(x[5]) * Vz_ref
    vz_ref = -np.sin(x[5]) * Vx_ref + np.cos(x[5]) * Vz_ref

    # === 4. Collect data ===
    terrain_x.append(X)
    terrain_z.append(z_terr)
    ref_z.append(z_ref)

    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    # ========================================================================
    # LQR TUNING - Conservative Gains
    # ========================================================================

    R = np.eye(m) * 1.0

    Q = np.eye(n)
    Q[0, 0] = 5.0  # vx
    Q[1, 1] = 5.0  # vz
    Q[2, 2] = 0.1  # omega
    Q[3, 3] = 1.0  # X
    Q[4, 4] = 50.0  # Z - priority to altitude control!
    Q[5, 5] = 1.0  # theta

    if n == 8:
        Q[6, 6] = 0.1  # Thrust_1
        Q[7, 7] = 0.1  # Thrust_2
        Q = 10 * Q  # Scale for n=8

    # ========================================================================
    # ERROR CALCULATION
    # ========================================================================

    e = np.zeros(n)
    e[0] = x[0] - vx_ref
    e[1] = x[1] - vz_ref
    e[2] = x[2] - 0.0
    e[3] = x[3] - x_ref
    e[4] = x[4] - z_ref
    e[5] = x[5] - theta_ref

    if n == 8:
        e[6] = x[6] - (MASS * g / 2.0)
        e[7] = x[7] - (MASS * g / 2.0)

    # ========================================================================
    # DIAGNOSTICS
    # ========================================================================

    omega_ref = 0.0
    T1 = u[0]
    T2 = u[1]
    T_sum = T1 + T2
    theta = x[5]
    Fx = T_sum * np.sin(theta)
    Fz = T_sum * np.cos(theta)

    print("========== DIAGNOSTIKA ==========")
    print(f"{'Czas':<10} | {t:9.3f} s |")
    print(f"{'Zmienna':<10} | {'Referncja':>12} | {'Aktualna':>12} | {'Błąd':>12}")
    print("-" * 80)

    print(f"{'vx':<10} | {vx_ref:12.5f} | {x[0]:12.5f} | {e[0]:12.5f}")
    print(f"{'vz':<10} | {Vz_ref:12.5f} | {x[1]:12.5f} | {e[1]:12.5f}")
    print(f"{'omega':<10} | {omega_ref:12.5f} | {x[2]:12.5f} | {e[2]:12.5f}")
    print(f"{'X':<10} | {x_ref:12.5f} | {x[3]:12.5f} | {e[3]:12.5f}")
    print(f"{'Z':<10} | {z_ref:12.5f} | {x[4]:12.5f} | {e[4]:12.5f}")
    print(f"{'theta':<10} | {0.0:12.5f} | {math.degrees(x[5]):12.5f} | {math.degrees(e[5]):12.5f}")

    print(f"\n{'T1':<10} = {T1:12.5f} N")
    print(f"{'T2':<10} = {T2:12.5f} N")
    print(f"{'T_sum':<10} = {T_sum:12.5f} N")
    print(f"{'Fx':<10} = {Fx:12.5f} N")
    print(f"{'Fz':<10} = {Fz:12.5f} N")

    if n == 8:
        print(f"\n{'x[6]':<10} = {x[6]:12.5f} N (thrust state)")
        print(f"{'x[7]':<10} = {x[7]:12.5f} N (thrust state)")

    print("==================================\n")

    # ========================================================================
    # LQR CONTROLLER
    # ========================================================================

    A, B = aa_matrices_AB("rhs", x, t, u, n, m)
    K, P = lqr_m(A, B, Q, R)

    # Compute control perturbations
    u_pert = -K @ e

    # Compute desired thrusts
    if n == 6:
        # Direct thrust commands
        T1_desired = T_baseline + u_pert[0]
        T2_desired = T_baseline + u_pert[1]
    else:
        # n=8: separate control
        T1_desired = T_baseline + u_pert[0]
        T2_desired = T_baseline + u_pert[1]

    # Apply physical limits
    T1 = np.clip(T1_desired, T_min_absolute, T_max_absolute)
    T2 = np.clip(T2_desired, T_min_absolute, T_max_absolute)

    u = np.array([T1, T2])

    print(f"DEBUG: u_pert[0]={u_pert[0]:10.5f}, u_pert[1]={u_pert[1]:10.5f}")
    print(f"DEBUG: T1_desired={T1_desired:10.5f}, T2_desired={T2_desired:10.5f}")
    print(f"DEBUG: T1_clipped={T1:10.5f}, T2_clipped={T2:10.5f}")
    print()

    # ========================================================================
    # ENVIRONMENTAL EFFECTS
    # ========================================================================

    az_turbulence = 0.0
    ax_wind = 0.0
    az_wind = 0.0

    if X > X_turb_1 and X < X_turb_2:
        az_turbulence = c_turb * (1.0 - 2.0 * np.random.rand())

    # ========================================================================
    # INTEGRATE STATE
    # ========================================================================

    x = aa_rk45("rhs", x, t, dt, u, az_turbulence, ax_wind, az_wind)

    # ========================================================================
    # VISUALIZATION (every 20 steps)
    # ========================================================================

    if i % 20 == 0:
        V = np.sqrt(x[0] ** 2 + x[1] ** 2)
        e_v = Vel - V
        teta = math.degrees(x[5])
        alt = x[4]

        xs, zs = aa_mdl(x[3], x[4], x[5], 0.50)

        # Remove old plot lines
        for line in plt.gca().lines[2:]:
            line.remove()

        # Plot flight path
        yp_array = np.array(yp)
        plt.plot(yp_array[:, 3], yp_array[:, 4], 'b', label='Flight path', alpha=0.7)
        plt.plot(yp_array[i, 3], yp_array[i, 4], 'bo', markersize=8)

        # Plot drone model
        plt.plot(xs[:5], zs[:5], 'k', linewidth=3)

        # Update title
        txt = (f't={t:7.3f}s V={V:7.3f}m/s vx={x[0]:7.3f} vz={x[1]:7.3f} '
               f'theta={teta:7.2f}° alt={alt:7.3f}m '
               f'| T1={T1:7.1f}N T2={T2:7.1f}N')
        plt.title(txt, fontsize=11)

        plt.legend(fontsize=10)
        plt.draw()
        plt.pause(0.001)

    t = t + dt

plt.ioff()
plt.show()