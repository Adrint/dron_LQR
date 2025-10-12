import numpy as np
import math
import matplotlib.pyplot as plt
from rk45 import aa_rk45
from matrices import aa_matrices_AB
from model_draw import aa_mdl
from lqr import lqr_m
from trajectory import generate_reference_profile

from constants import (z0, Vel, X_turb_1, X_turb_2, c_turb, MASS, g, T_max, T_min, TAU)

# ============================================================================
# CONFIGURATION
# ============================================================================

# Choose: n = 12 (simple) or n = 16 (with motor dynamics)
# NOTE: Full state is not controllable, so we use reduced feedback
n_full = 12  # Full state dimension
n_ctrl = 6  # Controllable state dimension: [vx, vy, vz, p, q, r]
m = 4  # 4 motors

dt = 0.01
t = 0.0

# ============================================================================
# INITIAL STATE
# ============================================================================

T_hover = MASS * g / 4.0

x = np.array([
    0.0,  # vx
    0.0,  # vy
    0.0,  # vz
    0.0,  # p (roll rate)
    0.0,  # q (pitch rate)
    0.0,  # r (yaw rate)
    0.0,  # X
    0.0,  # Y
    z0,  # Z (altitude)
    0.0,  # phi (roll angle)
    0.0,  # theta (pitch angle)
    0.0,  # psi (yaw angle)
])

# Control input - hover equilibrium
u = np.array([T_hover, T_hover, T_hover, T_hover])

# ============================================================================
# REFERENCE TRAJECTORY
# ============================================================================

X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)

# Storage
tp = []
yp = []
up = []

terrain_x = []
terrain_z = []
ref_z = []

# ============================================================================
# PLOTTING
# ============================================================================

plt.ion()
fig = plt.figure(1, figsize=(12, 7))

# 3D plot
ax = fig.add_subplot(111, projection='3d')
ax.plot(X_ref_all, np.zeros_like(X_ref_all), Z_ref_all, 'r-', label='Reference', linewidth=2)
ax.plot(X_ref_all, np.zeros_like(X_ref_all), Z_terr_all, 'g-', label='Terrain', linewidth=2)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title(f'Quadrocopter Flight Simulation - Reduced Feedback (n={n_ctrl})')
ax.legend(fontsize=10)
ax.set_xlim([0, 50])
ax.set_ylim([-5, 5])
ax.set_zlim([0, 20])

plt.draw()
plt.pause(1)

x_ref = 0.0

# ============================================================================
# PHYSICAL LIMITS
# ============================================================================

T_baseline = MASS * g / 4.0
T_max_absolute = T_max
T_min_absolute = T_min

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================

print("Starting simulation with REDUCED FEEDBACK (controllable states only)")
print(f"Full state dimension: {n_full}")
print(f"Control dimension: {n_ctrl} (velocity and angular rates)")
print("Position tracking is achieved through velocity integration\n")

for i in range(100000):
    if t >= 50.0:
        break

    X = x[6]

    # === 1. Find reference point ===
    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]

    # === 2. Reference velocity ===
    Vx_ref = Vel * np.cos(alfa)
    Vz_ref = Vel * np.sin(alfa)

    # === 3. Update reference position ===
    x_ref += Vel * np.cos(alfa) * dt

    # === 4. Collect data ===
    terrain_x.append(X)
    terrain_z.append(z_terr)
    ref_z.append(z_ref)

    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    # ========================================================================
    # EXTRACT CONTROLLABLE STATES
    # ========================================================================
    # x_ctrl = [vx, vy, vz, p, q, r]
    x_ctrl = x[:6]

    # Reference for controllable states
    x_ref_ctrl = np.array([
        Vx_ref,  # vx_ref
        0.0,  # vy_ref
        Vz_ref,  # vz_ref
        0.0,  # p_ref (roll rate)
        0.0,  # q_ref (pitch rate)
        0.0  # r_ref (yaw rate)
    ])

    # Error in controllable states
    e_ctrl = x_ctrl - x_ref_ctrl

    # ========================================================================
    # LQR ON REDUCED STATE SPACE
    # ========================================================================

    # Get full A and B matrices
    A_full, B_full = aa_matrices_AB("rhs", x, t, u, n_full, m)

    # Extract reduced system (first 6 rows/columns)
    A_red = A_full[:6, :6]
    B_red = B_full[:6, :]

    # Cost matrices for reduced system
    Q_red = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])  # [vx, vy, vz, p, q, r]
    R_red = np.eye(m) * 0.5

    try:
        K_red, P_red = lqr_m(A_red, B_red, Q_red, R_red)
    except Exception as exc:
        print(f"LQR Error at t={t:.3f}: {exc}")
        break

    # Compute control based on reduced state feedback
    u_pert = -K_red @ e_ctrl

    # Desired thrusts
    T_desired = np.array([
        T_baseline + u_pert[0],
        T_baseline + u_pert[1],
        T_baseline + u_pert[2],
        T_baseline + u_pert[3]
    ])

    # Apply limits
    u = np.clip(T_desired, T_min_absolute, T_max_absolute)

    # ========================================================================
    # DIAGNOSTICS (every 100 iterations)
    # ========================================================================

    if i % 100 == 0:
        vx, vy, vz = x[0], x[1], x[2]
        p, q, r = x[3], x[4], x[5]
        X_pos, Y_pos, Z_pos = x[6], x[7], x[8]
        phi = x[9]
        theta = x[10]
        psi = x[11]

        T_sum = u[0] + u[1] + u[2] + u[3]

        print("========== QUADROCOPTER DIAGNOSTIKA (REDUCED FEEDBACK) ==========")
        print(f"{'Czas':<10} | {t:9.3f} s |")
        print("-" * 80)
        print(f"{'POZYCJA':<10} | {'X':>12} | {'Y':>12} | {'Z':>12}")
        print(f"{'Referent':<10} | {x_ref:12.5f} | {0.0:12.5f} | {z_ref:12.5f}")
        print(f"{'Aktualna':<10} | {X_pos:12.5f} | {Y_pos:12.5f} | {Z_pos:12.5f}")
        print(f"{'Błąd Z':<10} | {Z_pos - z_ref:12.5f}")

        print(f"\n{'PRĘDKOŚĆ':<10} | {'VX':>12} | {'VY':>12} | {'VZ':>12}")
        print(f"{'Ref':<10} | {Vx_ref:12.5f} | {0.0:12.5f} | {Vz_ref:12.5f}")
        print(f"{'Aktualna':<10} | {vx:12.5f} | {vy:12.5f} | {vz:12.5f}")
        print(f"{'Błąd':<10} | {e_ctrl[0]:12.5f} | {e_ctrl[1]:12.5f} | {e_ctrl[2]:12.5f}")

        print(f"\n{'ORIENTACJA':<10} | {'Roll':>12} | {'Pitch':>12} | {'Yaw':>12}")
        print(f"{'Kąty [°]':<10} | {math.degrees(phi):12.5f} | {math.degrees(theta):12.5f} | {math.degrees(psi):12.5f}")

        print(f"\n{'PRĘDKOŚCI KĄT.':<10} | {'p':>12} | {'q':>12} | {'r':>12}")
        print(f"{'Wartości':<10} | {p:12.5f} | {q:12.5f} | {r:12.5f}")
        print(f"{'Błędy':<10} | {e_ctrl[3]:12.5f} | {e_ctrl[4]:12.5f} | {e_ctrl[5]:12.5f}")

        print(f"\n{'STEROWANIE [N]':<10} | {'T1':>12} | {'T2':>12} | {'T3':>12} | {'T4':>12} | {'Sum':>12}")
        print(f"{'Wartości':<10} | {u[0]:12.2f} | {u[1]:12.2f} | {u[2]:12.2f} | {u[3]:12.2f} | {T_sum:12.2f}")

        print("=" * 80 + "\n")

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
    # VISUALIZATION (every 50 steps)
    # ========================================================================

    if i % 50 == 0:
        yp_array = np.array(yp)

        # Clear and redraw
        ax.clear()

        # Reference path
        ax.plot(X_ref_all, np.zeros_like(X_ref_all), Z_ref_all, 'r-', label='Reference', linewidth=2, alpha=0.5)
        ax.plot(X_ref_all, np.zeros_like(X_ref_all), Z_terr_all, 'g-', label='Terrain', linewidth=2, alpha=0.5)

        # Actual flight path
        ax.plot(yp_array[:, 6], yp_array[:, 7], yp_array[:, 8], 'b-', label='Flight path', linewidth=1)

        # Current position
        ax.scatter([x[6]], [x[7]], [x[8]], c='blue', s=100, marker='o', label='Drone')

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_xlim([0, 50])
        ax.set_ylim([-5, 5])
        ax.set_zlim([0, 20])

        phi_deg = math.degrees(x[9])
        theta_deg = math.degrees(x[10])
        psi_deg = math.degrees(x[11])

        V = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)

        title_str = (f't={t:7.3f}s V={V:7.3f}m/s '
                     f'X={x[6]:7.2f} Y={x[7]:7.2f} Z={x[8]:7.2f} | '
                     f'φ={phi_deg:7.2f}° θ={theta_deg:7.2f}° ψ={psi_deg:7.2f}°')
        ax.set_title(title_str, fontsize=10)
        ax.legend(fontsize=9)

        plt.draw()
        plt.pause(0.001)

    t = t + dt

plt.ioff()
plt.show()