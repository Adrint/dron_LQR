
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Arc
from matplotlib.transforms import Affine2D
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rk45 import aa_rk45
from matrices import aa_matrices_AB
from model_draw import aa_mdl
from lqr import lqr_m
from trajectory import generate_reference_profile
from rhs import diagnostyka_sil


# Global variables
from constants import (z0, Vel, X_turb_1, X_turb_2, c_turb, MASS, g, T_max, T_min, TAU)

# ============================================================================
# CONFIGURATION - QUADCOPTER X CONFIGURATION
# ============================================================================

# Choose model dimension
print("\n=== MODEL SELECTION ===")
print("1. 2D model (pitch only): n=6 (simple) or n=10 (with thrust dynamics)")
print("2. 3D model (full 6DOF): n=12 (simple) or n=16 (with thrust dynamics)")
model_dim_input = input("Choose model dimension (2D/3D, default=2D): ").strip().lower()

if model_dim_input in ['3d', '3']:
    model_dim = '3D'
    print("\nUsing 3D model (full 6DOF)")
    thrust_dyn_input = input("Include thrust dynamics? (t/n, default=t): ").strip().lower()
    if thrust_dyn_input in ['n', 'no', 'nie']:
        n = 12  # 3D without thrust dynamics
    else:
        n = 16  # 3D with thrust dynamics
else:
    model_dim = '2D'
    print("\nUsing 2D model (pitch only)")
    thrust_dyn_input = input("Include thrust dynamics? (t/n, default=t): ").strip().lower()
    if thrust_dyn_input in ['n', 'no', 'nie']:
        n = 6   # 2D without thrust dynamics
    else:
        n = 10  # 2D with thrust dynamics

m = 4  # dimension of control input (4 motors)
print(f"Model configuration: n={n}, m={m}")

# Integration parameters
dt = input(f"Podaj krok czasowy dt [s] (domyślnie {0.01}): ")

if dt.strip() == "":
    dt = 0.01
else:
    dt = float(dt)

# Starting altitude input - USER FRIENDLY (positive = above ground)
z_start_input = input(f"Podaj wysokość startową nad ziemią [m] (domyślnie {0.0}): ")
if z_start_input.strip() == "":
    z_start_AGL = 0.0  # Altitude Above Ground Level
else:
    z_start_AGL = float(z_start_input)

# Validate input
if z_start_AGL < 0:
    print(f"\nBŁĄD: Wysokość nie może być ujemna! Ustawiam na 0m (ziemia).")
    z_start_AGL = 0.0

# Convert to NED coordinate system (Z positive = down)
z_start = -z_start_AGL  # NED: Z = -altitude

# Engine startup logic
if z_start_AGL == 0.0:
    # On ground - engines always off
    engines_on_start = False
    print("\nDron startuje z ziemi - silniki wyłączone.")
else:
    # Above ground - ask user
    engine_input = input(f"\nDron startuje na wysokości {z_start_AGL:.1f}m. Czy silniki mają być włączone? (t/n, domyślnie n): ")
    if engine_input.strip().lower() in ['t', 'tak', 'y', 'yes']:
        engines_on_start = True
        print("Silniki będą włączone (nominalny ciąg zawieszenia).")
    else:
        engines_on_start = False
        print("Silniki będą wyłączone - dron zacznie spadać.")

t = 0.0

# ============================================================================
# INITIAL STATE - QUADCOPTER (4 motors)
# ============================================================================

# Set initial altitude and thrust based on user inputs
z_initial = z_start

if engines_on_start:
    initial_thrust = MASS * g / 4.0  # Nominal hover thrust per motor (quadcopter)
else:
    initial_thrust = 0.0  # Motors off

# Initial lateral position (for 3D)
y_initial = 0.0  # Start centered

if n == 6:
    # 2D State: [vx, vz, q, X, Z, theta]
    x = np.array([0.0, 0.0, 0.0, 0.0, z_initial, 0.0])
elif n == 10:
    # 2D State with thrust: [vx, vz, q, X, Z, theta, T1, T2, T3, T4]
    x = np.array([0.0, 0.0, 0.0, 0.0, z_initial, 0.0, 
                  initial_thrust, initial_thrust, initial_thrust, initial_thrust])
elif n == 12:
    # 3D State: [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
    x = np.array([0.0, 0.0, 0.0,              # vx, vy, vz
                  0.0, 0.0, 0.0,              # p, q, r
                  0.0, y_initial, z_initial,  # X, Y, Z
                  0.0, 0.0, 0.0])             # phi, theta, psi
elif n == 16:
    # 3D State with thrust: [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi, T1, T2, T3, T4]
    x = np.array([0.0, 0.0, 0.0,              # vx, vy, vz
                  0.0, 0.0, 0.0,              # p, q, r
                  0.0, y_initial, z_initial,  # X, Y, Z
                  0.0, 0.0, 0.0,              # phi, theta, psi
                  initial_thrust, initial_thrust, initial_thrust, initial_thrust])  # T1, T2, T3, T4
else:
    raise ValueError(f"n must be 6, 10, 12, or 16. Got n={n}")

# Control input - start with initial thrust setting (4 motors)
u = np.array([initial_thrust, initial_thrust, initial_thrust, initial_thrust])

# Print startup configuration
print("\n" + "="*70)
print("QUADCOPTER LQR SIMULATION - NED COORDINATE SYSTEM")
print("="*70)
print(f"Configuration: X-config with 4 motors (2D pitch control)")
print(f"Model: n = {n} ({'with thrust dynamics' if n==10 else 'simple model'})")
print(f"Starting altitude AGL: {z_start_AGL:.2f} m (above ground)")
print(f"Starting Z coordinate (NED): {z_initial:.2f} m")
print(f"Initial thrust per motor: {initial_thrust:.2f} N")
print(f"Nominal hover thrust per motor: {MASS * g / 4.0:.2f} N")
print(f"Total nominal thrust: {MASS * g:.2f} N")
print(f"Time step: {dt} s")
print()
print("NED Coordinate System (internal):")
print("  - Z=0: Ground level")
print("  - Z<0: Above ground (e.g., Z=-2.0m = 2m altitude)")
print("  - But YOU enter altitude as positive (user-friendly!)")
print()
if z_start_AGL == 0.0:
    print("🚁 START FROM GROUND - Motors OFF, takeoff sequence will begin")
elif engines_on_start:
    print("🚁 START AIRBORNE - Motors ON at hover thrust, stable flight")
else:
    print("🚁 START AIRBORNE - Motors OFF, drone will fall and catch itself")
print("="*70)
print()

# ============================================================================
# REFERENCE TRAJECTORY
# ============================================================================

if model_dim == '3D':
    # 3D trajectory with lateral motion
    X_ref_all, Y_terr_all, Z_terr_all, Y_ref_all, Z_ref_all, alpha_all, beta_all = generate_reference_profile(Vel, dt, 50)
else:
    # 2D trajectory (backward compatibility)
    from trajectory import generate_reference_profile_2d
    X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile_2d(Vel, dt, 50)
    # Create dummy Y arrays for 2D mode
    Y_terr_all = np.zeros_like(X_ref_all)
    Y_ref_all = np.zeros_like(X_ref_all)
    beta_all = np.zeros_like(X_ref_all)

# Storage for plotting
tp = []
yp = []
up = []

# Terrain visualization data
terrain_x = []
terrain_z = []
ref_z = []

# ============================================================================
# PLOTTING SETUP - FOUR SUBPLOTS (2x2 GRID)
# ============================================================================

plt.ion()
fig = plt.figure(1, figsize=(18, 12))

# Subplot 1: Z(X) Trajectory (top left)
ax1 = plt.subplot(2, 2, 1)
ax1.plot(X_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
ax1.plot(X_ref_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
ax1.axhline(y=0, color='brown', linestyle='--', linewidth=1.5, label='Ground', alpha=0.7)
ax1.legend(fontsize=11)
ax1.axis([0, 50.0, -25.0, 0.5])
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Z [m] (NED: negative = above ground)')
ax1.set_title('Trajektoria Z(X)')
ax1.grid(True, alpha=0.3)
ax1.invert_yaxis()

# Subplot 2: Y(X) Trajectory (top right)
ax2 = plt.subplot(2, 2, 2)
ax2.plot(X_ref_all, Y_ref_all, 'r', label='Reference Y', linewidth=2)
ax2.plot(X_ref_all, Y_terr_all, 'g', label='Terrain Y', linewidth=2)
ax2.legend(fontsize=11)
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Y [m]')
ax2.set_title('Trajektoria Y(X)')
ax2.grid(True, alpha=0.3)

# Subplot 3: 3D Trajectory (bottom left)
ax3 = plt.subplot(2, 2, 3, projection='3d')
ax3.plot(X_ref_all, Y_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
ax3.plot(X_ref_all, Y_terr_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
ax3.set_xlabel('X [m]')
ax3.set_ylabel('Y [m]')
ax3.set_zlabel('Z [m]')
ax3.set_title('Trajektoria 3D (X, Y, Z)')
ax3.legend(fontsize=10)
ax3.grid(True, alpha=0.3)
ax3.invert_zaxis()

# Subplot 4: Drone Orientation (bottom right) - 3D VIEW
ax4 = plt.subplot(2, 2, 4, projection='3d')
ax4.set_xlim(-3, 3)
ax4.set_ylim(-3, 3)
ax4.set_zlim(-3, 3)
ax4.set_xlabel('X [m]')
ax4.set_ylabel('Y [m]')
ax4.set_zlabel('Z [m]')
ax4.set_title('Quadcopter 3D Orientation & Thrust Forces (SCALED)')
ax4.view_init(elev=20, azim=45)

plt.tight_layout()
plt.draw()
plt.pause(1)

x_ref = 0.0
# ============================================================================
# PHYSICAL THRUST LIMITS
# ============================================================================

T_baseline = MASS * g / 4.0  # Thrust baseline for equilibrium (quadcopter)
T_max_absolute = T_max  # Maximum physical thrust
T_min_absolute = T_min  # Minimum physical thrust

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================

# Initialize reference tracking
x_ref = 0.0
y_ref = 0.0 if model_dim == '3D' else 0.0

for i in range(10000):
    # Exit conditions based on model dimension
    if model_dim == '2D':
        if t >= 50.0 or x[3] >= 50:
            break
        X = x[3]  # Current forward position
    else:  # 3D
        if t >= 50.0 or x[6] >= 50:
            break
        X = x[6]  # Current forward position

    # === 1. Find reference point ===
    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]
    
    if model_dim == '3D':
        y_terr = Y_terr_all[idx_ref]
        y_ref = Y_ref_all[idx_ref]
        beta = beta_all[idx_ref]
    else:
        y_terr = 0.0
        y_ref = 0.0
        beta = 0.0

    # === 2. Reference velocity components ===
    Vx_ref = Vel * np.cos(alfa)
    Vz_ref = Vel * np.sin(alfa)
    
    if model_dim == '3D':
        Vy_ref = Vel * np.sin(beta) * np.cos(alfa)  # Lateral velocity component
    else:
        Vy_ref = 0.0

    # === 3. Update reference position ===
    x_ref += Vel * np.cos(alfa) * dt
    
    if model_dim == '2D':
        # 2D: Transform to body frame velocities
        theta = x[5]
        vx_ref = np.cos(theta) * Vx_ref + np.sin(theta) * Vz_ref
        vz_ref = -np.sin(theta) * Vx_ref + np.cos(theta) * Vz_ref
        vy_ref = 0.0
    else:  # 3D
        # 3D: Transform to body frame velocities using full rotation
        phi = x[9]
        theta = x[10]
        psi = x[11]
        
        # Simplified: assume reference velocities are already in body frame
        # For more accurate tracking, would need full rotation matrix
        vx_ref = Vx_ref
        vy_ref = Vy_ref
        vz_ref = Vz_ref

    # === 4. Collect data ===
    terrain_x.append(X)
    terrain_z.append(z_terr)
    ref_z.append(z_ref)

    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    # ========================================================================
    # LQR TUNING (QUADCOPTER)
    # ========================================================================

    R = np.eye(m) * 1.0  # m = 4 control inputs

    Q = np.eye(n)
    
    if model_dim == '2D':
        # 2D model weights
        Q[0, 0] = 50.0  # vx
        Q[1, 1] = 50.0  # vz
        Q[2, 2] = 0.1  # q (pitch rate)
        Q[3, 3] = 250.0  # X
        Q[4, 4] = 250.0  # Z
        Q[5, 5] = 1.0  # theta
        
        if n == 10:
            Q[6, 6] = 0.1  # Thrust_1
            Q[7, 7] = 0.1  # Thrust_2
            Q[8, 8] = 0.1  # Thrust_3
            Q[9, 9] = 0.1  # Thrust_4
            Q = 10 * Q
    else:  # 3D
        # 3D model weights
        Q[0, 0] = 50.0  # vx
        Q[1, 1] = 50.0  # vy
        Q[2, 2] = 50.0  # vz
        Q[3, 3] = 0.1   # p (roll rate)
        Q[4, 4] = 0.1   # q (pitch rate)
        Q[5, 5] = 0.1   # r (yaw rate)
        Q[6, 6] = 250.0 # X
        Q[7, 7] = 250.0 # Y
        Q[8, 8] = 250.0 # Z
        Q[9, 9] = 1.0   # phi (roll)
        Q[10, 10] = 1.0 # theta (pitch)
        Q[11, 11] = 1.0 # psi (yaw)
        
        if n == 16:
            Q[12, 12] = 0.1  # Thrust_1
            Q[13, 13] = 0.1  # Thrust_2
            Q[14, 14] = 0.1  # Thrust_3
            Q[15, 15] = 0.1  # Thrust_4
            Q = 10 * Q

    # ========================================================================
    # ERROR CALCULATION
    # ========================================================================

    e = np.zeros(n)
    
    # Reference angles (for simplicity, keep level flight)
    phi_ref = 0.0
    theta_ref = 0.0
    psi_ref = 0.0
    
    if model_dim == '2D':
        # 2D error vector
        e[0] = x[0] - vx_ref
        e[1] = x[1] - vz_ref
        e[2] = x[2] - 0.0  # q_ref = 0
        e[3] = x[3] - x_ref
        e[4] = x[4] - z_ref
        e[5] = x[5] - theta_ref
        
        if n == 10:
            e[6] = x[6] - (MASS * g / 4.0)
            e[7] = x[7] - (MASS * g / 4.0)
            e[8] = x[8] - (MASS * g / 4.0)
            e[9] = x[9] - (MASS * g / 4.0)
    else:  # 3D
        # 3D error vector
        e[0] = x[0] - vx_ref
        e[1] = x[1] - vy_ref
        e[2] = x[2] - vz_ref
        e[3] = x[3] - 0.0  # p_ref = 0
        e[4] = x[4] - 0.0  # q_ref = 0
        e[5] = x[5] - 0.0  # r_ref = 0
        e[6] = x[6] - x_ref
        e[7] = x[7] - y_ref
        e[8] = x[8] - z_ref
        e[9] = x[9] - phi_ref
        e[10] = x[10] - theta_ref
        e[11] = x[11] - psi_ref
        
        if n == 16:
            e[12] = x[12] - (MASS * g / 4.0)
            e[13] = x[13] - (MASS * g / 4.0)
            e[14] = x[14] - (MASS * g / 4.0)
            e[15] = x[15] - (MASS * g / 4.0)

    # ========================================================================
    # LQR CONTROLLER
    # ========================================================================

    A, B = aa_matrices_AB("rhs", x, t, u, n, m)
    K, P = lqr_m(A, B, Q, R)

    u_pert = -K @ e

    # Add nominal thrust (equilibrium point) to LQR perturbation
    # LQR gives control perturbation from equilibrium, but motors need absolute thrust commands
    T_nominal = MASS * g / 4.0  # Quadcopter: each motor carries 1/4 weight
    
    T1_desired = T_nominal + u_pert[0]
    T2_desired = T_nominal + u_pert[1]
    T3_desired = T_nominal + u_pert[2]
    T4_desired = T_nominal + u_pert[3]

    T1 = np.clip(T1_desired, T_min_absolute, T_max_absolute)
    T2 = np.clip(T2_desired, T_min_absolute, T_max_absolute)
    T3 = np.clip(T3_desired, T_min_absolute, T_max_absolute)
    T4 = np.clip(T4_desired, T_min_absolute, T_max_absolute)

    u = np.array([T1, T2, T3, T4])

    if X < 2:
        print("======" * 50)
        print(f't = {t:.3f}s')
        if model_dim == '2D':
            print(f'pozycja: x = {x[3]:.2f}m, z_NED = {x[4]:.2f}m, AGL = {-x[4]:.2f}m')
            print(f'prędkość vx = {x[0]:.2f}m/s, vz = {x[1]:.2f}m/s')
            print(f'kąt pochylenia theta = {math.degrees(x[5]):.2f}°')
        else:  # 3D
            print(f'pozycja: x = {x[6]:.2f}m, y = {x[7]:.2f}m, z_NED = {x[8]:.2f}m, AGL = {-x[8]:.2f}m')
            print(f'prędkość vx = {x[0]:.2f}m/s, vy = {x[1]:.2f}m/s, vz = {x[2]:.2f}m/s')
            print(f'kąty: phi = {math.degrees(x[9]):.2f}°, theta = {math.degrees(x[10]):.2f}°, psi = {math.degrees(x[11]):.2f}°')
            print(f'prędkości kątowe: p = {math.degrees(x[3]):.2f}°/s, q = {math.degrees(x[4]):.2f}°/s, r = {math.degrees(x[5]):.2f}°/s')
        print()
        diagnostyka_sil(x, u)
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
    # GROUND COLLISION PREVENTION (NED: Z >= 0 means at/below ground)
    # ========================================================================
    
    if model_dim == '2D':
        Z_idx = 4
        vz_idx = 1
        vx_idx = 0
        omega_idx = 2
    else:  # 3D
        Z_idx = 8
        vz_idx = 2
        vx_idx = 0
        omega_idx = 4  # q for 3D
    
    if x[Z_idx] >= 0.0:  # If at or below ground level
        x[Z_idx] = 0.0  # Keep at ground level
        x[vz_idx] = max(0.0, x[vz_idx])  # Stop upward velocity, allow downward
        if x[vx_idx] > 0:  # Friction slows down horizontal movement
            x[vx_idx] *= 0.95
        x[omega_idx] = 0.0  # Stop rotation (pitch for both 2D and 3D)

    # ========================================================================
    # VISUALIZATION
    # ========================================================================

    if model_dim == '2D':
        V = np.sqrt(x[0] ** 2 + x[1] ** 2)
        teta = math.degrees(x[5])
        alt = -x[4]  # Altitude above ground (AGL) = -Z in NED
        X_pos = x[3]
        Z_pos = x[4]
        theta_vis = x[5]
    else:  # 3D
        V = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
        phi_deg = math.degrees(x[9])
        theta_deg = math.degrees(x[10])
        psi_deg = math.degrees(x[11])
        alt = -x[8]  # Altitude above ground (AGL) = -Z in NED
        X_pos = x[6]
        Y_pos = x[7]
        Z_pos = x[8]
        theta_vis = x[10]  # Use theta for visualization

    e_v = Vel - V
    xs, zs = aa_mdl(X_pos, Z_pos, theta_vis, 0.50)

    # ====================================================================
    # UPDATE ALL SUBPLOTS
    # ====================================================================

    # ====================================================================
    # UPDATE SUBPLOT 1: Z(X) TRAJECTORY
    # ====================================================================

    ax1.clear()
    ax1.plot(X_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
    ax1.plot(X_ref_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
    ax1.axhline(y=0, color='brown', linestyle='--', linewidth=1.5, label='Ground', alpha=0.7)

    # Plot flight path
    yp_array = np.array(yp)
    if model_dim == '2D':
        ax1.plot(yp_array[:, 3], yp_array[:, 4], 'b', label='Flight path', alpha=0.7)
        ax1.plot(yp_array[i, 3], yp_array[i, 4], 'bo', markersize=8)
    else:  # 3D
        ax1.plot(yp_array[:, 6], yp_array[:, 8], 'b', label='Flight path', alpha=0.7)
        ax1.plot(yp_array[i, 6], yp_array[i, 8], 'bo', markersize=8)

    # Plot drone model
    ax1.plot(xs[:5], zs[:5], 'k', linewidth=3)

    if model_dim == '2D':
        txt = (f't={t:7.3f}s | V={V:7.3f}m/s | θ={teta:7.2f}° | x={X_pos:7.3f}m | z={alt:7.3f}m')
    else:  # 3D
        txt = (f't={t:7.3f}s | V={V:7.3f}m/s | φ={phi_deg:6.2f}° θ={theta_deg:6.2f}° ψ={psi_deg:6.2f}° | '
               f'x={X_pos:7.3f}m y={Y_pos:7.3f}m z={alt:7.3f}m')
    ax1.set_title(txt, fontsize=9)
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.axis([max(0, X-10), X+10, -25.0, 0.5])
    ax1.invert_yaxis()
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Z [m]')

    # ====================================================================
    # UPDATE SUBPLOT 2: Y(X) TRAJECTORY
    # ====================================================================

    ax2.clear()
    ax2.plot(X_ref_all, Y_ref_all, 'r', label='Reference Y', linewidth=2)
    ax2.plot(X_ref_all, Y_terr_all, 'g', label='Terrain Y', linewidth=2)
    
    if model_dim == '3D':
        ax2.plot(yp_array[:, 6], yp_array[:, 7], 'b', label='Flight path Y', alpha=0.7)
        ax2.plot(yp_array[i, 6], yp_array[i, 7], 'bo', markersize=8)
    else:
        ax2.plot(yp_array[:, 3], np.zeros_like(yp_array[:, 3]), 'b', label='Flight path Y', alpha=0.7)
        ax2.plot(yp_array[i, 3], 0, 'bo', markersize=8)
    
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('Trajektoria Y(X)', fontsize=10)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.axis([max(0, X-10), X+10, -5, 5])

    # ====================================================================
    # UPDATE SUBPLOT 3: 3D TRAJECTORY
    # ====================================================================

    ax3.clear()
    ax3.plot(X_ref_all, Y_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
    ax3.plot(X_ref_all, Y_terr_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
    
    if model_dim == '3D':
        ax3.plot(yp_array[:, 6], yp_array[:, 7], yp_array[:, 8], 'b', label='Flight path', alpha=0.7, linewidth=2)
        ax3.scatter([yp_array[i, 6]], [yp_array[i, 7]], [yp_array[i, 8]], color='blue', s=100)
    else:
        ax3.plot(yp_array[:, 3], np.zeros_like(yp_array[:, 3]), yp_array[:, 4], 'b', label='Flight path', alpha=0.7, linewidth=2)
        ax3.scatter([yp_array[i, 3]], [0], [yp_array[i, 4]], color='blue', s=100)
    
    ax3.set_xlabel('X [m]')
    ax3.set_ylabel('Y [m]')
    ax3.set_zlabel('Z [m]')
    ax3.set_title('Trajektoria 3D (X, Y, Z)', fontsize=10)
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.invert_zaxis()

    # ====================================================================
    # UPDATE SUBPLOT 4: QUADCOPTER 3D ORIENTATION - SCALED UP!
    # ====================================================================

    ax4.clear()
    ax4.set_xlim(-3, 3)
    ax4.set_ylim(-3, 3)
    ax4.set_zlim(-3, 3)
    ax4.set_xlabel('Y [m]', fontsize=11, weight='bold')
    ax4.set_ylabel('X [m]', fontsize=11, weight='bold')
    ax4.set_zlabel('Z [m]', fontsize=11, weight='bold')
    ax4.view_init(elev=20, azim=45)

    # Draw world coordinate axes - BIGGER
    axis_len = 5
    ax4.plot([0, 0], [0, axis_len], [0, 0], 'r-', linewidth=3, alpha=0.7)
    ax4.plot([0, axis_len], [0, 0], [0, 0], 'g-', linewidth=3, alpha=0.7)
    ax4.plot([0, 0], [0, 0], [0, axis_len], 'b-', linewidth=3, alpha=0.7)
    ax4.text(0, axis_len+0.2, 0, 'X', fontsize=14, color='red', weight='bold')
    ax4.text(axis_len+0.2, 0, 0, 'Y', fontsize=14, color='green', weight='bold')
    ax4.text(0, 0, axis_len+0.2, 'Z', fontsize=14, color='blue', weight='bold')

    # Drone dimensions - SCALED UP 2X!
    arm_length = 1.2  # Distance from center to motor (was 0.6)
    theta = x[5]  # Current pitch angle

    # Rotation matrix for pitch (around Y axis)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)

    # Motor positions in body frame (X configuration)
    angle_offset = np.pi / 4  # 45 degrees
    motors_body = [
        (arm_length * np.cos(angle_offset), arm_length * np.sin(angle_offset), 0),
        (arm_length * np.cos(angle_offset), -arm_length * np.sin(angle_offset), 0),
        (-arm_length * np.cos(angle_offset), arm_length * np.sin(angle_offset), 0),
        (-arm_length * np.cos(angle_offset), -arm_length * np.sin(angle_offset), 0)
    ]

    # Rotate motors to world frame
    motors_world = []
    for mx_b, my_b, mz_b in motors_body:
        mx_w = cos_t * mx_b - sin_t * mz_b
        my_w = my_b
        mz_w = sin_t * mx_b + cos_t * mz_b
        motors_world.append((mx_w, my_w, mz_w))

    # Draw drone arms (X shape) - THICKER
    ax4.plot([motors_world[0][1], motors_world[3][1]], 
             [motors_world[0][0], motors_world[3][0]], 
             [motors_world[0][2], motors_world[3][2]], 
             'k-', linewidth=2, alpha=0.9)
    ax4.plot([motors_world[1][1], motors_world[2][1]], 
             [motors_world[1][0], motors_world[2][0]], 
             [motors_world[1][2], motors_world[2][2]], 
             'k-', linewidth=2, alpha=0.9)

    # Draw center body (cube) - BIGGER
    body_size = 0.3  # was 0.15
    r = body_size / 2
    vertices_body = [
        [-r, -r, -r], [r, -r, -r], [r, r, -r], [-r, r, -r],
        [-r, -r, r], [r, -r, r], [r, r, r], [-r, r, r]
    ]
    vertices_world = []
    for vx_b, vy_b, vz_b in vertices_body:
        vx_w = cos_t * vx_b - sin_t * vz_b
        vy_w = vy_b
        vz_w = sin_t * vx_b + cos_t * vz_b
        vertices_world.append([vy_w, vx_w, vz_w])
    
    faces = [
        [vertices_world[0], vertices_world[1], vertices_world[2], vertices_world[3]],
        [vertices_world[4], vertices_world[5], vertices_world[6], vertices_world[7]],
        [vertices_world[0], vertices_world[1], vertices_world[5], vertices_world[4]],
        [vertices_world[2], vertices_world[3], vertices_world[7], vertices_world[6]],
        [vertices_world[0], vertices_world[3], vertices_world[7], vertices_world[4]],
        [vertices_world[1], vertices_world[2], vertices_world[6], vertices_world[5]]
    ]
    
    cube = Poly3DCollection(faces, alpha=0.75, facecolor='royalblue', edgecolor='navy', linewidth=2)
    ax4.add_collection3d(cube)

    # Draw motors and thrust vectors - BIGGER AND MORE VISIBLE
    thrusts = [T1, T2, T3, T4]
    thrust_scale = 0.003  # DOUBLED from 0.003
    motor_colors = ['red', 'blue', 'green', 'orange']


    for i, ((mx, my, mz), T) in enumerate(zip(motors_world, thrusts)):
        # Draw motor - BIGGER
        ax4.scatter([my], [mx], [mz], color=motor_colors[i], s=150,
                   edgecolors='black', linewidths=1, depthshade=True, zorder=10)
        
        # Thrust vector
        thrust_x = -sin_t * thrust_scale * T
        thrust_y = 0
        thrust_z = cos_t * thrust_scale * T

        # Draw thrust arrow - THICKER AND MORE VISIBLE
        ax4.quiver(my, mx, mz, thrust_y, thrust_x, thrust_z,
                  color=motor_colors[i], arrow_length_ratio=0.2, linewidth=5, alpha=0.95)


    # Draw body axes - BIGGER
    axis_body_len = 1.5  # was 0.8
    
    # X_body (forward) - RED
    xb_x = cos_t * axis_body_len
    xb_y = 0
    xb_z = sin_t * axis_body_len
    ax4.quiver(0, 0, 0, xb_y, xb_x, xb_z, color='darkred',
              arrow_length_ratio=0.15, linewidth=2, linestyle='--', alpha=0.9)
    ax4.text(xb_y, xb_x+0.1, xb_z, 'X\'', fontsize=11, color='darkred', weight='bold')
    
    # Y_body (right) - GREEN
    yb_x = 0
    yb_y = axis_body_len
    yb_z = 0
    ax4.quiver(0, 0, 0, yb_y, yb_x, yb_z, color='darkgreen', 
              arrow_length_ratio=0.15, linewidth=2, linestyle='--', alpha=0.9)
    ax4.text(yb_y+0.1, yb_x, yb_z, 'Y\'', fontsize=11, color='darkgreen', weight='bold')
    
    # Z_body (down) - BLUE
    zb_x = -sin_t * axis_body_len
    zb_y = 0
    zb_z = cos_t * axis_body_len
    ax4.quiver(0, 0, 0, zb_y, zb_x, zb_z, color='darkblue', 
              arrow_length_ratio=0.15, linewidth=2, linestyle='--', alpha=0.9)
    ax4.text(zb_y, zb_x, zb_z+0.1, 'Z\'', fontsize=11, color='darkblue', weight='bold')

    # Single info box on the left with all information
    if model_dim == '2D':
        info_text = f'Pitch θ = {teta:.2f}°\nω = {math.degrees(x[2]):.2f}°/s'
    else:  # 3D
        info_text = f'Roll φ = {phi_deg:.2f}°\nPitch θ = {theta_deg:.2f}°\nYaw ψ = {psi_deg:.2f}°\n'
        info_text += f'p = {math.degrees(x[3]):.2f}°/s\nq = {math.degrees(x[4]):.2f}°/s\nr = {math.degrees(x[5]):.2f}°/s'
    
    ax4.text2D(0.02, 1.02, info_text, transform=ax4.transAxes,
              fontsize=10, verticalalignment='top', bbox=dict(boxstyle='round', 
              facecolor='wheat', alpha=0.9, edgecolor='black', linewidth=2), family='monospace', weight='bold')
    
    # Add motor color legend
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=motor_colors[0], edgecolor='black', label=f'M1: {T1:.1f}N'),
                      Patch(facecolor=motor_colors[1], edgecolor='black', label=f'M2: {T2:.1f}N'),
                      Patch(facecolor=motor_colors[2], edgecolor='black', label=f'M3: {T3:.1f}N'),
                      Patch(facecolor=motor_colors[3], edgecolor='black', label=f'M4: {T4:.1f}N')]
    ax4.legend(handles=legend_elements, loc='upper right', fontsize=9, framealpha=0.9)

    # Title
    # ax4.set_title(f'Quadcopter 3D | Total Thrust={T1+T2+T3+T4:.0f}N',
    #               fontsize=12, weight='bold')

    plt.draw()
    plt.pause(0.001)

    t = t + dt

plt.ioff()
plt.show()
