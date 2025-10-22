
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Arc
from matplotlib.transforms import Affine2D
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

# Choose model: n=6 (simple) or n=10 (with thrust dynamics for 4 motors)
n = 10  # 6 or 10
m = 4  # dimension of control input (4 motors)

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

if n == 6:
    # State: [vx, vz, omega, X, Z, theta]
    x = np.array([0.0, 0.0, 0.0, 0.0, z_initial, 0.0])
elif n == 10:
    # State: [vx, vz, omega, X, Z, theta, Thrust_1, Thrust_2, Thrust_3, Thrust_4]
    x = np.array([0.0, 0.0, 0.0, 0.0, z_initial, 0.0, 
                  initial_thrust, initial_thrust, initial_thrust, initial_thrust])
else:
    raise ValueError("n must be 6 or 10")

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

X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)

# Storage for plotting
tp = []
yp = []
up = []

# Terrain visualization data
terrain_x = []
terrain_z = []
ref_z = []

# ============================================================================
# PLOTTING SETUP - TWO SUBPLOTS
# ============================================================================

plt.ion()
fig = plt.figure(1, figsize=(16, 6))

# Subplot 1: Trajectory (left)
ax1 = plt.subplot(1, 2, 1)
ax1.plot(X_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
ax1.plot(X_ref_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
ax1.axhline(y=0, color='brown', linestyle='--', linewidth=1.5, label='Ground', alpha=0.7)
ax1.legend(fontsize=11)
ax1.axis([0, 50.0, -25.0, 0.5])  # NED: Z negative is up, ground at Z=0
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Z [m] (NED: negative = above ground)')
ax1.set_title('Quadcopter Flight Simulation - NED Coordinates')
ax1.grid(True, alpha=0.3)
ax1.invert_yaxis()  # Invert Y axis so up appears up visually

# Subplot 2: Drone Orientation (right)
ax2 = plt.subplot(1, 2, 2)
ax2.set_xlim(-2, 2)
ax2.set_ylim(-2, 2)
ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3)
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Z [m]')
ax2.set_title('Quadcopter Orientation & Thrust Forces (4 Motors)')

# Draw reference axes
ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
ax2.axvline(x=0, color='k', linestyle='-', linewidth=0.5)
ax2.text(1.8, 0.1, 'X', fontsize=12)
ax2.text(0.1, 1.8, 'Z', fontsize=12)

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

for i in range(10000):
    if t >= 50.0 or x[3] >= 50:
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
    # LQR TUNING (QUADCOPTER)
    # ========================================================================

    R = np.eye(m) * 1.0  # m = 4 control inputs

    Q = np.eye(n)
    Q[0, 0] = 50.0  # vx
    Q[1, 1] = 50.0  # vz
    Q[2, 2] = 0.1  # omega
    Q[3, 3] = 250.0  # X
    Q[4, 4] = 250.0  # Z
    Q[5, 5] = 1.0  # theta

    if n == 10:
        Q[6, 6] = 0.1  # Thrust_1
        Q[7, 7] = 0.1  # Thrust_2
        Q[8, 8] = 0.1  # Thrust_3
        Q[9, 9] = 0.1  # Thrust_4
        Q = 10 * Q

    # ========================================================================
    # ERROR CALCULATION
    # ========================================================================

    e = np.zeros(n)
    e[0] = x[0] - vx_ref
    e[1] = x[1] - vz_ref
    e[2] = x[2] - 0.0
    e[3] = x[3] - x_ref
    e[4] = x[4] - z_ref
    e[5] = x[5] - 0.0

    if n == 10:
        e[6] = x[6] - (MASS * g / 4.0)
        e[7] = x[7] - (MASS * g / 4.0)
        e[8] = x[8] - (MASS * g / 4.0)
        e[9] = x[9] - (MASS * g / 4.0)

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

    if x[3] < 2:
        print("======" * 50)
        print(f't = {t:.3f}s')
        print(f'pozycja: x = {x[3]:.2f}m, z_NED = {x[4]:.2f}m, AGL = {-x[4]:.2f}m')
        print(f'prędkość vx = {x[0]:.2f}m/s, vz = {x[1]:.2f}m/s')
        print(f'kąt pochylenia theta = {math.degrees(x[5]):.2f}°\n')
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
    
    if x[4] >= 0.0:  # If at or below ground level
        x[4] = 0.0  # Keep at ground level
        x[1] = max(0.0, x[1])  # Stop upward velocity, allow downward
        if x[0] > 0:  # Friction slows down horizontal movement
            x[0] *= 0.95
        x[2] = 0.0  # Stop rotation

    # ========================================================================
    # VISUALIZATION
    # ========================================================================

    V = np.sqrt(x[0] ** 2 + x[1] ** 2)
    e_v = Vel - V
    teta = math.degrees(x[5])
    alt = -x[4]  # Altitude above ground (AGL) = -Z in NED

    xs, zs = aa_mdl(x[3], x[4], x[5], 0.50)

    # ====================================================================
    # UPDATE SUBPLOT 1: TRAJECTORY
    # ====================================================================

    ax1.clear()
    ax1.plot(X_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
    ax1.plot(X_ref_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
    ax1.axhline(y=0, color='brown', linestyle='--', linewidth=1.5, label='Ground', alpha=0.7)

    # Plot flight path
    yp_array = np.array(yp)
    ax1.plot(yp_array[:, 3], yp_array[:, 4], 'b', label='Flight path', alpha=0.7)
    ax1.plot(yp_array[i, 3], yp_array[i, 4], 'bo', markersize=8)

    # Plot drone model
    ax1.plot(xs[:5], zs[:5], 'k', linewidth=3)

    txt = (f't={t:7.3f}s V={V:7.3f}m/s theta={teta:7.2f}° AGL={alt:7.3f}m '
           f'| T1={T1:6.1f}N T2={T2:6.1f}N T3={T3:6.1f}N T4={T4:6.1f}N')
    ax1.set_title(txt, fontsize=10)
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axis([max(0, X-10), X+10, -25.0, 0.5])  # NED coordinates
    ax1.invert_yaxis()  # Keep visual orientation consistent
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Z [m] (NED)')

    # ====================================================================
    # UPDATE SUBPLOT 2: QUADCOPTER ORIENTATION (4 MOTORS)
    # ====================================================================

    ax2.clear()
    ax2.set_xlim(-2, 2)
    ax2.set_ylim(-2, 2)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    # Draw reference axes
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    ax2.axvline(x=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    ax2.text(1.8, 0.1, 'X (world)', fontsize=10, color='gray')
    ax2.text(0.1, 1.8, 'Z (world)', fontsize=10, color='gray')

    # Drone dimensions
    drone_width = 1.2
    drone_height = 0.4
    motor_offset_x = 0.5  # Distance from center to motor (front/rear)
    motor_offset_y = 0.3  # Visual separation for left/right motors

    theta = x[5]  # Current angle

    # Drone body (rectangle centered at origin)
    rect_corners_x = np.array([-drone_width/2, drone_width/2, drone_width/2, -drone_width/2, -drone_width/2])
    rect_corners_z = np.array([-drone_height/2, -drone_height/2, drone_height/2, drone_height/2, -drone_height/2])

    # Rotate
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    rect_x_rot = cos_t * rect_corners_x - sin_t * rect_corners_z
    rect_z_rot = sin_t * rect_corners_x + cos_t * rect_corners_z

    # Draw drone body
    ax2.fill(rect_x_rot, rect_z_rot, color='royalblue', alpha=0.7, edgecolor='navy', linewidth=2)

    # Motor positions in local frame (X-configuration)
    # Front motors (1, 2) at +X, separated in Y
    # Rear motors (3, 4) at -X, separated in Y
    motors_local = [
        (+motor_offset_x, +motor_offset_y),  # Motor 1: Front-Left (at +X in body frame)
        (+motor_offset_x, -motor_offset_y),  # Motor 2: Front-Right
        (-motor_offset_x, +motor_offset_y),  # Motor 3: Rear-Left (at -X in body frame)
        (-motor_offset_x, -motor_offset_y)   # Motor 4: Rear-Right
    ]

    # Rotate motor positions to world frame and draw
    motor_positions = []
    for i, (mx_local, mz_local) in enumerate(motors_local):
        mx = cos_t * mx_local - sin_t * mz_local
        mz = sin_t * mx_local + cos_t * mz_local
        motor_positions.append((mx, mz))
        
        # Draw motor
        color = 'darkblue' if i < 2 else 'darkgreen'  # Front motors blue, rear green
        ax2.plot(mx, mz, 'o', color=color, markersize=10)
        ax2.text(mx, mz - 0.2, f'M{i+1}', fontsize=8, ha='center', color=color, weight='bold')

    # Draw thrust arrows
    thrusts = [T1, T2, T3, T4]
    thrust_scale = 0.04
    thrust_colors = ['red', 'red', 'orange', 'orange']  # Front red, rear orange

    for i, ((mx, mz), T) in enumerate(zip(motor_positions, thrusts)):
        # Thrust direction in drone frame: upward in Z_body
        # In world frame after rotation
        thrust_x = -sin_t * thrust_scale * T
        thrust_z = cos_t * thrust_scale * T
        
        ax2.arrow(mx, mz, thrust_x, thrust_z,
                  head_width=0.1, head_length=0.1, fc=thrust_colors[i], ec='darkred', linewidth=2)
        ax2.text(mx + thrust_x, mz + thrust_z + 0.15,
                 f'{T:.0f}N', fontsize=8, ha='center', color=thrust_colors[i], weight='bold')

    # Draw body axes (X_body and Z_body)
    axis_length = 0.9
    # X_body (forward)
    x_body_x = cos_t * axis_length
    x_body_z = sin_t * axis_length
    ax2.arrow(0, 0, x_body_x, x_body_z,
              head_width=0.08, head_length=0.1, fc='green', ec='darkgreen',
              linewidth=1.5, linestyle='--', alpha=0.7)
    ax2.text(x_body_x + 0.1, x_body_z + 0.1, 'X_body', fontsize=9, color='green', weight='bold')

    # Z_body (up in drone frame)
    z_body_x = -sin_t * axis_length
    z_body_z = cos_t * axis_length
    ax2.arrow(0, 0, z_body_x, z_body_z,
              head_width=0.08, head_length=0.1, fc='blue', ec='darkblue',
              linewidth=1.5, linestyle='--', alpha=0.7)
    ax2.text(z_body_x + 0.1, z_body_z + 0.1, 'Z_body', fontsize=9, color='blue', weight='bold')

    # Draw angle arc
    if abs(theta) > 0.01:
        arc = Arc((0, 0), 0.6, 0.6, angle=0, theta1=0, theta2=np.degrees(theta),
                  color='orange', linewidth=2)
        ax2.add_patch(arc)
        ax2.text(0.4, 0.15 if theta > 0 else -0.15, f'θ={teta:.1f}°',
                 fontsize=10, color='orange', weight='bold')

    # Title with thrust info
    thrust_front = T1 + T2
    thrust_rear = T3 + T4
    ax2.set_title(f'Quadcopter | θ={teta:.2f}° | ω={math.degrees(x[2]):.2f}°/s | Front={thrust_front:.0f}N Rear={thrust_rear:.0f}N',
                  fontsize=10, weight='bold')
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Z [m]')

    plt.draw()
    plt.pause(0.001)

    t = t + dt

plt.ioff()
plt.show()
