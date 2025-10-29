import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Patch, FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rk45 import aa_rk45
from matrices import aa_matrices_AB
from model_draw import aa_mdl
from lqr import lqr_m
from trajectory import generate_reference_profile
from rhs import diagnostyka_sil

# Global variables
from constants import (Vel, X_turb_1, X_turb_2, c_turb, MASS, g, T_max, T_min, NOMINAL_thrust, MOTOR_ARM_LENGTH_X, MOTOR_ARM_LENGTH_Y)

# Physical limits
from limits import (apply_all_limits, check_limits_exceeded, print_limits,
                    MAX_HORIZONTAL_VELOCITY, MAX_ASCENT_RATE, MAX_DESCENT_RATE,
                    MAX_PITCH_ANGLE, MAX_ROLL_ANGLE, MAX_PITCH_RATE, MAX_ROLL_RATE, MAX_YAW_RATE)


# ============================================================================
# HELPER FUNCTION: 3D ROTATION MATRIX (ZYX Convention)
# ============================================================================

def rotation_matrix_zyx(phi, theta, psi):
    """
    Compute full 3D rotation matrix using ZYX Euler angles (yaw-pitch-roll).
    Uses AEROSPACE CONVENTION for NED coordinate system.

    phi: roll angle (rotation around X)
    theta: pitch angle (rotation around Y)
        - theta > 0: nose up
        - theta < 0: nose down
    psi: yaw angle (rotation around Z)

    Returns: 3x3 rotation matrix from body frame to NED inertial frame
    """
    # Roll (X-axis rotation) - standard right-hand rule
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    Rx = np.array([
        [1, 0, 0],
        [0, c_phi, -s_phi],
        [0, s_phi, c_phi]
    ])

    # Pitch (Y-axis rotation) - AEROSPACE CONVENTION (reversed from math convention)
    # In aerospace/NED: positive theta rotates nose up
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    Ry = np.array([
        [c_theta, 0, -s_theta],  # Note: -sin for aerospace convention
        [0, 1, 0],
        [s_theta, 0, c_theta]  # Note: +sin for aerospace convention
    ])

    # Yaw (Z-axis rotation) - standard right-hand rule
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    Rz = np.array([
        [c_psi, -s_psi, 0],
        [s_psi, c_psi, 0],
        [0, 0, 1]
    ])

    # Combined rotation: R = Rz * Ry * Rx (ZYX Euler angles)
    return Rz @ Ry @ Rx

def rotate_point(point, R):
    """Apply rotation matrix to a 3D point."""
    return R @ np.array(point)

def draw_angle_arc_3d(ax, center, radius, start_vec, end_vec, normal, color, label):
    """Draw an arc showing an angle in 3D space with proper sign handling."""
    # Normalize vectors
    start_vec = start_vec / np.linalg.norm(start_vec)
    end_vec = end_vec / np.linalg.norm(end_vec)
    normal = normal / np.linalg.norm(normal)

    # Calculate angle magnitude (always positive from arccos)
    angle_magnitude = np.arccos(np.clip(np.dot(start_vec, end_vec), -1.0, 1.0))

    # Determine the sign of rotation using cross product
    # cross(start, end) · normal tells us the direction
    cross_prod = np.cross(start_vec, end_vec)
    direction = np.dot(cross_prod, normal)

    # If direction is negative, we rotate in opposite direction
    # This means we go the "long way" around the circle
    if direction < 0:
        angle_magnitude = -angle_magnitude

    # Generate arc points
    n_points = 20
    angles = np.linspace(0, angle_magnitude, n_points)

    # Create arc in local plane using Rodrigues' rotation formula
    arc_points = []
    for a in angles:
        v = start_vec * np.cos(a) + np.cross(normal, start_vec) * np.sin(a) + \
            normal * np.dot(normal, start_vec) * (1 - np.cos(a))
        arc_points.append(center + radius * v)

    arc_points = np.array(arc_points)
    ax.plot(arc_points[:, 1], arc_points[:, 0], arc_points[:, 2],
            color=color, linewidth=2, alpha=0.8)

    # Add label at mid-arc
    if len(arc_points) > 0:
        mid_point = arc_points[len(arc_points) // 2]
        ax.text(mid_point[1], mid_point[0], mid_point[2], label,
                fontsize=11, color=color, weight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=color, alpha=0.8))

# Display physical limits
print("\n")
print_limits()
print("\n")

print("3D model (full 6DOF): n=12 (simple) or n=16 (with thrust dynamics)")
thrust_dyn_input = input("Include thrust dynamics? (t/n, default=t): ").strip().lower()

if thrust_dyn_input in ['n', 'no', 'nie']:
    n = 12  # 3D without thrust dynamics
else:
    n = 16  # 3D with thrust dynamics

m = 4  # dimension of control input (4 motors)
print(f"Model configuration: n={n}, m={m}")

# Integration parameters
dt = input(f"Podaj krok czasowy dt [s] (domyślnie {0.075}): ")
if dt.strip() == "":
    dt = 0.075
else:
    dt = float(dt)

# Starting altitude input - USER FRIENDLY (positive = above ground)
z_start_input = input(f"Podaj wysokość startową nad ziemią [m] (domyślnie {1.0}): ")
if z_start_input.strip() == "":
    z_start_input = 1.0  # Altitude Above Ground Level
else:
    z_start_input = float(z_start_input)

# Validate input
if z_start_input < 0:
    print(f"\nBŁĄD: Wysokość nie może być ujemna! Ustawiam na 0m (ziemia).")
    z_start_input = 0.0

# Engine startup logic
if z_start_input == 0.0:
    engines_on_start = False
    print("\nDron startuje z ziemi - silniki wyłączone.")
else:
    engine_input = input(f"\nDron startuje na wysokości {z_start_input:.1f}m. Czy silniki mają być włączone? (t/n, domyślnie n): ")
    if engine_input.strip().lower() in ['t', 'tak', 'y', 'yes']:
        engines_on_start = True
        print("Silniki będą włączone (nominalny ciąg zawieszenia).")
    else:
        engines_on_start = False
        print("Silniki będą wyłączone - dron zacznie spadać.")


# ============================================================================
# INITIAL STATE - QUADCOPTER (4 motors, 3D)
# ============================================================================

t = 0.0 #czas początkowy
z_initial = -z_start_input  # NED: Z = -altitude
y_initial = 0.0  # Start centered

if engines_on_start:
    initial_thrust = MASS * g / 4.0  # Nominal hover thrust per motor
else:
    initial_thrust = 0.0  # Motors off

if n == 12:
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
    raise ValueError(f"n must be 12 or 16. Got n={n}")

# Control input - start with initial thrust setting (4 motors)
u = np.array([initial_thrust, initial_thrust, initial_thrust, initial_thrust])

# Print startup configuration
print("\n" + "="*70)
print("QUADCOPTER LQR SIMULATION - NED COORDINATE SYSTEM")
print("="*70)
print(f"Configuration: X-config with 4 motors, 3D full 6DOF")
print(f"Model: n = {n} ({'with thrust dynamics' if n==16 else 'simple model'})")
print(f"Starting altitude: {z_start_input:.2f} m (above ground)")
print(f"Initial thrust per motor: {initial_thrust:.2f} N")
print(f"Nominal hover thrust per motor: {MASS * g / 4.0:.2f} N")
print(f"Total nominal thrust: {MASS * g:.2f} N")
print(f"Time step: {dt} s")
print("="*70)


# ============================================================================
# REFERENCE TRAJECTORY
# ============================================================================

X_ref_all, Y_terr_all, Z_terr_all, Y_ref_all, Z_ref_all, alpha_all, beta_all = generate_reference_profile(Vel, dt, 50)

# Storage for plotting
tp = []
yp = []
up = []
limit_violations_log = []  # Log of limit violations

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

offset_x = 0.2
offset_y = 0.2
# Subplot 4: Drone Orientation (bottom right) - 3D VIEW
ax4 = plt.subplot(2, 2, 4, projection='3d')
ax4.set_xlim(-MOTOR_ARM_LENGTH_X - offset_x, MOTOR_ARM_LENGTH_X + offset_x)
ax4.set_ylim(-MOTOR_ARM_LENGTH_Y - offset_y, MOTOR_ARM_LENGTH_Y + offset_y)
ax4.set_zlim(-3, 3)
ax4.set_xticks(np.arange(-MOTOR_ARM_LENGTH_X - offset_x, MOTOR_ARM_LENGTH_X + offset_x, 0.5))
ax4.set_yticks(np.arange(-MOTOR_ARM_LENGTH_Y - offset_y, MOTOR_ARM_LENGTH_Y, 0.5))
ax4.set_zticks(np.arange(-3.0, 3.0, 1.0))
ax4.set_xlabel('Y [m]')
ax4.set_ylabel('X [m]')
ax4.set_zlabel('Z [m]')
ax4.set_title('Quadcopter 3D Orientation (Full Rotation)')
ax4.view_init(elev=20, azim=45)

plt.tight_layout()
plt.draw()
plt.pause(1)




# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================
# Reference angles (for simplicity, keep level flight)
phi_ref = 0.0
theta_ref = 0.0
psi_ref = 0.0
x_ref = 0.0
y_ref = 0.0
limits_exceeded_count = 0

for i in range(10000):
    # Exit condition
    if t >= 50.0 or x[6] >= 50:
        break
    
    X = x[6]  # Current forward position

    # === 1. Find reference point ===
    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]
    y_terr = Y_terr_all[idx_ref]
    y_ref = Y_ref_all[idx_ref]
    beta = beta_all[idx_ref]

    # === 2. Reference velocity components ===
    Vx_ref = Vel * np.cos(alfa)
    Vz_ref = Vel * np.sin(alfa)
    Vy_ref = Vel * np.sin(beta) * np.cos(alfa)  # Lateral velocity component

    # === 3. Update reference position ===
    x_ref += Vel * np.cos(alfa) * dt

   # === 4. Collect data ===
    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    R = np.eye(m) * 1.0
    Q = np.eye(n)
    e = np.zeros(n)

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
    
    # 3D error vector
    e[0] = x[0] - Vx_ref
    e[1] = x[1] - Vy_ref
    e[2] = x[2] - Vz_ref
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
        e[12] = x[12] - (NOMINAL_thrust)
        e[13] = x[13] - (NOMINAL_thrust)
        e[14] = x[14] - (NOMINAL_thrust)
        e[15] = x[15] - (NOMINAL_thrust)

    # ========================================================================
    # LQR CONTROLLER
    # ========================================================================

    A, B = aa_matrices_AB("rhs", x, t, u, n, m)
    K, P = lqr_m(A, B, Q, R)

    u_pert = -K @ e

    T1_desired = NOMINAL_thrust + u_pert[0]
    T2_desired = NOMINAL_thrust + u_pert[1]
    T3_desired = NOMINAL_thrust + u_pert[2]
    T4_desired = NOMINAL_thrust + u_pert[3]

    T1 = np.clip(T1_desired, T_min, T_max)
    T2 = np.clip(T2_desired, T_min, T_max)
    T3 = np.clip(T3_desired, T_min, T_max)
    T4 = np.clip(T4_desired, T_min, T_max)

    u = np.array([T1, T2, T3, T4])

    if X < 2:
        print("======" * 50)
        print(f't = {t:.3f}s')
        print(f'pozycja: x = {x[6]:.2f}m, y = {x[7]:.2f}m, z_NED = {x[8]:.2f}m, AGL = {-x[8]:.2f}m')
        print(f'prędkość vx = {x[0]:.2f}m/s, vy = {x[1]:.2f}m/s, vz = {x[2]:.2f}m/s')
        print(f'kąty: phi = {math.degrees(x[9]):.2f}°, theta = {math.degrees(x[10]):.2f}°, psi = {math.degrees(x[11]):.2f}°')
        print(f'prędkości kątowe: p = {math.degrees(x[3]):.2f}°/s, q = {math.degrees(x[4]):.2f}°/s, r = {math.degrees(x[5]):.2f}°/s')
        print()
        #diagnostyka_sil(x, u)

    # ========================================================================
    # ENVIRONMENTAL EFFECTS
    # ========================================================================

    az_turbulence = 0.0
    ax_wind = 0.0
    az_wind = 0.0

    if X > X_turb_1 and X < X_turb_2:
        az_turbulence = c_turb * (1.0 - 2.0 * np.random.rand())

    x = aa_rk45("rhs", x, t, dt, u, az_turbulence, ax_wind, az_wind)

    # ========================================================================
    # APPLY PHYSICAL LIMITS ⚠️
    # ========================================================================
    
    x_before_limit = x.copy()
    x = apply_all_limits(x)
    
    # Check if limits were exceeded
    limits_exceeded, violations = check_limits_exceeded(x_before_limit, verbose=False)
    
    if limits_exceeded:
        limits_exceeded_count += 1
        limit_violations_log.append((t, violations))
        
        # Print warning every 100 limit exceedances (not every time to avoid spam)
        if limits_exceeded_count % 100 == 1:
            print(f"\n⚠️  PHYSICAL LIMITS EXCEEDED at t={t:.2f}s:")
            for v in violations:
                print(f"   {v}")

    # ========================================================================
    # GROUND COLLISION PREVENTION (NED: Z >= 0 means at/below ground)
    # ========================================================================
    
    if x[8] >= 0.0:  # If at or below ground level
        x[8] = 0.0  # Keep at ground level
        x[2] = max(0.0, x[2])  # Stop upward velocity, allow downward
        if x[0] > 0:  # Friction slows down horizontal movement
            x[0] *= 0.95
        x[4] = 0.0  # Stop pitch rotation

    # ========================================================================
    # VISUALIZATION
    # ========================================================================

    V = np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)
    phi = x[9]
    theta = x[10]
    psi = x[11]
    phi_deg = math.degrees(phi)
    theta_deg = math.degrees(theta)
    psi_deg = math.degrees(psi)
    alt = -x[8]  # Altitude above ground (AGL) = -Z in NED
    X_pos = x[6]
    Y_pos = x[7]
    Z_pos = x[8]
    theta_vis = x[10]  # Use theta for visualization

    xs, zs = aa_mdl(X_pos, Z_pos, theta_vis, 0.50)

    # ====================================================================
    # UPDATE ALL SUBPLOTS
    # ====================================================================

    yp_array = np.array(yp)

    # ====================================================================
    # UPDATE SUBPLOT 1: Z(X) TRAJECTORY
    # ====================================================================

    ax1.clear()
    ax1.plot(X_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
    ax1.plot(X_ref_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
    ax1.axhline(y=0, color='brown', linestyle='--', linewidth=1.5, label='Ground', alpha=0.7)

    # Plot flight path
    ax1.plot(yp_array[:, 6], yp_array[:, 8], 'b', label='Flight path', alpha=0.7)
    ax1.plot(yp_array[i, 6], yp_array[i, 8], 'bo', markersize=8)

    # Plot drone model
    ax1.plot(xs[:5], zs[:5], 'k', linewidth=3)

    # Add limit violation indicator
    limit_indicator = " ⚠️ LIMITS!" if limits_exceeded else ""
    txt = (f't={t:7.3f}s | V={V:7.3f}m/s{limit_indicator}\n'
           f'φ={phi_deg:6.2f}° θ={theta_deg:6.2f}° ψ={psi_deg:6.2f}° | '
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
    ax2.plot(yp_array[:, 6], yp_array[:, 7], 'b', label='Flight path Y', alpha=0.7)
    ax2.plot(yp_array[i, 6], yp_array[i, 7], 'bo', markersize=8)
    
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('Trajektoria Y(X)', fontsize=10)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.axis([max(0, X-10), X+10, -10, 10])

    # ====================================================================
    # UPDATE SUBPLOT 3: 3D TRAJECTORY
    # ====================================================================

    ax3.clear()
    ax3.plot(X_ref_all, Y_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
    ax3.plot(X_ref_all, Y_terr_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
    ax3.plot(yp_array[:, 6], yp_array[:, 7], yp_array[:, 8], 'b', label='Flight path', alpha=0.7, linewidth=2)
    ax3.scatter([yp_array[i, 6]], [yp_array[i, 7]], [yp_array[i, 8]], color='blue', s=100)
    
    ax3.set_xlabel('X [m]')
    ax3.set_ylabel('Y [m]')
    ax3.set_zlabel('Z [m]')
    ax3.set_title('Trajektoria 3D (X, Y, Z)', fontsize=10)
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.invert_zaxis()

    # ====================================================================
    # UPDATE SUBPLOT 4: QUADCOPTER 3D ORIENTATION WITH FULL ROTATION
    # ====================================================================

    ax4.clear()
    ax4.set_xlim(-MOTOR_ARM_LENGTH_X - offset_x, MOTOR_ARM_LENGTH_X + offset_x)
    ax4.set_ylim(-MOTOR_ARM_LENGTH_Y - offset_y, MOTOR_ARM_LENGTH_Y + offset_y)
    ax4.set_zlim(-3, 3)
    ax4.set_xlabel('Y [m]', fontsize=11, weight='bold')
    ax4.set_ylabel('X [m]', fontsize=11, weight='bold')
    ax4.set_zlabel('Z [m]', fontsize=11, weight='bold')
    ax4.view_init(elev=20, azim=45)
    ax4.set_xticks(np.arange(-MOTOR_ARM_LENGTH_X - offset_x, MOTOR_ARM_LENGTH_X + offset_x, 0.5))
    ax4.set_yticks(np.arange(-MOTOR_ARM_LENGTH_Y - offset_y, MOTOR_ARM_LENGTH_Y, 0.5))
    ax4.set_zticks(np.arange(-3.0, 3.0, 1.0))

    # Draw world coordinate axes
    axis_len = 5
    ax4.plot([0, 0], [0, axis_len], [0, 0], 'r-', linewidth=3, alpha=0.7, label='X (world)')
    ax4.plot([0, axis_len], [0, 0], [0, 0], 'g-', linewidth=3, alpha=0.7, label='Y (world)')
    ax4.plot([0, 0], [0, 0], [0, axis_len], 'b-', linewidth=3, alpha=0.7, label='Z (world)')
    ax4.text(0, axis_len+0.2, 0, 'X', fontsize=14, color='red', weight='bold')
    ax4.text(axis_len+0.2, 0, 0, 'Y', fontsize=14, color='green', weight='bold')
    ax4.text(0, 0, axis_len+0.2, 'Z', fontsize=14, color='blue', weight='bold')

    # Get full rotation matrix using all three Euler angles (aerospace convention)
    R = rotation_matrix_zyx(phi, theta, psi)

    # Drone dimensions
    arm_length = MOTOR_ARM_LENGTH_X

    # Motor positions in body frame (X configuration)
    angle_offset = np.pi / 4  # 45 degrees
    motors_body = [
        np.array([arm_length * np.cos(angle_offset), arm_length * np.sin(angle_offset), 0]),
        np.array([arm_length * np.cos(angle_offset), -arm_length * np.sin(angle_offset), 0]),
        np.array([-arm_length * np.cos(angle_offset), arm_length * np.sin(angle_offset), 0]),
        np.array([-arm_length * np.cos(angle_offset), -arm_length * np.sin(angle_offset), 0])
    ]

    # Rotate motors to world frame using full rotation matrix
    motors_world = [rotate_point(m, R) for m in motors_body]

    # Draw drone arms (X shape)
    ax4.plot([motors_world[0][1], motors_world[3][1]],
             [motors_world[0][0], motors_world[3][0]],
             [motors_world[0][2], motors_world[3][2]],
             'k-', linewidth=3, alpha=0.9)
    ax4.plot([motors_world[1][1], motors_world[2][1]],
             [motors_world[1][0], motors_world[2][0]],
             [motors_world[1][2], motors_world[2][2]],
             'k-', linewidth=3, alpha=0.9)

    # Draw center body (cube) with full rotation
    body_size = MOTOR_ARM_LENGTH_X/4
    r = body_size / 2
    vertices_body = [
        np.array([-r, -r, -r]), np.array([r, -r, -r]), np.array([r, r, -r]), np.array([-r, r, -r]),
        np.array([-r, -r, r]), np.array([r, -r, r]), np.array([r, r, r]), np.array([-r, r, r])
    ]
    vertices_world = [rotate_point(v, R) for v in vertices_body]

    faces = [
        [vertices_world[0], vertices_world[1], vertices_world[2], vertices_world[3]],
        [vertices_world[4], vertices_world[5], vertices_world[6], vertices_world[7]],
        [vertices_world[0], vertices_world[1], vertices_world[5], vertices_world[4]],
        [vertices_world[2], vertices_world[3], vertices_world[7], vertices_world[6]],
        [vertices_world[0], vertices_world[3], vertices_world[7], vertices_world[4]],
        [vertices_world[1], vertices_world[2], vertices_world[6], vertices_world[5]]
    ]

    cube = Poly3DCollection(faces, alpha=0.8, facecolor='royalblue', edgecolor='navy', linewidth=2)
    ax4.add_collection3d(cube)

    # Draw motors (WITHOUT thrust arrows)
    thrusts = [T1, T2, T3, T4]
    motor_colors = ['red', 'blue', 'green', 'orange']

    for j, (m_pos, T) in enumerate(zip(motors_world, thrusts)):
        ax4.scatter([m_pos[1]], [m_pos[0]], [m_pos[2]],
                   color=motor_colors[j], s=200,
                   edgecolors='black', linewidths=2, depthshade=True, zorder=10)

    # Draw body axes (rotated with full rotation)
    axis_body_len = MOTOR_ARM_LENGTH_X - 0.3

    # X_body (forward) - RED
    x_body_dir = rotate_point([axis_body_len, 0, 0], R)
    ax4.quiver(0, 0, 0, x_body_dir[1], x_body_dir[0], x_body_dir[2],
              color='darkred', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
    ax4.text(x_body_dir[1]*1.1, x_body_dir[0]*1.1, x_body_dir[2]*1.1,
            'X\'', fontsize=12, color='darkred', weight='bold')

    # Y_body (right) - GREEN
    y_body_dir = rotate_point([0, axis_body_len, 0], R)
    ax4.quiver(0, 0, 0, y_body_dir[1], y_body_dir[0], y_body_dir[2],
              color='darkgreen', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
    ax4.text(y_body_dir[1]*1.1, y_body_dir[0]*1.1, y_body_dir[2]*1.1,
            'Y\'', fontsize=12, color='darkgreen', weight='bold')

    # Z_body (down) - BLUE
    z_body_dir = rotate_point([0, 0, axis_body_len], R)
    ax4.quiver(0, 0, 0, z_body_dir[1], z_body_dir[0], z_body_dir[2],
              color='darkblue', arrow_length_ratio=0.2, linewidth=3, linestyle='--', alpha=0.9)
    ax4.text(z_body_dir[1]*1.1, z_body_dir[0]*1.1, z_body_dir[2]*1.1,
            'Z\'', fontsize=12, color='darkblue', weight='bold')

    # Draw angle indicators for phi, theta, psi with DIFFERENT RADII to avoid overlap
    
    # Roll angle (phi) - rotation around X axis - smallest radius
    if abs(phi_deg) > 0.1:  # Only show if significant
        draw_angle_arc_3d(ax4, np.array([0, 0, 0]), 3,
                         np.array([0, 1, 0]), rotate_point([0, 1, 0], R),
                         np.array([1, 0, 0]), 'purple', f'φ={phi_deg:.1f}°')
    
    # Pitch angle (theta) - rotation around Y axis - medium radius
    if abs(theta_deg) > 0.1:  # Only show if significant
        draw_angle_arc_3d(ax4, np.array([0, 0, 0]), 4,
                         np.array([1, 0, 0]), rotate_point([1, 0, 0], R),
                         np.array([0, 1, 0]), 'firebrick', f'θ={theta_deg:.1f}°')
    
    # Yaw angle (psi) - rotation around Z axis - largest radius
    if abs(psi_deg) > 0.1:  # Only show if significant
        draw_angle_arc_3d(ax4, np.array([0, 0, 0]), 2,
                         np.array([1, 0, 0]), rotate_point([1, 0, 0], R),
                         np.array([0, 0, 1]), 'mediumblue', f'ψ={psi_deg:.1f}°')

    # Info box with orientation data and LIMITS STATUS
    v_horiz = np.sqrt(x[0]**2 + x[1]**2)
    info_text = (f'Roll φ = {phi_deg:.2f}°\n'
                f'Pitch θ = {theta_deg:.2f}°\n'
                f'Yaw ψ = {psi_deg:.2f}°\n'
                f'p = {math.degrees(x[3]):.2f}°/s \n'
                f'q = {math.degrees(x[4]):.2f}°/s \n'
                f'r = {math.degrees(x[5]):.2f}°/s \n'
                )
    
    box_color = 'red' if limits_exceeded else 'wheat'
    ax4.text2D(0.02, 1.02, info_text, transform=ax4.transAxes,
              fontsize=9, verticalalignment='top', 
              bbox=dict(boxstyle='round', facecolor=box_color, alpha=0.9, 
                       edgecolor='black', linewidth=2), 
              family='monospace', weight='bold')
    
    # Add motor color legend with thrust values
    legend_elements = [
        Patch(facecolor=motor_colors[0], edgecolor='black', label=f'M1: {T1:.1f}N'),
        Patch(facecolor=motor_colors[1], edgecolor='black', label=f'M2: {T2:.1f}N'),
        Patch(facecolor=motor_colors[2], edgecolor='black', label=f'M3: {T3:.1f}N'),
        Patch(facecolor=motor_colors[3], edgecolor='black', label=f'M4: {T4:.1f}N')
    ]
    ax4.legend(handles=legend_elements, loc='upper right', fontsize=9, framealpha=0.9)

    plt.draw()
    plt.pause(0.001)

    t = t + dt

# ============================================================================
# SIMULATION SUMMARY
# ============================================================================

print("\n" + "="*70)
print("SIMULATION COMPLETE - SUMMARY")
print("="*70)
print(f"Total simulation time: {t:.2f} s")
print(f"Total iterations: {i+1}")
print(f"Physical limits exceeded: {limits_exceeded_count} times")

if limit_violations_log:
    print(f"\nFirst limit violation at t={limit_violations_log[0][0]:.2f}s:")
    for v in limit_violations_log[0][1]:
        print(f"  {v}")

print("="*70)

plt.ioff()
plt.show()
