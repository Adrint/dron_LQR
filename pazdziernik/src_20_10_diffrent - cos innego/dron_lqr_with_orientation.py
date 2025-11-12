import numpy as np
import math
import matplotlib.pyplot as plt
from rk45 import aa_rk45
from matrices import aa_matrices_AB
from model_draw import aa_mdl
from lqr import lqr_m
from trajectory import generate_reference_profile
from rhs import diagnostyka_sil
from constants import (z0, Vel, X_turb_1, X_turb_2, c_turb, MASS, g, T_max, T_min, TAU)

# ============================================================================
# CONFIGURATION
# ============================================================================

n = 8  # 8: with thrust dynamics
m = 2

dt = input(f"Podaj krok czasowy dt [s] (domyślnie {0.01}): ")
if dt.strip() == "":
    dt = 0.01
else:
    dt = float(dt)

t = 0.0

# ============================================================================
# INITIAL STATE (NED: Z ujemne = wysoko)
# ============================================================================

if n == 6:
    x = np.array([0.0, 0.0, 0.0, 0.0, z0, 0.0])
elif n == 8:
    x = np.array([0.0, 0.0, 0.0, 0.0, z0, 0.0, MASS * g / 2, MASS * g / 2])
else:
    raise ValueError("n must be 6 or 8")

u = np.array([MASS * g / 2.0, MASS * g / 2.0])

# ============================================================================
# REFERENCE TRAJECTORY
# ============================================================================

X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)

tp = []
yp = []
up = []

# ============================================================================
# PLOTTING SETUP
# ============================================================================

plt.ion()
fig = plt.figure(1, figsize=(16, 6))

# Subplot 1: Trajectory
ax1 = plt.subplot(1, 2, 1)
# Konwersja do "normalnych" wysokości dla wyświetlania
ax1.plot(X_ref_all, -Z_ref_all, 'r', label='Reference', linewidth=2)
ax1.plot(X_ref_all, -Z_terr_all, 'g', label='Terrain', linewidth=2)
ax1.legend(fontsize=11)
ax1.axis([0, 50.0, 0.0, 10.0])
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Altitude [m]')
ax1.set_title('Drone Flight Simulation (NED)')
ax1.grid(True, alpha=0.3)

# Subplot 2: Drone Orientation
ax2 = plt.subplot(1, 2, 2)
ax2.set_xlim(-2, 2)
ax2.set_ylim(-2, 2)
ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3)
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Z [m]')
ax2.set_title('Drone Orientation')

ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
ax2.axvline(x=0, color='k', linestyle='-', linewidth=0.5)

plt.draw()
plt.pause(1)

x_ref = 0.0


T_max_absolute = T_max
T_min_absolute = T_min

# ============================================================================
# MAIN SIMULATION LOOP
# ============================================================================

for i in range(10000):
    if t >= 50.0 or x[3] >= 50:
        break

    X = x[3]

    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]

    Vx_ref = Vel * np.cos(alfa)
    Vz_ref = Vel * np.sin(alfa)

    x_ref += Vel * np.cos(alfa) * dt
    vx_ref = np.cos(x[5]) * Vx_ref - np.sin(x[5]) * Vz_ref  # NED correction
    vz_ref = np.sin(x[5]) * Vx_ref + np.cos(x[5]) * Vz_ref  # NED correction

    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    # LQR
    R = np.eye(m) * 1.0
    Q = np.eye(n)
    Q[0, 0] = 5.0
    Q[1, 1] = 5.0
    Q[2, 2] = 0.1
    Q[3, 3] = 50.0
    Q[4, 4] = 50.0
    Q[5, 5] = 10.0

    if n == 8:
        Q[6, 6] = 0.1
        Q[7, 7] = 0.1
        Q = 10 * Q

    e = np.zeros(n)
    e[0] = x[0] - vx_ref
    e[1] = x[1] - vz_ref
    e[2] = x[2] - 0.0
    e[3] = x[3] - x_ref
    e[4] = x[4] - z_ref
    e[5] = x[5] - 0.0

    if n == 8:
        e[6] = x[6] - (MASS * g / 2.0)
        e[7] = x[7] - (MASS * g / 2.0)

    A, B = aa_matrices_AB("rhs", x, t, u, n, m)
    K, P = lqr_m(A, B, Q, R)

    u_pert = -K @ e

    T1_desired = u_pert[0]
    T2_desired = u_pert[1]

    T1 = np.clip(T1_desired, T_min_absolute, T_max_absolute)
    T2 = np.clip(T2_desired, T_min_absolute, T_max_absolute)

    u = np.array([T1, T2])

    if x[3] < 5:
        print(f't = {t:.3f}s, X = {x[3]:.3f}m, h = {-x[4]:.3f}m, θ = {np.degrees(x[5]):.2f}°')
        diagnostyka_sil(x, u)

    az_turbulence = 0.0
    ax_wind = 0.0
    az_wind = 0.0

    x = aa_rk45("rhs", x, t, dt, u, az_turbulence, ax_wind, az_wind)

    # Visualization
    V = np.sqrt(x[0] ** 2 + x[1] ** 2)
    teta = math.degrees(x[5])
    alt = -x[4]  # Wysokość = -Z w NED

    xs, zs = aa_mdl(x[3], -x[4], x[5], 0.50)  # Konwersja Z dla rysowania

    # Update trajectory plot
    ax1.clear()
    ax1.plot(X_ref_all, -Z_ref_all, 'r', label='Reference', linewidth=2)
    ax1.plot(X_ref_all, -Z_terr_all, 'g', label='Terrain', linewidth=2)

    yp_array = np.array(yp)
    ax1.plot(yp_array[:, 3], -yp_array[:, 4], 'b', label='Flight path', alpha=0.7)
    ax1.plot(yp_array[i, 3], -yp_array[i, 4], 'bo', markersize=8)

    ax1.plot(xs[:5], zs[:5], 'k', linewidth=3)

    txt = (f't={t:7.3f}s V={V:7.3f}m/s θ={teta:7.2f}° h={alt:7.3f}m '
           f'| T1={T1:7.1f}N T2={T2:7.1f}N')
    ax1.set_title(txt, fontsize=10)
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axis([max(0, X-10), X+10, 0.0, 10.0])
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Altitude [m]')

    # Update orientation plot
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

    # Drone dimensions (ZWIĘKSZONE dla lepszej widoczności)
    drone_width = 1.2  # Było 0.6
    drone_height = 0.4  # Było 0.2
    motor_offset = 0.5  # Było 0.25 - Distance from center to motor

    theta = x[5]  # Current angle

    # Drone body (rectangle centered at origin)
    # Create rectangle in local coordinates
    rect_corners_x = np.array([-drone_width / 2, drone_width / 2, drone_width / 2, -drone_width / 2, -drone_width / 2])
    rect_corners_z = np.array(
        [-drone_height / 2, -drone_height / 2, drone_height / 2, drone_height / 2, -drone_height / 2])

    # Rotate
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    rect_x_rot = cos_t * rect_corners_x - sin_t * rect_corners_z
    rect_z_rot = sin_t * rect_corners_x + cos_t * rect_corners_z

    # Draw drone body
    ax2.fill(rect_x_rot, rect_z_rot, color='royalblue', alpha=0.7, edgecolor='navy', linewidth=2)

    # Motor positions (left and right)
    motor1_x_local = -motor_offset
    motor2_x_local = +motor_offset
    motor_z_local = 0

    # Rotate motor positions
    motor1_x = cos_t * motor1_x_local - sin_t * motor_z_local
    motor1_z = sin_t * motor1_x_local + cos_t * motor_z_local

    motor2_x = cos_t * motor2_x_local - sin_t * motor_z_local
    motor2_z = sin_t * motor2_x_local + cos_t * motor_z_local

    # Draw motors
    ax2.plot(motor1_x, motor1_z, 'ko', markersize=8, label='Motor 1')
    ax2.plot(motor2_x, motor2_z, 'ko', markersize=8, label='Motor 2')

    # Thrust vectors (in drone's frame, pointing "up" in drone coordinates)
    thrust_scale = 0.002  # Scale factor for visualization (zmniejszone bo dron większy)

    # Thrust direction in drone frame: (0, 1) = upward
    # Rotate to world frame
    thrust1_x = -sin_t * thrust_scale * T1
    thrust1_z = cos_t * thrust_scale * T1

    thrust2_x = -sin_t * thrust_scale * T2
    thrust2_z = cos_t * thrust_scale * T2

    # Draw thrust arrows
    ax2.arrow(motor1_x, motor1_z, thrust1_x, thrust1_z,
              head_width=0.1, head_length=0.1, fc='red', ec='darkred', linewidth=2)
    ax2.arrow(motor2_x, motor2_z, thrust2_x, thrust2_z,
              head_width=0.1, head_length=0.1, fc='red', ec='darkred', linewidth=2)

    # Add thrust value labels
    ax2.text(motor1_x + thrust1_x, motor1_z + thrust1_z + 0.15,
             f'T1={T1:.1f}N', fontsize=9, ha='center', color='red', weight='bold')
    ax2.text(motor2_x + thrust2_x, motor2_z + thrust2_z + 0.15,
             f'T2={T2:.1f}N', fontsize=9, ha='center', color='red', weight='bold')

    # Draw body axes (X_body and Z_body)
    axis_length = 1.0  # Było 0.8
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
    from matplotlib.patches import Arc

    if abs(theta) > 0.01:
        arc = Arc((0, 0), 0.6, 0.6, angle=0, theta1=0, theta2=np.degrees(theta),
                  color='orange', linewidth=2)
        ax2.add_patch(arc)
        ax2.text(0.4, 0.15 if theta > 0 else -0.15, f'θ={teta:.1f}°',
                 fontsize=10, color='orange', weight='bold')

    ax2.set_title(f'Drone Orientation | θ={teta:.2f}° | ω={math.degrees(x[2]):.2f}°/s',
                  fontsize=11, weight='bold')
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Z [m]')

    plt.draw()
    plt.pause(0.001)

t = t + dt

plt.ioff()
plt.show()
