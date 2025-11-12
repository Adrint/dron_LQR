import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rk45 import aa_rk45
from matrices import aa_matrices_AB
from lqr import lqr_m
from trajectory import generate_reference_profile
from rhs import diagnostyka_sil
from constants import (z0, Vel, X_turb_1, X_turb_2, c_turb, MASS, g, T_max, T_min, TAU)

# ============================================================================
# CONFIGURATION
# ============================================================================

n = 10
m = 4

dt = input(f"Podaj krok czasowy dt [s] (domyślnie {0.01}): ")
if dt.strip() == "":
    dt = 0.01
else:
    dt = float(dt)

z_start_input = input(f"Podaj wysokość startową nad ziemią [m] (domyślnie {0.0}): ")
if z_start_input.strip() == "":
    z_start_AGL = 0.0
else:
    z_start_AGL = float(z_start_input)

if z_start_AGL < 0:
    z_start_AGL = 0.0

z_start = -z_start_AGL

if z_start_AGL == 0.0:
    engines_on_start = False
    print("\nDron startuje z ziemi - silniki wyłączone.")
else:
    engine_input = input(f"\nDron startuje na wysokości {z_start_AGL:.1f}m. Czy silniki mają być włączone? (t/n): ")
    engines_on_start = engine_input.strip().lower() in ['t', 'tak', 'y', 'yes']
    print("Silniki będą " + ("włączone." if engines_on_start else "wyłączone."))

t = 0.0
z_initial = z_start
initial_thrust = (MASS * g / 4.0) if engines_on_start else 0.0

if n == 10:
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z_initial, 0.0, 0.0])
elif n == 14:
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z_initial, 0.0, 0.0,
                  initial_thrust, initial_thrust, initial_thrust, initial_thrust])

u = np.array([initial_thrust, initial_thrust, initial_thrust, initial_thrust])

print("\n" + "="*70)
print("QUADCOPTER 3D FLIGHT SIMULATION")
print("="*70)
print(f"Model: n={n}, dt={dt}s, Start altitude={z_start_AGL:.2f}m")
print("="*70 + "\n")

# ============================================================================
# REFERENCE TRAJECTORY
# ============================================================================

X_ref_all, Y_ref_all, Z_terr_all, Z_ref_all, alpha_all, beta_all = generate_reference_profile(Vel, dt, 50)

tp = []
yp = []
up = []

# ============================================================================
# PLOTTING
# ============================================================================

plt.ion()
fig = plt.figure(1, figsize=(16, 6))

ax1 = plt.subplot(1, 2, 1, projection='3d')
ax1.plot(X_ref_all, Y_ref_all, Z_ref_all, 'r', label='Reference', linewidth=2)
ax1.plot(X_ref_all, Y_ref_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
xx, yy = np.meshgrid(np.linspace(0, 50, 10), np.linspace(-5, 5, 10))
zz = np.zeros_like(xx)
ax1.plot_surface(xx, yy, zz, alpha=0.2, color='brown')
ax1.legend(fontsize=11)
ax1.set_xlim([0, 50])
ax1.set_ylim([-5, 5])
ax1.set_zlim([-25, 0.5])
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_zlabel('Z [m]')
ax1.view_init(elev=25, azim=-60)
ax1.invert_zaxis()

ax2 = plt.subplot(1, 2, 2, projection='3d')
ax2.set_xlim(-2, 2)
ax2.set_ylim(-2, 2)
ax2.set_zlim(-2, 2)
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Y [m]')
ax2.set_zlabel('Z [m]')
ax2.view_init(elev=20, azim=45)

plt.draw()
plt.pause(1)

T_max_absolute = T_max
T_min_absolute = T_min

# ============================================================================
# MAIN LOOP
# ============================================================================

for i in range(10000):
    if t >= 50.0 or x[5] >= 50:
        break

    X_pos = x[5]
    Y_pos = x[6]
    Z_pos = x[7]

    # Find closest point on trajectory
    distances = np.sqrt((X_ref_all - X_pos)**2 + (Y_ref_all - Y_pos)**2 + (Z_ref_all - Z_pos)**2)
    idx_ref = np.argmin(distances)
    
    x_ref = X_ref_all[idx_ref]
    y_ref = Y_ref_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]
    beta = beta_all[idx_ref]

    # Reference velocities in NED frame
    denominator = np.sqrt(1.0 + np.tan(beta)**2 + np.tan(alfa)**2)
    Vx_ref_NED = Vel / denominator
    Vy_ref_NED = Vel * np.tan(beta) / denominator
    Vz_ref_NED = Vel * np.tan(alfa) / denominator

    # Transform to body frame
    theta = x[8]
    phi = x[9]
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    
    vx_ref = cos_theta * Vx_ref_NED - sin_theta * Vz_ref_NED
    vy_ref = sin_theta * sin_phi * Vx_ref_NED + cos_phi * Vy_ref_NED + cos_theta * sin_phi * Vz_ref_NED
    vz_ref = sin_theta * cos_phi * Vx_ref_NED - sin_phi * Vy_ref_NED + cos_theta * cos_phi * Vz_ref_NED

    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    # LQR Controller
    R = np.eye(m) * 100.0
    Q = np.eye(n) * 0.1
    Q[0, 0] = 50.0
    Q[1, 1] = 50.0
    Q[2, 2] = 50.0
    Q[3, 3] = 0.1
    Q[4, 4] = 0.1
    Q[5, 5] = 250.0
    Q[6, 6] = 250.0
    Q[7, 7] = 250.0
    Q[8, 8] = 1.0
    Q[9, 9] = 1.0

    if n == 14:
        Q[10, 10] = 0.1
        Q[11, 11] = 0.1
        Q[12, 12] = 0.1
        Q[13, 13] = 0.1
        Q = 10 * Q

    e = np.zeros(n)
    e[0] = x[0] - vx_ref
    e[1] = x[1] - vy_ref
    e[2] = x[2] - vz_ref
    e[3] = x[3]
    e[4] = x[4]
    e[5] = x[5] - x_ref
    e[6] = x[6] - y_ref
    e[7] = x[7] - z_ref
    e[8] = x[8]
    e[9] = x[9]

    if n == 14:
        e[10] = x[10] - (MASS * g / 4.0)
        e[11] = x[11] - (MASS * g / 4.0)
        e[12] = x[12] - (MASS * g / 4.0)
        e[13] = x[13] - (MASS * g / 4.0)

    A, B = aa_matrices_AB("rhs", x, t, u, n, m)
    K, P = lqr_m(A, B, Q, R)
    u_pert = -K @ e

    T_nominal = MASS * g / 4.0
    T1 = np.clip(T_nominal + u_pert[0], T_min_absolute, T_max_absolute)
    T2 = np.clip(T_nominal + u_pert[1], T_min_absolute, T_max_absolute)
    T3 = np.clip(T_nominal + u_pert[2], T_min_absolute, T_max_absolute)
    T4 = np.clip(T_nominal + u_pert[3], T_min_absolute, T_max_absolute)

    u = np.array([T1, T2, T3, T4])

    if X_pos < 2:
        print(f't={t:.3f}s X={X_pos:.2f} Y={Y_pos:.2f} Z={Z_pos:.2f} | '
              f'Ref: X={x_ref:.2f} Y={y_ref:.2f} Z={z_ref:.2f}')

    # Environment
    az_turbulence = 0.0
    ax_wind = 0.0
    az_wind = 0.0
    ay_wind = 0.0

    if X_turb_1 < X_pos < X_turb_2:
        az_turbulence = c_turb * (1.0 - 2.0 * np.random.rand())

    # Integrate
    x = aa_rk45("rhs", x, t, dt, u, az_turbulence, ax_wind, az_wind, ay_wind)

    # Ground collision
    if x[7] >= 0.0:
        x[7] = 0.0
        x[2] = max(0.0, x[2])
        x[0] *= 0.95
        x[1] *= 0.95
        x[3] = 0.0
        x[4] = 0.0

    # Visualization
    V = np.sqrt(x[0]**2 + x[1]**2 + x[2]**2)
    teta = math.degrees(x[8])
    phi_deg = math.degrees(x[9])
    alt = -x[7]

    ax1.clear()
    ax1.plot(X_ref_all, Y_ref_all, Z_ref_all, 'r', label='Ref', linewidth=2)
    ax1.plot(X_ref_all, Y_ref_all, Z_terr_all, 'g', label='Terrain', linewidth=2)
    xx, yy = np.meshgrid(np.linspace(max(0, X_pos-15), min(50, X_pos+15), 10), np.linspace(-5, 5, 10))
    zz = np.zeros_like(xx)
    ax1.plot_surface(xx, yy, zz, alpha=0.15, color='brown')

    if len(yp) > 0:
        yp_array = np.array(yp)
        ax1.plot(yp_array[:, 5], yp_array[:, 6], yp_array[:, 7], 'b', linewidth=2)
        ax1.scatter([yp_array[-1, 5]], [yp_array[-1, 6]], [yp_array[-1, 7]], 
                   color='blue', s=100, marker='o', edgecolors='white', linewidths=2)

    ax1.scatter([X_pos], [Y_pos], [Z_pos], color='darkblue', s=200, marker='o', 
               edgecolors='yellow', linewidths=3)

    ax1.set_title(f't={t:.2f}s V={V:.2f}m/s θ={teta:.1f}° φ={phi_deg:.1f}° AGL={alt:.2f}m\n'
                  f'X={X_pos:.1f}m Y={Y_pos:.1f}m T={T1+T2+T3+T4:.0f}N', fontsize=10)
    ax1.legend(fontsize=10)
    ax1.set_xlim([max(0, X_pos-10), min(50, X_pos+10)])
    ax1.set_ylim([-5, 5])
    ax1.set_zlim([-25, 0.5])
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.view_init(elev=25, azim=-60)
    ax1.invert_zaxis()

    ax2.clear()
    ax2.set_xlim(-2, 2)
    ax2.set_ylim(-2, 2)
    ax2.set_zlim(-2, 2)
    ax2.view_init(elev=20, azim=45)

    axis_len = 1.5
    ax2.plot([0, axis_len], [0, 0], [0, 0], 'r-', linewidth=2)
    ax2.plot([0, 0], [0, axis_len], [0, 0], 'g-', linewidth=2)
    ax2.plot([0, 0], [0, 0], [0, axis_len], 'b-', linewidth=2)

    arm_length = 0.6
    angle_offset = np.pi / 4
    motors_body = [
        (arm_length * np.cos(angle_offset), arm_length * np.sin(angle_offset), 0),
        (arm_length * np.cos(angle_offset), -arm_length * np.sin(angle_offset), 0),
        (-arm_length * np.cos(angle_offset), arm_length * np.sin(angle_offset), 0),
        (-arm_length * np.cos(angle_offset), -arm_length * np.sin(angle_offset), 0)
    ]

    motors_world = []
    for mx_b, my_b, mz_b in motors_body:
        my_temp = cos_phi * my_b - sin_phi * mz_b
        mz_temp = sin_phi * my_b + cos_phi * mz_b
        mx_w = cos_theta * mx_b - sin_theta * mz_temp
        my_w = my_temp
        mz_w = sin_theta * mx_b + cos_theta * mz_temp
        motors_world.append((mx_w, my_w, mz_w))

    ax2.plot([motors_world[0][0], motors_world[3][0]], 
             [motors_world[0][1], motors_world[3][1]], 
             [motors_world[0][2], motors_world[3][2]], 'k-', linewidth=4)
    ax2.plot([motors_world[1][0], motors_world[2][0]], 
             [motors_world[1][1], motors_world[2][1]], 
             [motors_world[1][2], motors_world[2][2]], 'k-', linewidth=4)

    body_size = 0.15
    r = body_size / 2
    vertices_body = [[-r, -r, -r], [r, -r, -r], [r, r, -r], [-r, r, -r],
                     [-r, -r, r], [r, -r, r], [r, r, r], [-r, r, r]]
    
    vertices_world = []
    for vx_b, vy_b, vz_b in vertices_body:
        vy_temp = cos_phi * vy_b - sin_phi * vz_b
        vz_temp = sin_phi * vy_b + cos_phi * vz_b
        vx_w = cos_theta * vx_b - sin_theta * vz_temp
        vy_w = vy_temp
        vz_w = sin_theta * vx_b + cos_theta * vz_temp
        vertices_world.append([vx_w, vy_w, vz_w])
    
    faces = [[vertices_world[0], vertices_world[1], vertices_world[2], vertices_world[3]],
             [vertices_world[4], vertices_world[5], vertices_world[6], vertices_world[7]],
             [vertices_world[0], vertices_world[1], vertices_world[5], vertices_world[4]],
             [vertices_world[2], vertices_world[3], vertices_world[7], vertices_world[6]],
             [vertices_world[0], vertices_world[3], vertices_world[7], vertices_world[4]],
             [vertices_world[1], vertices_world[2], vertices_world[6], vertices_world[5]]]
    
    cube = Poly3DCollection(faces, alpha=0.7, facecolor='royalblue', edgecolor='navy')
    ax2.add_collection3d(cube)

    thrusts = [T1, T2, T3, T4]
    motor_colors = ['red', 'red', 'orange', 'orange']
    thrust_scale = 0.003

    for idx, ((mx, my, mz), T) in enumerate(zip(motors_world, thrusts)):
        ax2.scatter([mx], [my], [mz], color=motor_colors[idx], s=150, edgecolors='black', linewidths=2)
        
        thrust_body_z = -thrust_scale * T
        ty_temp = -sin_phi * thrust_body_z
        tz_temp = cos_phi * thrust_body_z
        thrust_x = -sin_theta * tz_temp
        thrust_y = ty_temp
        thrust_z = cos_theta * tz_temp
        
        ax2.quiver(mx, my, mz, thrust_x, thrust_y, thrust_z, color=motor_colors[idx], 
                  arrow_length_ratio=0.3, linewidth=3)
        ax2.text(mx, my, mz + 0.3, f'{T:.0f}N', fontsize=8, ha='center', color=motor_colors[idx])

    info_text = f'θ={teta:.1f}° φ={phi_deg:.1f}°\nF:{T1+T2:.0f}N R:{T3+T4:.0f}N\nL:{T1+T3:.0f}N R:{T2+T4:.0f}N'
    ax2.text2D(0.02, 0.98, info_text, transform=ax2.transAxes, fontsize=9, 
              verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_zlabel('Z [m]')
    ax2.set_title(f'Total={T1+T2+T3+T4:.0f}N', fontsize=11)

    plt.draw()
    plt.pause(0.001)

    t += dt

plt.ioff()
plt.show()
