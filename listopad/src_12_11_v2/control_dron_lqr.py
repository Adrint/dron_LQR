import numpy as np
import math
from control_rk45 import aa_rk45
from control_matrices import aa_matrices_AB
from control_lqr import lqr_m
from trajectory import generate_reference_profile
from control_limits import apply_all_limits, check_limits_exceeded
from config import config
from visualization import DroneVisualizer
from map_3_path_planner import DroneRoutePlanner
from map_4_animate_route import PlannerStyleAnimator

def debug_print(x, t, x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref, verbose=False):
    if x[6] < 5:
        print("=" * 80)
        print(f't = {t:.3f}s')
        print(f'pozycja: x = {x[6]:.2f}m, y = {x[7]:.2f}m, z_NED = {x[8]:.2f}m, AGL = {-x[8]:.2f}m')
        print(
            f'pozycja ref: x_ref = {x_ref:.2f}m, y_ref = {y_ref:.2f}m, z_ref = {z_ref:.2f}m, AGL_ref = {-z_ref:.2f}m')
        print(f'prędkość: vx = {x[0]:.2f}m/s, vy = {x[1]:.2f}m/s, vz = {x[2]:.2f}m/s')
        print(f'prędkość ref: vx_ref = {vx_ref:.2f}m/s, vy_ref = {vy_ref:.2f}m/s, vz_ref = {vz_ref:.2f}m/s')
        print()

def dron_lqr():
    # ========================================================================
    # Parametry
    # ========================================================================

    mass = config.mass
    g = config.g
    engine_on = config.engine_on
    full_thrust = mass * g
    nominal_thrust = full_thrust / 4
    thrust_min = 0
    thrust_max = nominal_thrust * 1.5
    velocity = config.velocity
    n = config.n
    m = config.m
    t = config.t
    dt = config.dt
    avoid_distance = config.avoid_distance
    c_turb = config.c_turb

    # LQR weights
    Q_velocity_x = config.Q_velocity_x
    Q_velocity_y = config.Q_velocity_y
    Q_velocity_z = config.Q_velocity_z
    Q_angular_rate_p = config.Q_angular_rate_p
    Q_angular_rate_Q = config.Q_angular_rate_q
    Q_angular_rate_r = config.Q_angular_rate_r
    Q_position_x = config.Q_position_X
    Q_position_y = config.Q_position_Y
    Q_position_z = config.Q_position_Z
    Q_phi = config.Q_phi
    Q_theta = config.Q_theta
    Q_psi = config.Q_psi
    Q_thrust = config.Q_thrust
    R_weight = config.R_weight



    x_turb_1 = config.x_turb_1
    x_turb_2 = config.x_turb_2

    # ========================================================================
    # Inicjalizacja turbulencji Dryden
    # ========================================================================

    # Dryden turbulence model parameters
    tau_turb = 100.0  # Time constant [s] - controls frequency content
    sigma_turb = c_turb  # Turbulence intensity
    az_turbulence = 0.0  # Initial turbulence state
    az_turb_prev = 0.0  # Previous turbulence value for filtering

    z_initial = -config.altitude_start  # NED: z = -altitude
    y_initial = 0.0

    if engine_on:
        initial_thrust = nominal_thrust
    else:
        initial_thrust = 0.0

    if n == 12:
        # 3D State: [vx, vy, vz, p, q, r, x, y, z, phi, theta, psi]
        x = np.array([0.0, 0.0, 0.0,  # vx, vy, vz
                      0.0, 0.0, 0.0,  # p, q, r
                      0.0, y_initial, z_initial,  # x, y, z
                      0.0, 0.0, 0.0])  # phi, theta, psi
    elif n == 16:
        # 3D State with thrust: [vx, vy, vz, p, q, r, x, y, z, phi, theta, psi, T1, T2, T3, T4]
        x = np.array([0.0, 0.0, 0.0,  # vx, vy, vz
                      0.0, 0.0, 0.0,  # p, q, r
                      0.0, y_initial, z_initial,  # x, y, z
                      0.0, 0.0, 0.0,  # phi, theta, psi
                      initial_thrust, initial_thrust, initial_thrust, initial_thrust])  # T1, T2, T3, T4
    else:
        raise ValueError(f"n must be 12 or 16. Got n={n}")

    # Control input
    u = np.array([initial_thrust, initial_thrust, initial_thrust, initial_thrust])


    # ========================================================================
    # Trajektoria oraz inicjalizacja wizualizacji
    # ========================================================================

    x_ref_all, y_terr_all, z_terr_all, y_ref_all, z_ref_all, alpha_all, beta_all = generate_reference_profile(velocity,dt,avoid_distance)
    visualizer = DroneVisualizer(x_ref_all, y_terr_all, z_terr_all, y_ref_all, z_ref_all)

    planner = DroneRoutePlanner(config)
    res = planner.plan(point_a=None, point_b=None, show_plots=False)
    path_latlon = res["path_latlon"]
    buildings_gdf = planner.buildings_gdf
    point_a = (path_latlon[0][0], path_latlon[0][1])
    point_b = (path_latlon[-1][0], path_latlon[-1][1])

    latlon_to_meters_fn = DroneRoutePlanner.latlon_to_meters
    estimate_height_fn = DroneRoutePlanner._estimate_height

    anim = PlannerStyleAnimator(
        path_latlon=path_latlon,
        point_a=point_a,
        point_b=point_b,
        buildings_gdf=buildings_gdf,
        latlon_to_meters_fn=latlon_to_meters_fn,
        estimate_height_fn=estimate_height_fn,
        sim_ref_xy=(x_ref_all, y_ref_all)  # <= KLUCZOWE: referencja z symulacji
    )

    # ========================================================================
    # Zmienne referencyjne
    # ========================================================================

    phi_ref = 0.0
    theta_ref = 0.0
    psi_ref = 0.0

    x_ref = 0.0
    y_ref = 0.0
    z_ref = 0.0

    limits_exceeded_count = 0

    # Storage for plotting
    tp = []
    yp = []
    up = []

    # ========================================================================
    # GŁÓWNA PĘTLA SYMULACJI
    # ========================================================================

    print("\n" + "=" * 80)
    print("Rozpoczęcie symulacji")
    print("=" * 80)

    for i in range(1000):
        # Exit condition
        if t >= 10000.0:
            break

        x_forward = x[6]  # Obecna pozycja

        # ====================================================================
        # 1. Punkt referencyjny
        # ====================================================================

        idx_ref = np.argmin(np.abs(x_ref_all - x_ref))
        alfa = alpha_all[idx_ref]
        beta = beta_all[idx_ref]
        y_ref = y_ref_all[idx_ref]
        z_ref = z_ref_all[idx_ref]
        x_ref += velocity * np.cos(alfa) * np.cos(beta) * dt


        # ====================================================================
        # 2. Prędkości referencyjne 3D (POPRAWIONE!)
        # ====================================================================

        vx_ref = velocity * np.cos(alfa) * np.cos(beta)  # Forward component
        vy_ref = velocity * np.cos(alfa) * np.sin(beta)
        vz_ref = velocity * np.sin(alfa)

        # Verify magnitude (should equal velocity)
        # v_magnitude = sqrt(vx_ref^2 + vy_ref^2 + vz_ref^2) ≈ velocity

        # ====================================================================
        # 3. Zbieranie danych
        # ====================================================================

        tp.append(t)
        yp.append(x.copy())
        up.append(u.copy())

        # ====================================================================
        # 4. REGULATOR LQR
        # ====================================================================

        R = np.eye(m) * R_weight
        Q = np.eye(n)
        e = np.zeros(n)

        # Wagi 3D
        Q[0, 0] = Q_velocity_x  # vx
        Q[1, 1] = Q_velocity_y  # vy
        Q[2, 2] = Q_velocity_z  # vz
        Q[3, 3] = Q_angular_rate_p  # p (roll rate)
        Q[4, 4] = Q_angular_rate_Q  # Q (pitch rate)
        Q[5, 5] = Q_angular_rate_r  # r (yaw rate)
        Q[6, 6] = Q_position_x  # x
        Q[7, 7] = Q_position_y  # y
        Q[8, 8] = Q_position_z  # z
        Q[9, 9] = Q_phi  # phi (roll)
        Q[10, 10] = Q_theta  # theta (pitch)
        Q[11, 11] = Q_psi  # psi (yaw)

        if n == 16:
            Q[12, 12] = Q_thrust  # Thrust_1
            Q[13, 13] = Q_thrust  # Thrust_2
            Q[14, 14] = Q_thrust  # Thrust_3
            Q[15, 15] = Q_thrust  # Thrust_4
            Q = 10 * Q

        # Wektor błędów
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
            e[12] = x[12] - nominal_thrust
            e[13] = x[13] - nominal_thrust
            e[14] = x[14] - nominal_thrust
            e[15] = x[15] - nominal_thrust

        # Obliczenie macierzy A, B i wzmocnienia K
        A, B = aa_matrices_AB("rhs", x, t, u, az_turbulence)
        K, P = lqr_m(A, B, Q, R)

        u_pert = -K @ e

        # ====================================================================
        # 5. ANTI-WINDUP: Back-calculation method
        # ====================================================================

        # Desired thrust (before saturation)
        T1_desired = nominal_thrust + u_pert[0]
        T2_desired = nominal_thrust + u_pert[1]
        T3_desired = nominal_thrust + u_pert[2]
        T4_desired = nominal_thrust + u_pert[3]

        # Apply saturation
        T1 = np.clip(T1_desired, thrust_min, thrust_max)
        T2 = np.clip(T2_desired, thrust_min, thrust_max)
        T3 = np.clip(T3_desired, thrust_min, thrust_max)
        T4 = np.clip(T4_desired, thrust_min, thrust_max)

        u = np.array([T1, T2, T3, T4])

        # ====================================================================
        # 6. Efekty środowiskowe
        # ====================================================================

        if x_turb_1 < x_forward < x_turb_2:
            # Dryden turbulence: first-order colored noise filter
            # daz/dt = -az/tau + sigma*white_noise/tau
            # This creates realistic, temporally-correlated turbulence

            white_noise = np.random.randn()  # Standard normal distribution
            daz_dt = -az_turb_prev / tau_turb + (sigma_turb / np.sqrt(tau_turb)) * white_noise
            az_turbulence = az_turb_prev + daz_dt * dt
            az_turb_prev = az_turbulence
        else:
            # Outside turbulence zone - decay exponentially
            az_turbulence = az_turb_prev * np.exp(-dt / tau_turb)
            az_turb_prev = az_turbulence

        # ====================================================================
        # 7. Integracja numeryczna
        # ====================================================================

        x = aa_rk45("rhs", x, t, dt, u, az_turbulence)

        # ====================================================================
        # 8. Limity
        # ====================================================================

        x_before_limit = x.copy()
        x = apply_all_limits(x)
        limits_exceeded, violations = check_limits_exceeded(x_before_limit, verbose=False)

        # ====================================================================
        # 9. Wizualizacja
        # ====================================================================

        yp_array = np.array(yp)
        visualizer.update_plots(x, t, i, yp_array, T1, T2, T3, T4, limits_exceeded)
        anim.update_drone(yp_array)

        # ====================================================================
        # 10. Debugowanie (OPCJONALNE)
        # ====================================================================
        #debug_print(x, t, x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref)

        t = t + dt

    visualizer.close()
    anim.close()

    print("\n" + "=" * 80)
    print("Koniec symulacji")
    print("=" * 80)
    print(f"Całkowity czas symulacji: {t:.2f}s")

