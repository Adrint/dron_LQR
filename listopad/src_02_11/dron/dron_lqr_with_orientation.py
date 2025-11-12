import numpy as np
import math
from rk45 import aa_rk45
from matrices import aa_matrices_AB
from lqr import lqr_m
from trajectory import generate_reference_profile
from limits import apply_all_limits, check_limits_exceeded
from config import config
from visualization import DroneVisualizer


def dron_lqr():

    # ========================================================================
    # Parametry
    # ========================================================================

    engine_on = config.engine_on
    nominal_thrust = config.nominal_thrust
    velocity = config.velocity
    n = config.n
    m = config.m
    motor_arm_x = config.motor_arm_x
    motor_arm_y = config.motor_arm_y
    t = config.t
    dt = config.dt
    avoid_distance = config.avoid_distance
    c_turb = config.c_turb
    az_turbulence = 0.0

    # LqR weights
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

    thrust_min = config.thrust_min
    thrust_max = config.thrust_max
    x_turb_1 = config.x_turb_1
    x_turb_2 = config.x_turb_2

    # ========================================================================
    # Inicjalizacja stanu
    # ========================================================================

    z_initial = -config.h_start  # NED: z = -altitude
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
    # Trajektoria referencyjna
    # ========================================================================

    x_ref_all, y_terr_all, z_terr_all, y_ref_all, z_ref_all, alpha_all, beta_all = generate_reference_profile(velocity, dt, avoid_distance)

    # ========================================================================
    # Inicjalizacja wizualizacji
    # ========================================================================

    visualizer = DroneVisualizer(
        x_ref_all, y_terr_all, z_terr_all, y_ref_all, z_ref_all,
        motor_arm_x, motor_arm_y
    )

    # ========================================================================
    # Zmienne referencyjne
    # ========================================================================

    phi_ref = 0.0
    theta_ref = 0.0
    psi_ref = 0.0
    x_ref = 0.0

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

    for i in range(10000):
        # Exit condition
        if t >= 50.0 or x[6] >= 50:
            break

        x_forward = x[6]  # Obecna pozycja

        # ====================================================================
        # 1. Punkt referencyjny
        # ====================================================================

        idx_ref = np.argmin(np.abs(x_ref_all - x_ref))
        y_ref = y_ref_all[idx_ref]
        z_ref = z_ref_all[idx_ref]
        alfa = alpha_all[idx_ref]
        beta = beta_all[idx_ref]

        # Aktualizacja pozycji referencyjnej
        x_ref += velocity * np.cos(alfa) * dt

        # ====================================================================
        # 2. Prędkości referencyjne
        # ====================================================================

        vx_ref = velocity * np.cos(alfa)
        vz_ref = velocity * np.sin(alfa)
        vy_ref = velocity * np.sin(beta) * np.cos(alfa)


        # ====================================================================
        # 4. Zbieranie danych
        # ====================================================================

        tp.append(t)
        yp.append(x.copy())
        up.append(u.copy())

        # ====================================================================
        # 5. REGULATOR LqR
        # ====================================================================

        R = np.eye(m) * 1.0
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
            Q[12, 12] = 0.1  # Thrust_1
            Q[13, 13] = 0.1  # Thrust_2
            Q[14, 14] = 0.1  # Thrust_3
            Q[15, 15] = 0.1  # Thrust_4
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

        # Obliczenie ciągów z ograniczeniami
        T1_desired = nominal_thrust + u_pert[0]
        T2_desired = nominal_thrust + u_pert[1]
        T3_desired = nominal_thrust + u_pert[2]
        T4_desired = nominal_thrust + u_pert[3]

        T1 = np.clip(T1_desired, thrust_min, thrust_max)
        T2 = np.clip(T2_desired, thrust_min, thrust_max)
        T3 = np.clip(T3_desired, thrust_min, thrust_max)
        T4 = np.clip(T4_desired, thrust_min, thrust_max)

        u = np.array([T1, T2, T3, T4])

        # ====================================================================
        # 6. Efekty środowiskowe (turbulencje)
        # ====================================================================

        if x_turb_1 < x_forward < x_turb_2:
            az_turbulence = c_turb * (1.0 - 2.0 * np.random.rand())

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
        # 10. Wizualizacja
        # ====================================================================

        yp_array = np.array(yp)
        visualizer.update_plots(x, t, i, yp_array, T1, T2, T3, T4, limits_exceeded)

        # ====================================================================
        # 11. Debugowanie (OPCJONALNE)
        # ====================================================================

        if x[6] < 2:
            print("=" * 80)
            print(f't = {t:.3f}s')
            print(f'pozycja: x = {x[6]:.2f}m, y = {x[7]:.2f}m, z_NED = {x[8]:.2f}m, AGL = {-x[8]:.2f}m')
            print(f'pozycja: x_ref = {x_ref:.2f}m, y_ref = {y_ref:.2f}m, z_ref = {z_ref:.2f}m, AGL_ref = {-z_ref:.2f}m')
            print(f'prędkość vx = {x[0]:.2f}m/s, vy = {x[1]:.2f}m/s, vz = {x[2]:.2f}m/s')
            print(
                f'kąty: phi = {math.degrees(x[9]):.2f}°, theta = {math.degrees(x[10]):.2f}°, psi = {math.degrees(x[11]):.2f}°')
            print(
                f'prędkości kątowe: p = {math.degrees(x[3]):.2f}°/s, q = {math.degrees(x[4]):.2f}°/s, r = {math.degrees(x[5]):.2f}°/s')
            print()

        t = t + dt

    # ========================================================================
    # Podsumowanie
    # ========================================================================

    print("\n" + "=" * 80)
    print("Koniec symulacji")
    print("=" * 80)
    print(f"Całkowity czas symulacji: {t:.2f}s")
    print(f"Liczba przekroczeń limitów: {limits_exceeded_count}")

    visualizer.close()
