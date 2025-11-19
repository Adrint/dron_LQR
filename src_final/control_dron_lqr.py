import numpy as np
import math
import matplotlib.pyplot as plt
from control_rk45 import aa_rk45
from control_matrices import aa_matrices_AB
from control_lqr import lqr_m
from trajectory import generate_reference_profile
from control_limits import apply_all_limits
from config import config
from map_4_animate_route import DroneLiveAnimator
from map_5_visualization import DroneVisualizer
from simulation_controller import SimulationController


def dron_lqr(path_result=None):
    # Parametry z konfiguracji
    mass = config.mass
    g = config.g
    n = config.n
    m = config.m
    t = config.t
    dt = config.dt
    avoid_distance = config.avoid_distance
    c_turb = config.c_turb
    engine_on = config.engine_on
    velocity = config.velocity
    x_turb_1 = config.x_turb_1
    x_turb_2 = config.x_turb_2

    # Ograniczenia ciągu
    full_thrust = mass * g
    nominal_thrust = full_thrust / 4
    thrust_min = 0
    thrust_max = nominal_thrust * 1.5

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

    # Inicjalizacja turbulencji
    tau_turb = 100.0  # Time constant [s] - controls frequency content
    sigma_turb = c_turb  # Turbulence intensity
    az_turbulence = 0.0  # Initial turbulence state
    az_turb_prev = 0.0  # Previous turbulence value for filtering

    # Trajektoria
    s_ref_all, y_ref_all, z_ref_all, alpha_all, beta_all, x_ref_all = generate_reference_profile(
        velocity, dt, avoid_distance)

    # Inicjalizacja kontrolera symulacji
    controller = SimulationController()
    controller.start()

    # Inicjalizacja wizualizacji
    animator = DroneLiveAnimator()
    animator.initialize(path_result=path_result)
    drone_orientation = DroneVisualizer(simulation_controller=controller)

    # Tryb silnika
    if engine_on:
        initial_thrust = nominal_thrust
    else:
        initial_thrust = 0.0

    if path_result is not None:
        x_start = path_result['X'][0]
        y_start = path_result['Y'][0]
        z_start = -path_result['Z'][0]
    else:
        x_start = x_ref_all[0]
        y_start = y_ref_all[0]
        z_start = z_ref_all[0]

    # ========================================================================
    # GROUND PROTECTION SYSTEM
    # ========================================================================
    z_absolute_ground = 0.0  # Absolute ground level (never go below)

    # Initial protection zone configuration
    protection_config = {
        'z_start': z_start,
        'z_initial_protection': z_start + 10.0,  # 10m buffer in NED coordinates
        'initial_protection_active': (z_start < -0.1)  # Active only if starting above ground
    }

    print(f"Wysokość startowa: Z = {z_start:.2f}m (NED) = {-z_start:.2f}m (AGL)")
    print(f"Poziom ziemi (bezwzględny): Z = {z_absolute_ground:.2f}m")

    if n == 12:
        x = np.array([0.0, 0.0, 0.0,  # vx, vy, vz
                      0.0, 0.0, 0.0,  # p, q, r
                      x_start, y_start, z_start,  # x, y, z - punkt A
                      0.0, 0.0, 0.0])  # phi, theta, psi
    elif n == 16:
        x = np.array([0.0, 0.0, 0.0,  # vx, vy, vz
                      0.0, 0.0, 0.0,  # p, q, r
                      x_start, y_start, z_start,  # x, y, z - punkt A
                      0.0, 0.0, 0.0,  # phi, theta, psi
                      initial_thrust, initial_thrust, initial_thrust, initial_thrust])  # T1, T2, T3, T4
    else:
        raise ValueError(f"n must be 12 or 16. Got n={n}")

    # POPRAWKA: Inicjalizuj historię z początkową pozycją
    # Dodaj początkową pozycję do historii animatora
    animator.hist_x.append(x_start)
    animator.hist_y.append(y_start)
    animator.hist_z.append(-z_start)  # Konwersja NED -> AGL

    # Oblicz początkowy dystans
    x_rot_init = animator.cos_rot * x_start - animator.sin_rot * y_start
    s_init = x_rot_init - animator.xa_rot
    animator.hist_s.append(s_init)

    # Sterowanie początkowe
    u = np.array([initial_thrust, initial_thrust, initial_thrust, initial_thrust])

    # Zmienne referencyjne
    s_ref = 0.0  # Dystans przebity
    phi_ref = 0.0
    theta_ref = 0.0
    psi_ref = 0.0

    # Parametry pętli
    max_iterations = 1000000
    print_interval = 100  # Co ile iteracji wyświetlać status

    # ========================================================================
    # GŁÓWNA PĘTLA SYMULACJI
    # ========================================================================

    print("\n" + "=" * 80)
    print("Rozpoczęcie symulacji")
    print("=" * 80)

    for i in range(max_iterations):

        # Obsługa pauzy i zatrzymania
        # POPRAWKA: Użyj plt.pause() zamiast time.sleep() aby matplotlib mógł obsługiwać zdarzenia
        while controller.is_paused() and not controller.is_stopped():
            plt.pause(0.1)  # Pozwala matplotlib przetwarzać zdarzenia GUI

        if controller.is_stopped():
            print(f"\n Symulacja zatrzymana przez użytkownika (t={t:.2f}s)")
            break

        # 1. Punkt referencyjny
        idx_ref = np.argmin(np.abs(s_ref_all - s_ref))
        alfa = alpha_all[idx_ref]
        beta = beta_all[idx_ref]
        y_ref = y_ref_all[idx_ref]
        z_ref = z_ref_all[idx_ref]
        x_ref = x_ref_all[idx_ref]  # Współrzędna X dla regulatora

        # Przyrost dystansu
        ds = velocity * dt
        s_ref += ds

        # 2. Prędkości referencyjne 3D
        vx_ref = velocity * np.cos(alfa) * np.cos(beta)
        vy_ref = velocity * np.cos(alfa) * np.sin(beta)
        vz_ref = velocity * np.sin(alfa)

        # 3. REGULATOR LQR
        R = np.eye(m) * R_weight
        Q = np.eye(n)
        e = np.zeros(n)

        # Wagi
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

            # Linearyzacja i LQR
            A, B = aa_matrices_AB("rhs", x, t, u, az_turbulence)
            K, P = lqr_m(A, B, Q, R)

            # Sterowanie
            u_pert = -K @ e
            u_desired = nominal_thrust + u_pert
            u = np.clip(u_desired, thrust_min, thrust_max)

            # 5. Turbulencje
            if x_turb_1 < s_ref < x_turb_2:
                az_turbulence = c_turb * (1.0 - 2.0 * np.random.rand())

            # 4. Integracja i limity
            x = aa_rk45("rhs", x, t, dt, u, az_turbulence)

            # Apply limits with ground protection
            x = apply_all_limits(x, z_ground=z_absolute_ground, protection_config=protection_config)

            # 5. Aktualizacja wizualizacji
            animator.update(x, t, i)
            drone_orientation.update_plots(x, t, i, u[0], u[1], u[2], u[3])

            # 6. Wyświetlanie statusu
            if i % print_interval == 0 and i > 0:
                print(f"t={t:6.2f}s | Pozycja: X={x[6]:7.2f} Y={x[7]:7.2f} Z={x[8]:7.2f} | "
                      f"AGL={-x[8]:6.2f}m | s={s_ref:7.2f}m")

            t += dt

            # 7. WARUNEK KOŃCA - ostatni punkt
            x_goal = x_ref_all[-1]
            y_goal = y_ref_all[-1]
            z_goal = z_ref_all[-1]

            # Błędy pozycji
            dx = x[6] - x_goal
            dy = x[7] - y_goal
            dz = x[8] - z_goal

            horiz_err = math.hypot(dx, dy)
            vert_err = abs(dz)

            # Progi dokładności
            HORIZ_TOL = 0.5  # [m]
            VERT_TOL = 0.5  # [m]

            if horiz_err < HORIZ_TOL and vert_err < VERT_TOL:
                print(f"\n Osiągnięto cel! Czas: {t:.2f}s")
                print(f"Błąd poziomy: {horiz_err:.3f}m")
                print(f"Błąd pionowy: {vert_err:.3f}m")
                break

            # Timeout
            if t > 300:  # 5 minut max
                print(f"\n Timeout - przekroczono maksymalny czas symulacji")
                break

    # 9. Zakończenie wizualizacji
    animator.close()
    drone_orientation.close()
    controller.close()

    print("\n" + "=" * 80)
    print("Koniec symulacji")
    print("=" * 80)
    print(f"Całkowity czas: {t:.2f}s")
    print(f"Liczba kroków: {i}")