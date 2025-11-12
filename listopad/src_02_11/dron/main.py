import numpy as np
import sys
import os
from dron_lqr_with_orientation import dron_lqr
from config import config

def drone_parameters():
    print("\nPARAMETRY FIZYCZNE DRONA")
    # Masa drona
    mass_input = input("Masa drona [kg] [8.0]: ").strip()
    mass = float(mass_input) if mass_input else 8.0

    # Masa silnika
    mass_engine_input = input("Masa pojedynczego rotora [kg] [4.0]: ").strip()
    mass_engine = float(mass_engine_input) if mass_engine_input else 4.0

    # Długość ramienia X
    motor_arm_x_input = input("Długość ramienia X (od środka do rotoru) [m] [0.6]: ").strip()
    motor_arm_x = float(motor_arm_x_input) if motor_arm_x_input else 0.6

    # Długość ramienia Y
    motor_arm_y_input = input("Długość ramienia Y (od środka do rotoru) [m] [0.6]: ").strip()
    motor_arm_y = float(motor_arm_y_input) if motor_arm_y_input else 0.6

    # Powierzchnia czołowa
    size_input = input("Powierzchnia czołowa drona [m²] [0.4]: ").strip()
    size = float(size_input) if size_input else 0.4

    # Współczynnik oporu
    cd_input = input("Współczynnik oporu CD_0 [0.30]: ").strip()
    CD_0 = float(cd_input) if cd_input else 0.30

    # Współczynnik momentu śmigła
    k_torque_input = input("Współczynnik momentu śmigła k_torQue [0.015]: ").strip()
    k_torque = float(k_torque_input) if k_torque_input else 0.015

    # Stała czasowa silnika
    tau_input = input("Stała czasowa silnika TAU [s] [0.05]: ").strip()
    tau = float(tau_input) if tau_input else 0.05

    # Współczynniki tłumienia
    print("\n--- Współczynniki tłumienia ---")
    C_Lp_input = input("Współczynnik tłumienia roll C_Lp [-0.01]: ").strip()
    C_Lp = float(C_Lp_input) if C_Lp_input else -0.01

    CM_Q_input = input("Współczynnik tłumienia pitch CM_Q [-0.01]: ").strip()
    CM_Q = float(CM_Q_input) if CM_Q_input else -0.01

    C_Nr_input = input("Współczynnik tłumienia yaw C_Nr [-0.01]: ").strip()
    C_Nr = float(C_Nr_input) if C_Nr_input else -0.01

    return mass, mass_engine, motor_arm_x, motor_arm_y, size, CD_0, k_torque, tau, C_Lp, CM_Q, C_Nr

def limits():
    print("\nLIMITY FIZYCZNE DRONA")
    print("\n--- Limity ciągu [N] ---")
    thrust_max_input = input("Maksymalny ciąg na silnik [N] [130.0]: ").strip()
    thrust_max = float(thrust_max_input) if thrust_max_input else 130.0

    thrust_min_input = input("Minimalny ciąg na silnik [N] [0.0]: ").strip()
    thrust_min = float(thrust_min_input) if thrust_min_input else 0.0

    # Limity prędkości kątowych
    print("\n--- Limity prędkości kątowych [°/s] ---")
    max_pitch_rate_input = input("Maksymalna prędkość pitch [°/s] [300]: ").strip()
    max_pitch_rate_deg = float(max_pitch_rate_input) if max_pitch_rate_input else 300.0
    max_pitch_rate_rad = np.deg2rad(max_pitch_rate_deg)

    max_roll_rate_input = input("Maksymalna prędkość roll [°/s] [300]: ").strip()
    max_roll_rate_deg = float(max_roll_rate_input) if max_roll_rate_input else 300.0
    max_roll_rate_rad = np.deg2rad(max_roll_rate_deg)

    max_yaw_rate_input = input("Maksymalna prędkość yaw [°/s] [100]: ").strip()
    max_yaw_rate_deg = float(max_yaw_rate_input) if max_yaw_rate_input else 100.0
    max_yaw_rate_rad = np.deg2rad(max_yaw_rate_deg)

    # Limity kątów
    print("\n--- Limity kątów [°] ---")
    max_theta_angle_input = input("Maksymalny kąt  (theta) [°] [25]: ").strip()
    max_theta_angle_deg = float(max_theta_angle_input) if max_theta_angle_input else 25.0
    max_theta_angle_rad = np.deg2rad(max_theta_angle_deg)

    max_phi_angle_input = input("Maksymalny kąt  (phi) [°] [25]: ").strip()
    max_phi_angle_deg = float(max_phi_angle_input) if max_phi_angle_input else 25.0
    max_phi_angle_rad = np.deg2rad(max_phi_angle_deg)

    max_psi_angle_input = input("Maksymalny kąt  (psi) [°] [180]: ").strip()
    max_psi_angle_deg = float(max_psi_angle_input) if max_psi_angle_input else 180.0
    max_psi_angle_rad = np.deg2rad(max_psi_angle_deg)


    # Limity prędkości liniowych
    print("\n--- Limity prędkości liniowych [m/s] ---")
    max_horizontal_vel_input = input("Maksymalna prędkość pozioma [m/s] [23.0]: ").strip()
    max_horizontal_vel = float(max_horizontal_vel_input) if max_horizontal_vel_input else 23.0

    max_ascent_input = input("Maksymalna prędkość wznoszenia [m/s] [6.0]: ").strip()
    max_ascent = float(max_ascent_input) if max_ascent_input else 6.0

    max_descent_input = input("Maksymalna prędkość opadania [m/s] [5.0]: ").strip()
    max_descent = float(max_descent_input) if max_descent_input else 5.0

    max_descent_tilted_input = input("Maksymalna prędkość opadania (z przechyłem) [m/s] [7.0]: ").strip()
    max_descent_tilted = float(max_descent_tilted_input) if max_descent_tilted_input else 7.0

    return thrust_max, thrust_min, max_pitch_rate_rad, max_roll_rate_rad, max_yaw_rate_rad, max_theta_angle_rad, max_phi_angle_rad, max_psi_angle_rad, max_horizontal_vel, max_ascent, max_descent, max_descent_tilted

def control_parameters():
    print("\nPARAMETRY STEROWANIA")

    # Wagi LQR - Q (stan)
    print("\n--- Wagi LQR dla stanu (Q) ---")
    Q_velocity_x_input = input("Waga dla prędkości (vx) [50.0]: ").strip()
    Q_velocity_x = float(Q_velocity_x_input) if Q_velocity_x_input else 50.0

    Q_velocity_y_input = input("Waga dla prędkości (vy) [50.0]: ").strip()
    Q_velocity_y = float(Q_velocity_y_input) if Q_velocity_y_input else 50.0

    Q_velocity_z_input = input("Waga dla prędkości (vz) [50.0]: ").strip()
    Q_velocity_z = float(Q_velocity_z_input) if Q_velocity_z_input else 50.0

    Q_angular_rate_p_input = input("Waga dla prędkości kątowej (p) [0.1]: ").strip()
    Q_angular_rate_p = float(Q_angular_rate_p_input) if Q_angular_rate_p_input else 0.1

    Q_angular_rate_q_input = input("Waga dla prędkości kątowej (q) [0.1]: ").strip()
    Q_angular_rate_q = float(Q_angular_rate_q_input) if Q_angular_rate_q_input else 0.1

    Q_angular_rate_r_input = input("Waga dla prędkości kątowej (r) [0.1]: ").strip()
    Q_angular_rate_r = float(Q_angular_rate_r_input) if Q_angular_rate_r_input else 0.1

    Q_position_X_input = input("Waga dla pozycji (X) [250.0]: ").strip()
    Q_position_X = float(Q_position_X_input) if Q_position_X_input else 250.0

    Q_position_Y_input = input("Waga dla pozycji (Y) [250.0]: ").strip()
    Q_position_Y = float(Q_position_Y_input) if Q_position_Y_input else 250.0

    Q_position_Z_input = input("Waga dla pozycji (Z) [250.0]: ").strip()
    Q_position_Z = float(Q_position_Z_input) if Q_position_Z_input else 2050.0

    Q_phi_input = input("Waga dla kątów (phi) [1.0]: ").strip()
    Q_phi = float(Q_phi_input) if Q_phi_input else 1.0

    Q_theta_input = input("Waga dla kątów (theta) [1.0]: ").strip()
    Q_theta = float(Q_theta_input) if Q_theta_input else 1.0

    Q_psi_input = input("Waga dla kątów (psi) [1.0]: ").strip()
    Q_psi = float(Q_psi_input) if Q_psi_input else 1.0

    Q_thrust_input = input("Waga dla ciągów (T1-T4, tylko dla n=16) [0.1]: ").strip()
    Q_thrust = float(Q_thrust_input) if Q_thrust_input else 0.1

    # Waga R (sterowanie)
    r_input = input("Waga dla sterowania (R) [1.0]: ").strip()
    r = float(r_input) if r_input else 1.0

    return Q_velocity_x, Q_velocity_y, Q_velocity_z, Q_angular_rate_p, Q_angular_rate_q, Q_angular_rate_r, Q_position_X, Q_position_Y, Q_position_Z, Q_phi, Q_theta, Q_psi, Q_thrust, r

def flight_parameters():

    print("\nPARAMETRY LOTU")
    g_input = input(f"Przyspieszenie ziemskie [9.81]:")
    g = float(g_input) if g_input else 9.81

    RO_0_input = input(f"Gęstość powietrza [1.225 kg/m^3]")
    RO_0 = float(RO_0_input) if RO_0_input else 1.225  # Air density at sea level [kg/m^3]

    # Prędkość referencyjna
    velocity_input = input("Prędkość referencyjna drona [m/s] [2.0]: ").strip()
    velocity = float(velocity_input) if velocity_input else 2.0

    # Wysokości
    print("\n--- Wysokości lotu ---")
    h_start_input = input("Wysokość startowa [m] [2.0]: ").strip()
    h_start = float(h_start_input) if h_start_input else 0.0

    h_cruise_input = input("Wysokość przelotowa [m] [15.0]: ").strip()
    h_cruise = float(h_cruise_input) if h_cruise_input else 15.0

    h_end_input = input("Wysokość lądowania [m] [5.0]: ").strip()
    h_end = float(h_end_input) if h_end_input else 5.0

    avoid_distance_input = input("Podaj bezpieczną odległość [m] [2.0]: ").strip()
    avoid_distance = float(avoid_distance_input) if avoid_distance_input else 2.0

    # Turbulencje
    print("\n--- Turbulencje i wiatr ---")
    # az_turb_input = input("Przyspieszenie turbulencji pionowej [m/s²] [0.0]: ").strip()
    # az_turbulence = float(az_turb_input) if az_turb_input else 0.0

    ax_wind_input = input("Przyspieszenie wiatru poziomego [m/s²] [0.0]: ").strip()
    ax_wind = float(ax_wind_input) if ax_wind_input else 0.0

    ay_wind_input = input("Przyspieszenie wiatru poprzecznego [m/s²] [0.0]: ").strip()
    ay_wind = float(ay_wind_input) if ay_wind_input else 0.0

    az_wind_input = input("Przyspieszenie wiatru pionowego [m/s²] [0.0]: ").strip()
    az_wind = float(az_wind_input) if az_wind_input else 0.0

    # Strefa turbulencji
    c_turb_input = input("Siła turbulencji c_turb [1000.0]: ").strip()
    c_turb = float(c_turb_input) if c_turb_input else 1000.0

    x_turb1_input = input("Początek strefy turbulencji [m] [1500.0]: ").strip()
    x_turb_1 = float(x_turb1_input) if x_turb1_input else 1500.0

    x_turb2_input = input("Koniec strefy turbulencji [m] [2000.0]: ").strip()
    x_turb_2 = float(x_turb2_input) if x_turb2_input else 2000.0

    # Silniki na starcie
    engine_choice = input(f"Start na wysokości {h_start}m. Silniki włączone? (t/n) [n]: ").strip().lower()
    engine_on = True if engine_choice in ['t', 'tak', 'y', 'yes'] else False

    return g, RO_0, velocity, h_start, h_cruise, h_end, avoid_distance, ax_wind, ay_wind, az_wind, c_turb, x_turb_1, x_turb_2, engine_on

def path_analise_parameters():
    print("\nPARAMETRY ANALIZY TRAJEKTORII")

    # Dynamika silników
    print("\nCzy uwzględnić dynamikę silników?")
    print("  t/tak - model z dynamiką (n=16, dokładniejszy)")
    print("  n/nie - model prosty (n=12, szybszy)")

    thrust_dyn_input = input("Wybór [t]: ").strip().lower()
    if thrust_dyn_input in ['n', 'no', 'nie']:
        n = 12
        include_thrust_dynamics = False
    else:
        n = 16
        include_thrust_dynamics = True

    m = 4  # 4 silniki

    # Czas początkowy
    t_input = input(f"Podaj czas początkowy t [s] [0.0]: ").strip()
    t = float(t_input) if t_input else 0.0

    # Krok czasowy
    dt_input = input("Krok czasowy dt [s] [0.075]: ").strip()
    dt = float(dt_input) if dt_input else 0.075

    MAX_PATH_ITERATIONS_input = input("Maksymalna ilość iteracji [50]: ").strip()
    MAX_PATH_ITERATIONS = int(MAX_PATH_ITERATIONS_input) if MAX_PATH_ITERATIONS_input else 50

    DETOUR_SEARCH_ANGLES_input = input("Liczba kierunków do szukania objazdu [8]: ").strip()
    DETOUR_SEARCH_ANGLES = int(DETOUR_SEARCH_ANGLES_input) if DETOUR_SEARCH_ANGLES_input else 8

    DETOUR_DISTANCE_MULTIPLIER_input = input("Mnożnik dystansu objazdu [1.5]: ").strip()
    DETOUR_DISTANCE_MULTIPLIER = float(DETOUR_DISTANCE_MULTIPLIER_input) if DETOUR_DISTANCE_MULTIPLIER_input else 1.5

    PATH_SMOOTHING_POINTS_input = input("Liczba punktów do wygładzania trajektorii [100]: ").strip()
    PATH_SMOOTHING_POINTS = int(PATH_SMOOTHING_POINTS_input) if PATH_SMOOTHING_POINTS_input else 100

    COLLISION_CHECK_SAMPLES_input = input("Liczba próbek do sprawdzania kolizji na segment [50]: ").strip()
    COLLISION_CHECK_SAMPLES = int(COLLISION_CHECK_SAMPLES_input) if COLLISION_CHECK_SAMPLES_input else 50

    return n, include_thrust_dynamics, m, t, dt, MAX_PATH_ITERATIONS, DETOUR_SEARCH_ANGLES, DETOUR_DISTANCE_MULTIPLIER, PATH_SMOOTHING_POINTS, COLLISION_CHECK_SAMPLES

def main():
    print("Wciśnij Enter aby użyć wartości domyślnej\n")
    # Parametry drona
    mass, mass_engine, motor_arm_x, motor_arm_y, size, CD_0, k_torque, tau, C_Lp, CM_Q, C_Nr = drone_parameters()

    # Limity drona
    thrust_max, thrust_min, max_pitch_rate, max_roll_rate, max_yaw_rate, max_theta_angle_rad, max_phi_angle_rad, max_psi_angle_rad, max_horizontal_vel, max_ascent, max_descent, max_descent_tilted = limits()

    # Parametry sterowania
    Q_velocity_x, Q_velocity_y, Q_velocity_z, Q_angular_rate_p, Q_angular_rate_q, Q_angular_rate_r, Q_position_X, Q_position_Y, Q_position_Z, Q_phi, Q_theta, Q_psi, Q_thrust, r = control_parameters()

    # Parametry lotu
    g, RO_0, velocity, h_start, h_cruise, h_end, avoid_distance, ax_wind, ay_wind, az_wind, c_turb, x_turb_1, x_turb_2, engine_on = flight_parameters()

    # Parametry analizy trajektorii
    n, include_thrust_dynamics, m, t, dt, MAX_PATH_ITERATIONS, DETOUR_SEARCH_ANGLES, DETOUR_DISTANCE_MULTIPLIER, PATH_SMOOTHING_POINTS, COLLISION_CHECK_SAMPLES = path_analise_parameters()

    config.set_params(
        # === PARAMETRY FIZYCZNE DRONA ===
        mass=mass,
        mass_engine=mass_engine,
        motor_arm_x=motor_arm_x,
        motor_arm_y=motor_arm_y,
        size=size,
        CD_0=CD_0,
        k_torque=k_torque,
        tau=tau,
        C_Lp=C_Lp,
        CM_Q=CM_Q,
        C_Nr=C_Nr,

        # === PARAMETRY LOTU ===
        g=g,
        RO_0=RO_0,
        velocity=velocity,
        h_start=h_start,
        h_cruise=h_cruise,
        h_end=h_end,
        avoid_distance=avoid_distance,
        #az_turbulence=az_turbulence,
        ax_wind=ax_wind,
        ay_wind=ay_wind,
        az_wind=az_wind,
        c_turb=c_turb,
        x_turb_1=x_turb_1,
        x_turb_2=x_turb_2,
        engine_on=engine_on,

        # === LIMITY CIĄGU ===
        thrust_max=thrust_max,
        thrust_min=thrust_min,

        # === LIMITY PRĘDKOŚCI KĄTOWYCH ===
        max_pitch_rate_rad=max_pitch_rate,
        max_roll_rate_rad=max_roll_rate,
        max_yaw_rate_rad=max_yaw_rate,

        # === LIMITY KĄTÓW ===
        max_theta_angle_rad=max_theta_angle_rad,
        max_phi_angle_rad=max_phi_angle_rad,
        max_psi_angle_rad=max_psi_angle_rad,

        # === LIMITY PRĘDKOŚCI LINIOWYCH ===
        max_horizontal_vel=max_horizontal_vel,
        max_ascent=max_ascent,
        max_descent=max_descent,
        max_descent_tilted=max_descent_tilted,

        # === PARAMETRY STEROWANIA LQR ===
        Q_velocity_x=Q_velocity_x,
        Q_velocity_y=Q_velocity_y,
        Q_velocity_z=Q_velocity_z,
        Q_angular_rate_p=Q_angular_rate_p,
        Q_angular_rate_q=Q_angular_rate_q,
        Q_angular_rate_r=Q_angular_rate_r,
        Q_position_X=Q_position_X,
        Q_position_Y=Q_position_Y,
        Q_position_Z=Q_position_Z,
        Q_phi=Q_phi,
        Q_theta=Q_theta,
        Q_psi=Q_psi,
        Q_thrust=Q_thrust,
        r=r,

        # === PARAMETRY ANALIZY TRAJEKTORII ===
        n=n,
        include_thrust_dynamics=include_thrust_dynamics,
        m=m,
        t=t,
        dt=dt,
        MAX_PATH_ITERATIONS=MAX_PATH_ITERATIONS,
        DETOUR_SEARCH_ANGLES=DETOUR_SEARCH_ANGLES,
        DETOUR_DISTANCE_MULTIPLIER=DETOUR_DISTANCE_MULTIPLIER,
        PATH_SMOOTHING_POINTS=PATH_SMOOTHING_POINTS,
        COLLISION_CHECK_SAMPLES=COLLISION_CHECK_SAMPLES
    )

    print(f"Momenty bezwładności (IX, IY, IZ) = ({config.IX}, {config.IY}, {config.IZ})")
    dron_lqr()

if __name__ == "__main__":
    print("Witaj w programie konfiguratora drona!")
    main()


