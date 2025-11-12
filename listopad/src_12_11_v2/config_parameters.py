import numpy as np
import sys
import os
from config import config

# ===============================
# DOMYŚLNE WARTOŚCI
# ===============================
DEFAULTS_DRONE = {
    "mass": 20.0,
    "mass_engine": 2.0,
    "motor_arm": 1.0,
    "body_length": 0.4,
    "body_width": 0.4,
    "body_height": 0.4,
    "CD_0": 0.30,
    "k_torque": 0.015,
    "tau": 0.05,
    "C_Lp": -0.01,
    "CM_Q": -0.01,
    "C_Nr": -0.01,
}

DEFAULTS_LIMITS = {
    "max_pitch_rate_deg": 300.0,
    "max_roll_rate_deg": 300.0,
    "max_yaw_rate_deg": 100.0,
    "max_theta_angle_deg": 25.0,
    "max_phi_angle_deg": 25.0,
    "max_psi_angle_deg": 20.0,
    "max_horizontal_vel": 23.0,
    "max_ascent": 6.0,
    "max_descent": 5.0,
    "max_descent_tilted": 7.0,
}

DEFAULTS_CONTROL = {
    "Q_velocity_x": 50.0,
    "Q_velocity_y": 50.0,
    "Q_velocity_z": 50.0,
    "Q_angular_rate_p": 0.1,
    "Q_angular_rate_q": 0.1,
    "Q_angular_rate_r": 0.1,
    "Q_position_X": 500.0,
    "Q_position_Y": 500.0,
    "Q_position_Z": 500.0,
    "Q_phi": 1.0,
    "Q_theta": 1.0,
    "Q_psi": 1.0,
    "Q_thrust": 0.1,
    "R_weight": 0.1,
}

DEFAULTS_FLIGHT = {
    "g": 9.81,
    "RO_0": 1.225,
    "velocity": 5.0,
    "altitude_start": 0.0,          # ujednolicone z promptem
    "altitude_cruise_min": 2.0,
    "altitude_cruise_max": 8.0,
    "altitude_end": 0.0,
    "avoid_distance": 4.0,
    "ax_wind": 0.0,
    "ay_wind": 0.0,
    "az_wind": 0.0,
    "c_turb": 1000.0,
    "x_turb_1": 1500.0,
    "x_turb_2": 2000.0,
    "engine_on": False,
}

DEFAULTS_PATH = {
    "n": 16,
    "include_thrust_dynamics": True,
    "m": 4,
    "t": 0.0,
    "dt": 0.01,
    "grid_resolution": 5.0,
}


# ===============================
# FUNKCJE ZBIERAJĄCE PARAMETRY
# ===============================
def drone_parameters(use_default: bool = False):
    D = DEFAULTS_DRONE
    print("\nPARAMETRY FIZYCZNE DRONA")

    mass_input = "" if use_default else input(f"Masa drona [kg] [{D['mass']}]: ").strip()
    mass = float(mass_input) if mass_input else D["mass"]
    if use_default:
        print(f"Masa drona: {mass} kg")

    mass_engine_input = "" if use_default else input(f"Masa pojedynczego rotora [kg] [{D['mass_engine']}]: ").strip()
    mass_engine = float(mass_engine_input) if mass_engine_input else D["mass_engine"]
    if use_default:
        print(f"Masa pojedynczego rotora: {mass_engine} kg")

    motor_arm_input = "" if use_default else input(f"Długość ramienia (od środka do rotoru) [m] [{D['motor_arm']}]: ").strip()
    motor_arm = float(motor_arm_input) if motor_arm_input else D["motor_arm"]
    if use_default:
        print(f"Długość ramienia (od środka do rotoru): {motor_arm} m")

    print("\n--- Wymiary korpusu drona ---")
    body_length_input = "" if use_default else input(f"  Długość korpusu [m] [{D['body_length']}]: ").strip()
    body_length = float(body_length_input) if body_length_input else D["body_length"]
    if use_default:
        print(f"Długość korpusu: {body_length} m")

    body_width_input = "" if use_default else input(f"  Szerokość korpusu [m] [{D['body_width']}]: ").strip()
    body_width = float(body_width_input) if body_width_input else D["body_width"]
    if use_default:
        print(f"Szerokość korpusu: {body_width} m")

    body_height_input = "" if use_default else input(f"  Wysokość korpusu [m] [{D['body_height']}]: ").strip()
    body_height = float(body_height_input) if body_height_input else D["body_height"]
    if use_default:
        print(f"Wysokość korpusu: {body_height} m")

    cd_input = "" if use_default else input(f"Współczynnik oporu CD_0 [{D['CD_0']}]: ").strip()
    CD_0 = float(cd_input) if cd_input else D["CD_0"]
    if use_default:
        print(f"CD_0: {CD_0}")

    k_torque_input = "" if use_default else input(f"Współczynnik momentu śmigła k_torque [{D['k_torque']}]: ").strip()
    k_torque = float(k_torque_input) if k_torque_input else D["k_torque"]
    if use_default:
        print(f"k_torque: {k_torque}")

    tau_input = "" if use_default else input(f"Stała czasowa silnika TAU [s] [{D['tau']}]: ").strip()
    tau = float(tau_input) if tau_input else D["tau"]
    if use_default:
        print(f"TAU: {tau} s")

    print("\n--- Współczynniki tłumienia ---")
    C_Lp_input = "" if use_default else input(f"Współczynnik tłumienia roll C_Lp [{D['C_Lp']}]: ").strip()
    C_Lp = float(C_Lp_input) if C_Lp_input else D["C_Lp"]
    if use_default:
        print(f"C_Lp: {C_Lp}")

    CM_Q_input = "" if use_default else input(f"Współczynnik tłumienia pitch CM_Q [{D['CM_Q']}]: ").strip()
    CM_Q = float(CM_Q_input) if CM_Q_input else D["CM_Q"]
    if use_default:
        print(f"CM_Q: {CM_Q}")

    C_Nr_input = "" if use_default else input(f"Współczynnik tłumienia yaw C_Nr [{D['C_Nr']}]: ").strip()
    C_Nr = float(C_Nr_input) if C_Nr_input else D["C_Nr"]
    if use_default:
        print(f"C_Nr: {C_Nr}")

    return mass, mass_engine, motor_arm, body_length, body_width, body_height, CD_0, k_torque, tau, C_Lp, CM_Q, C_Nr

def limits(use_default: bool = False):
    L = DEFAULTS_LIMITS
    print("\nLIMITY FIZYCZNE DRONA\n--- Limity ciągu [N] ---")

    # thrust_max_input = "" if use_default else input(f"Maksymalny ciąg na silnik [N] [{L['thrust_max']}]: ").strip()
    # thrust_max = float(thrust_max_input) if thrust_max_input else L["thrust_max"]
    # if use_default:
    #     print(f"Maksymalny ciąg na silnik: {thrust_max} N")
    #
    # thrust_min_input = "" if use_default else input(f"Minimalny ciąg na silnik [N] [{L['thrust_min']}]: ").strip()
    # thrust_min = float(thrust_min_input) if thrust_min_input else L["thrust_min"]
    # if use_default:
    #     print(f"Minimalny ciąg na silnik: {thrust_min} N")

    print("\n--- Limity prędkości kątowych [°/s] ---")
    max_pitch_rate_input = "" if use_default else input(f"Maksymalna prędkość pitch [°/s] [{L['max_pitch_rate_deg']}]: ").strip()
    max_pitch_rate_deg = float(max_pitch_rate_input) if max_pitch_rate_input else L["max_pitch_rate_deg"]
    if use_default:
        print(f"Maksymalna prędkość pitch: {max_pitch_rate_deg} °/s")
    max_pitch_rate_rad = np.deg2rad(max_pitch_rate_deg)

    max_roll_rate_input = "" if use_default else input(f"Maksymalna prędkość roll [°/s] [{L['max_roll_rate_deg']}]: ").strip()
    max_roll_rate_deg = float(max_roll_rate_input) if max_roll_rate_input else L["max_roll_rate_deg"]
    if use_default:
        print(f"Maksymalna prędkość roll: {max_roll_rate_deg} °/s")
    max_roll_rate_rad = np.deg2rad(max_roll_rate_deg)

    max_yaw_rate_input = "" if use_default else input(f"Maksymalna prędkość yaw [°/s] [{L['max_yaw_rate_deg']}]: ").strip()
    max_yaw_rate_deg = float(max_yaw_rate_input) if max_yaw_rate_input else L["max_yaw_rate_deg"]
    if use_default:
        print(f"Maksymalna prędkość yaw: {max_yaw_rate_deg} °/s")
    max_yaw_rate_rad = np.deg2rad(max_yaw_rate_deg)

    print("\n--- Limity kątów [°] ---")
    max_theta_angle_input = "" if use_default else input(f"Maksymalny kąt (theta) [°] [{L['max_theta_angle_deg']}]: ").strip()
    max_theta_angle_deg = float(max_theta_angle_input) if max_theta_angle_input else L["max_theta_angle_deg"]
    if use_default:
        print(f"Maksymalny kąt (theta): {max_theta_angle_deg} °")
    max_theta_angle_rad = np.deg2rad(max_theta_angle_deg)

    max_phi_angle_input = "" if use_default else input(f"Maksymalny kąt (phi) [°] [{L['max_phi_angle_deg']}]: ").strip()
    max_phi_angle_deg = float(max_phi_angle_input) if max_phi_angle_input else L["max_phi_angle_deg"]
    if use_default:
        print(f"Maksymalny kąt (phi): {max_phi_angle_deg} °")
    max_phi_angle_rad = np.deg2rad(max_phi_angle_deg)

    max_psi_angle_input = "" if use_default else input(f"Maksymalny kąt (psi) [°] [{L['max_psi_angle_deg']}]: ").strip()
    max_psi_angle_deg = float(max_psi_angle_input) if max_psi_angle_input else L["max_psi_angle_deg"]
    if use_default:
        print(f"Maksymalny kąt (psi): {max_psi_angle_deg} °")
    max_psi_angle_rad = np.deg2rad(max_psi_angle_deg)

    print("\n--- Limity prędkości liniowych [m/s] ---")
    max_horizontal_vel_input = "" if use_default else input(f"Maksymalna prędkość pozioma [m/s] [{L['max_horizontal_vel']}]: ").strip()
    max_horizontal_vel = float(max_horizontal_vel_input) if max_horizontal_vel_input else L["max_horizontal_vel"]
    if use_default:
        print(f"Maksymalna prędkość pozioma: {max_horizontal_vel} m/s")

    max_ascent_input = "" if use_default else input(f"Maksymalna prędkość wznoszenia [m/s] [{L['max_ascent']}]: ").strip()
    max_ascent = float(max_ascent_input) if max_ascent_input else L["max_ascent"]
    if use_default:
        print(f"Maksymalna prędkość wznoszenia: {max_ascent} m/s")

    max_descent_input = "" if use_default else input(f"Maksymalna prędkość opadania [m/s] [{L['max_descent']}]: ").strip()
    max_descent = float(max_descent_input) if max_descent_input else L["max_descent"]
    if use_default:
        print(f"Maksymalna prędkość opadania: {max_descent} m/s")

    max_descent_tilted_input = "" if use_default else input(f"Maks. prędkość opadania (z przechyłem) [m/s] [{L['max_descent_tilted']}]: ").strip()
    max_descent_tilted = float(max_descent_tilted_input) if max_descent_tilted_input else L["max_descent_tilted"]
    if use_default:
        print(f"Maks. prędkość opadania (z przechyłem): {max_descent_tilted} m/s")

    return (max_pitch_rate_rad, max_roll_rate_rad, max_yaw_rate_rad,
            max_theta_angle_rad, max_phi_angle_rad, max_psi_angle_rad,
            max_horizontal_vel, max_ascent, max_descent, max_descent_tilted)

def control_parameters(use_default: bool = False):
    C = DEFAULTS_CONTROL
    print("\nPARAMETRY STEROWANIA\n--- Wagi LQR dla stanu (Q) ---")

    Q_velocity_x_input = "" if use_default else input(f"Waga dla prędkości (vx) [{C['Q_velocity_x']}]: ").strip()
    Q_velocity_x = float(Q_velocity_x_input) if Q_velocity_x_input else C["Q_velocity_x"]
    if use_default:
        print(f"Q vx: {Q_velocity_x}")

    Q_velocity_y_input = "" if use_default else input(f"Waga dla prędkości (vy) [{C['Q_velocity_y']}]: ").strip()
    Q_velocity_y = float(Q_velocity_y_input) if Q_velocity_y_input else C["Q_velocity_y"]
    if use_default:
        print(f"Q vy: {Q_velocity_y}")

    Q_velocity_z_input = "" if use_default else input(f"Waga dla prędkości (vz) [{C['Q_velocity_z']}]: ").strip()
    Q_velocity_z = float(Q_velocity_z_input) if Q_velocity_z_input else C["Q_velocity_z"]
    if use_default:
        print(f"Q vz: {Q_velocity_z}")

    Q_angular_rate_p_input = "" if use_default else input(f"Waga dla prędkości kątowej (p) [{C['Q_angular_rate_p']}]: ").strip()
    Q_angular_rate_p = float(Q_angular_rate_p_input) if Q_angular_rate_p_input else C["Q_angular_rate_p"]
    if use_default:
        print(f"Q p: {Q_angular_rate_p}")

    Q_angular_rate_q_input = "" if use_default else input(f"Waga dla prędkości kątowej (q) [{C['Q_angular_rate_q']}]: ").strip()
    Q_angular_rate_q = float(Q_angular_rate_q_input) if Q_angular_rate_q_input else C["Q_angular_rate_q"]
    if use_default:
        print(f"Q q: {Q_angular_rate_q}")

    Q_angular_rate_r_input = "" if use_default else input(f"Waga dla prędkości kątowej (r) [{C['Q_angular_rate_r']}]: ").strip()
    Q_angular_rate_r = float(Q_angular_rate_r_input) if Q_angular_rate_r_input else C["Q_angular_rate_r"]
    if use_default:
        print(f"Q r: {Q_angular_rate_r}")

    Q_position_X_input = "" if use_default else input(f"Waga dla pozycji (X) [{C['Q_position_X']}]: ").strip()
    Q_position_X = float(Q_position_X_input) if Q_position_X_input else C["Q_position_X"]
    if use_default:
        print(f"Q X: {Q_position_X}")

    Q_position_Y_input = "" if use_default else input(f"Waga dla pozycji (Y) [{C['Q_position_Y']}]: ").strip()
    Q_position_Y = float(Q_position_Y_input) if Q_position_Y_input else C["Q_position_Y"]
    if use_default:
        print(f"Q Y: {Q_position_Y}")

    Q_position_Z_input = "" if use_default else input(f"Waga dla pozycji (Z) [{C['Q_position_Z']}]: ").strip()
    Q_position_Z = float(Q_position_Z_input) if Q_position_Z_input else C["Q_position_Z"]
    if use_default:
        print(f"Q Z: {Q_position_Z}")

    Q_phi_input = "" if use_default else input(f"Waga dla kątów (phi) [{C['Q_phi']}]: ").strip()
    Q_phi = float(Q_phi_input) if Q_phi_input else C["Q_phi"]
    if use_default:
        print(f"Q phi: {Q_phi}")

    Q_theta_input = "" if use_default else input(f"Waga dla kątów (theta) [{C['Q_theta']}]: ").strip()
    Q_theta = float(Q_theta_input) if Q_theta_input else C["Q_theta"]
    if use_default:
        print(f"Q theta: {Q_theta}")

    Q_psi_input = "" if use_default else input(f"Waga dla kątów (psi) [{C['Q_psi']}]: ").strip()
    Q_psi = float(Q_psi_input) if Q_psi_input else C["Q_psi"]
    if use_default:
        print(f"Q psi: {Q_psi}")

    Q_thrust_input = "" if use_default else input(f"Waga dla ciągów (T1-T4, dla n=16) [{C['Q_thrust']}]: ").strip()
    Q_thrust = float(Q_thrust_input) if Q_thrust_input else C["Q_thrust"]
    if use_default:
        print(f"Q thrust: {Q_thrust}")

    R_input = "" if use_default else input(f"Waga dla sterowania (R) [{C['R']}]: ").strip()
    R_weight = float(R_input) if R_input else C["R_weight"]
    if use_default:
        print(f"R: {R_weight}")

    return (Q_velocity_x, Q_velocity_y, Q_velocity_z,
            Q_angular_rate_p, Q_angular_rate_q, Q_angular_rate_r,
            Q_position_X, Q_position_Y, Q_position_Z,
            Q_phi, Q_theta, Q_psi, Q_thrust, R_weight)

def flight_parameters(use_default: bool = False):
    F = DEFAULTS_FLIGHT
    print("\nPARAMETRY LOTU")

    g_input = "" if use_default else input(f"Przyspieszenie ziemskie [{F['g']}]: ").strip()
    g = float(g_input) if g_input else F["g"]
    if use_default:
        print(f"g: {g}")

    RO_0_input = "" if use_default else input(f"Gęstość powietrza [{F['RO_0']} kg/m^3]: ").strip()
    RO_0 = float(RO_0_input) if RO_0_input else F["RO_0"]
    if use_default:
        print(f"Gęstość powietrza: {RO_0} kg/m^3")

    velocity_input = "" if use_default else input(f"Prędkość referencyjna drona [m/s] [{F['velocity']}]: ").strip()
    velocity = float(velocity_input) if velocity_input else F["velocity"]
    if use_default:
        print(f"Prędkość referencyjna: {velocity} m/s")

    print("\n--- Wysokości lotu ---")
    altitude_start_input = "" if use_default else input(f"Wysokość startowa [m] [{F['altitude_start']}]: ").strip()
    altitude_start = float(altitude_start_input) if altitude_start_input else F["altitude_start"]
    if use_default:
        print(f"Wysokość startowa: {altitude_start} m")

    altitude_cruise_min_input = "" if use_default else input(f"Wysokość przelotowa min [m] [{F['altitude_cruise_min']}]: ").strip()
    altitude_cruise_min = float(altitude_cruise_min_input) if altitude_cruise_min_input else F["altitude_cruise_min"]
    if use_default:
        print(f"Wysokość przelotowa min: {altitude_cruise_min} m")

    altitude_cruise_max_input = "" if use_default else input(f"Wysokość przelotowa max [m] [{F['altitude_cruise_max']}]: ").strip()
    altitude_cruise_max = float(altitude_cruise_max_input) if altitude_cruise_max_input else F["altitude_cruise_max"]
    if use_default:
        print(f"Wysokość przelotowa max: {altitude_cruise_max} m")

    altitude_end_input = "" if use_default else input(f"Wysokość lądowania [m] [{F['altitude_end']}]: ").strip()
    altitude_end = float(altitude_end_input) if altitude_end_input else F["altitude_end"]
    if use_default:
        print(f"Wysokość lądowania: {altitude_end} m")

    avoid_distance_input = "" if use_default else input(f"Bezpieczna odległość [m] [{F['avoid_distance']}]: ").strip()
    avoid_distance = float(avoid_distance_input) if avoid_distance_input else F["avoid_distance"]
    if use_default:
        print(f"Bezpieczna odległość: {avoid_distance} m")

    print("\n--- Turbulencje i wiatr ---")
    ax_wind_input = "" if use_default else input(f"Przyspieszenie wiatru poziomego [m/s²] [{F['ax_wind']}]: ").strip()
    ax_wind = float(ax_wind_input) if ax_wind_input else F["ax_wind"]
    if use_default:
        print(f"ax_wind: {ax_wind} m/s²")

    ay_wind_input = "" if use_default else input(f"Przyspieszenie wiatru poprzecznego [m/s²] [{F['ay_wind']}]: ").strip()
    ay_wind = float(ay_wind_input) if ay_wind_input else F["ay_wind"]
    if use_default:
        print(f"ay_wind: {ay_wind} m/s²")

    az_wind_input = "" if use_default else input(f"Przyspieszenie wiatru pionowego [m/s²] [{F['az_wind']}]: ").strip()
    az_wind = float(az_wind_input) if az_wind_input else F["az_wind"]
    if use_default:
        print(f"az_wind: {az_wind} m/s²")

    c_turb_input = "" if use_default else input(f"Siła turbulencji c_turb [{F['c_turb']}]: ").strip()
    c_turb = float(c_turb_input) if c_turb_input else F["c_turb"]
    if use_default:
        print(f"c_turb: {c_turb}")

    x_turb1_input = "" if use_default else input(f"Początek strefy turbulencji [m] [{F['x_turb_1']}]: ").strip()
    x_turb_1 = float(x_turb1_input) if x_turb1_input else F["x_turb_1"]
    if use_default:
        print(f"Początek strefy turbulencji: {x_turb_1} m")

    x_turb2_input = "" if use_default else input(f"Koniec strefy turbulencji [m] [{F['x_turb_2']}]: ").strip()
    x_turb_2 = float(x_turb2_input) if x_turb2_input else F["x_turb_2"]
    if use_default:
        print(f"Koniec strefy turbulencji: {x_turb_2} m")

    engine_choice = "" if use_default else input(
        f"Start na wysokości {altitude_start} m. Silniki włączone? (t/n) [{'t' if F['engine_on'] else 'n'}]: "
    ).strip().lower()
    engine_on = (F["engine_on"] if use_default else (engine_choice in ["t", "tak", "y", "yes"]))
    if use_default:
        print(f"Silniki włączone na starcie: {'tak' if engine_on else 'nie'}")

    return (g, RO_0, velocity,
            altitude_start, altitude_cruise_min, altitude_cruise_max, altitude_end,
            avoid_distance, ax_wind, ay_wind, az_wind, c_turb, x_turb_1, x_turb_2, engine_on)

def path_analise_parameters(use_default: bool = False):
    P = DEFAULTS_PATH
    print("\nPARAMETRY ANALIZY TRAJEKTORII")

    if use_default:
        thrust_dyn_input = ""
    else:
        print("\nCzy uwzględnić dynamikę silników?")
        print("  t/tak - model z dynamiką (n=16, dokładniejszy)")
        print("  n/nie - model prosty (n=12, szybszy)")
        thrust_dyn_input = input("Wybór [t]: ").strip().lower()

    if thrust_dyn_input in ["n", "no", "nie"]:
        n = 12
        include_thrust_dynamics = False
    else:
        n = 16
        include_thrust_dynamics = True
    if use_default:
        print(f"Model: {'z dynamiką (n=16)' if include_thrust_dynamics else 'prosty (n=12)'}")

    m = P["m"]
    if use_default:
        print(f"Liczba silników (m): {m}")

    t_input = "" if use_default else input(f"Podaj czas początkowy t [s] [{P['t']}]: ").strip()
    t = float(t_input) if t_input else P["t"]
    if use_default:
        print(f"Czas początkowy t: {t} s")

    dt_input = "" if use_default else input(f"Krok czasowy dt [s] [{P['dt']}]: ").strip()
    dt = float(dt_input) if dt_input else P["dt"]
    if use_default:
        print(f"Krok czasowy dt: {dt} s")

    grid_resolution_input = "" if use_default else input(f"Podaj rozdzielczość grid [{P['grid_resolution']}]: ").strip()
    grid_resolution = float(grid_resolution_input) if grid_resolution_input else P["grid_resolution"]
    if use_default:
        print(f"Rozdzielczość grid: {grid_resolution}")

    return n, include_thrust_dynamics, m, t, dt, grid_resolution


def input_parameters():
    print("Wciśnij Enter, aby użyć wartości domyślnej.\n")
    choice = input("Czy chcesz wpisać własne dane? (t/n) [n]: ").strip().lower()
    use_default = False if choice == "t" else True

    # === PARAMETRY FIZYCZNE DRONA ===
    (mass, mass_engine, motor_arm, body_length, body_width, body_height,
     CD_0, k_torque, tau, C_Lp, CM_Q, C_Nr) = drone_parameters(use_default)

    # === LIMITY DRONA ===
    (max_pitch_rate, max_roll_rate, max_yaw_rate,
     max_theta_angle_rad, max_phi_angle_rad, max_psi_angle_rad,
     max_horizontal_vel, max_ascent, max_descent, max_descent_tilted) = limits(use_default)


    # === PARAMETRY STEROWANIA ===
    (Q_velocity_x, Q_velocity_y, Q_velocity_z,
     Q_angular_rate_p, Q_angular_rate_q, Q_angular_rate_r,
     Q_position_X, Q_position_Y, Q_position_Z,
     Q_phi, Q_theta, Q_psi, Q_thrust, R_weight) = control_parameters(use_default)

    # === PARAMETRY LOTU ===
    (g, RO_0, velocity, altitude_start, altitude_cruise_min, altitude_cruise_max,
     altitude_end, avoid_distance, ax_wind, ay_wind, az_wind, c_turb,
     x_turb_1, x_turb_2, engine_on) = flight_parameters(use_default)

    # === PARAMETRY ANALIZY TRAJEKTORII ===
    n, include_thrust_dynamics, m, t, dt, grid_resolution = path_analise_parameters(use_default)

    # === ZAPIS DO CONFIG ===
    config.set_params(
        # Fizyczne
        mass=mass,
        mass_engine=mass_engine,
        motor_arm=motor_arm,
        body_length=body_length,
        body_width=body_width,
        body_height=body_height,
        size=body_height * body_width,
        CD_0=CD_0,
        k_torque=k_torque,
        tau=tau,
        C_Lp=C_Lp,
        CM_Q=CM_Q,
        C_Nr=C_Nr,

        # Lot
        g=g,
        RO_0=RO_0,
        velocity=velocity,
        altitude_start=altitude_start,
        altitude_cruise_min=altitude_cruise_min,
        altitude_cruise_max=altitude_cruise_max,
        altitude_end=altitude_end,
        avoid_distance=avoid_distance,
        ax_wind=ax_wind,
        ay_wind=ay_wind,
        az_wind=az_wind,
        c_turb=c_turb,
        x_turb_1=x_turb_1,
        x_turb_2=x_turb_2,
        engine_on=engine_on,

        # Limity ciągu
        # thrust_max=thrust_max,
        # thrust_min=thrust_min,

        # Limity prędkości kątowych
        max_pitch_rate_rad=max_pitch_rate,
        max_roll_rate_rad=max_roll_rate,
        max_yaw_rate_rad=max_yaw_rate,

        # Limity kątów
        max_theta_angle_rad=max_theta_angle_rad,
        max_phi_angle_rad=max_phi_angle_rad,
        max_psi_angle_rad=max_psi_angle_rad,

        # Limity prędkości liniowych
        max_horizontal_vel=max_horizontal_vel,
        max_ascent=max_ascent,
        max_descent=max_descent,
        max_descent_tilted=max_descent_tilted,

        # Sterowanie LQR
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
        R_weight=R_weight,

        # Analiza trajektorii
        n=n,
        include_thrust_dynamics=include_thrust_dynamics,
        m=m,
        t=t,
        dt=dt,
        grid_resolution=grid_resolution,
    )

    print(f"Momenty bezwładności (IX, IY, IZ) = ({config.IX}, {config.IY}, {config.IZ})")


if __name__ == "__main__":
    print("Witaj w programie konfiguratora drona!")
    input_parameters()
