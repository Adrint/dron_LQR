import numpy as np
import sys
import os
from config import config


def input_float(prompt: str, default: float) -> float:
    """Bezpieczny input: liczba lub enter = domyślna wartość."""
    while True:
        user_input = input(f"{prompt} [{default}]: ").strip()
        if user_input == "":
            return default
        try:
            return float(user_input)
        except ValueError:
            print("✗ Wprowadź poprawną liczbę.")


def input_yes_no(prompt: str, default: bool) -> bool:
    """Bezpieczny input: t/n z obsługą domyślnej wartości (enter)."""
    default_str = "t" if default else "n"
    while True:
        user_input = input(f"{prompt} (t/n) [{default_str}]: ").strip().lower()
        if user_input == "":
            return default
        if user_input in ["t", "tak", "y", "yes"]:
            return True
        if user_input in ["n", "nie", "no"]:
            return False
        print("✗ Wprowadź 't' lub 'n'.")


# ===============================
# DOMYŚLNE WARTOŚCI
# ===============================
DEFAULTS_DRONE = {
    "mass": 9.0,
    "mass_engine": 1.0,
    "motor_arm": 0.7,
    "body_length": 0.2,
    "body_width": 0.2,
    "body_height": 0.2,
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
    "Q_velocity_x": 20.0,
    "Q_velocity_y": 20.0,
    "Q_velocity_z": 20.0,
    "Q_angular_rate_p": 0.1,
    "Q_angular_rate_q": 0.1,
    "Q_angular_rate_r": 0.1,
    "Q_position_X": 100.0,
    "Q_position_Y": 100.0,
    "Q_position_Z": 100.0,
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
    "altitude_cruise_min": 1.0,
    "altitude_cruise_max": 3.0,
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
    "dt": 0.05,
    "grid_resolution": 3.0,
}


# ===============================
# FUNKCJE ZBIERAJĄCE PARAMETRY
# ===============================
def drone_parameters(use_default: bool = False):
    D = DEFAULTS_DRONE
    print("\nPARAMETRY FIZYCZNE DRONA")

    if use_default:
        mass = D["mass"]
        print(f"Masa drona: {mass} kg")
    else:
        mass = input_float("Masa drona [kg]", D["mass"])

    if use_default:
        mass_engine = D["mass_engine"]
        print(f"Masa pojedynczego rotora: {mass_engine} kg")
    else:
        mass_engine = input_float("Masa pojedynczego rotora [kg]", D["mass_engine"])

    if use_default:
        motor_arm = D["motor_arm"]
        print(f"Długość ramienia (od środka do rotoru): {motor_arm} m")
    else:
        motor_arm = input_float("Długość ramienia (od środka do rotoru) [m]", D["motor_arm"])

    print("\n--- Wymiary korpusu drona ---")

    if use_default:
        body_length = D["body_length"]
        print(f"Długość korpusu: {body_length} m")
    else:
        body_length = input_float("  Długość korpusu [m]", D["body_length"])

    if use_default:
        body_width = D["body_width"]
        print(f"Szerokość korpusu: {body_width} m")
    else:
        body_width = input_float("  Szerokość korpusu [m]", D["body_width"])

    if use_default:
        body_height = D["body_height"]
        print(f"Wysokość korpusu: {body_height} m")
    else:
        body_height = input_float("  Wysokość korpusu [m]", D["body_height"])

    if use_default:
        CD_0 = D["CD_0"]
        print(f"CD_0: {CD_0}")
    else:
        CD_0 = input_float("Współczynnik oporu CD_0", D["CD_0"])

    if use_default:
        k_torque = D["k_torque"]
        print(f"k_torque: {k_torque}")
    else:
        k_torque = input_float("Współczynnik momentu śmigła k_torque", D["k_torque"])

    if use_default:
        tau = D["tau"]
        print(f"TAU: {tau} s")
    else:
        tau = input_float("Stała czasowa silnika TAU [s]", D["tau"])

    print("\n--- Współczynniki tłumienia ---")

    if use_default:
        C_Lp = D["C_Lp"]
        print(f"C_Lp: {C_Lp}")
    else:
        C_Lp = input_float("Współczynnik tłumienia roll C_Lp", D["C_Lp"])

    if use_default:
        CM_Q = D["CM_Q"]
        print(f"CM_Q: {CM_Q}")
    else:
        CM_Q = input_float("Współczynnik tłumienia pitch CM_Q", D["CM_Q"])

    if use_default:
        C_Nr = D["C_Nr"]
        print(f"C_Nr: {C_Nr}")
    else:
        C_Nr = input_float("Współczynnik tłumienia yaw C_Nr", D["C_Nr"])

    return mass, mass_engine, motor_arm, body_length, body_width, body_height, CD_0, k_torque, tau, C_Lp, CM_Q, C_Nr


def limits(use_default: bool = False):
    L = DEFAULTS_LIMITS
    print("\nLIMITY FIZYCZNE DRONA\n--- Limity ciągu [N] ---")

    # thrust_max / thrust_min jeśli będzie potrzebne można dodać z input_float

    print("\n--- Limity prędkości kątowych [°/s] ---")

    if use_default:
        max_pitch_rate_deg = L["max_pitch_rate_deg"]
        print(f"Maksymalna prędkość pitch: {max_pitch_rate_deg} °/s")
    else:
        max_pitch_rate_deg = input_float("Maksymalna prędkość pitch [°/s]", L["max_pitch_rate_deg"])
    max_pitch_rate_rad = np.deg2rad(max_pitch_rate_deg)

    if use_default:
        max_roll_rate_deg = L["max_roll_rate_deg"]
        print(f"Maksymalna prędkość roll: {max_roll_rate_deg} °/s")
    else:
        max_roll_rate_deg = input_float("Maksymalna prędkość roll [°/s]", L["max_roll_rate_deg"])
    max_roll_rate_rad = np.deg2rad(max_roll_rate_deg)

    if use_default:
        max_yaw_rate_deg = L["max_yaw_rate_deg"]
        print(f"Maksymalna prędkość yaw: {max_yaw_rate_deg} °/s")
    else:
        max_yaw_rate_deg = input_float("Maksymalna prędkość yaw [°/s]", L["max_yaw_rate_deg"])
    max_yaw_rate_rad = np.deg2rad(max_yaw_rate_deg)

    print("\n--- Limity kątów [°] ---")

    if use_default:
        max_theta_angle_deg = L["max_theta_angle_deg"]
        print(f"Maksymalny kąt (theta): {max_theta_angle_deg} °")
    else:
        max_theta_angle_deg = input_float("Maksymalny kąt (theta) [°]", L["max_theta_angle_deg"])
    max_theta_angle_rad = np.deg2rad(max_theta_angle_deg)

    if use_default:
        max_phi_angle_deg = L["max_phi_angle_deg"]
        print(f"Maksymalny kąt (phi): {max_phi_angle_deg} °")
    else:
        max_phi_angle_deg = input_float("Maksymalny kąt (phi) [°]", L["max_phi_angle_deg"])
    max_phi_angle_rad = np.deg2rad(max_phi_angle_deg)

    if use_default:
        max_psi_angle_deg = L["max_psi_angle_deg"]
        print(f"Maksymalny kąt (psi): {max_psi_angle_deg} °")
    else:
        max_psi_angle_deg = input_float("Maksymalny kąt (psi) [°]", L["max_psi_angle_deg"])
    max_psi_angle_rad = np.deg2rad(max_psi_angle_deg)

    print("\n--- Limity prędkości liniowych [m/s] ---")

    if use_default:
        max_horizontal_vel = L["max_horizontal_vel"]
        print(f"Maksymalna prędkość pozioma: {max_horizontal_vel} m/s")
    else:
        max_horizontal_vel = input_float("Maksymalna prędkość pozioma [m/s]", L["max_horizontal_vel"])

    if use_default:
        max_ascent = L["max_ascent"]
        print(f"Maksymalna prędkość wznoszenia: {max_ascent} m/s")
    else:
        max_ascent = input_float("Maksymalna prędkość wznoszenia [m/s]", L["max_ascent"])

    if use_default:
        max_descent = L["max_descent"]
        print(f"Maksymalna prędkość opadania: {max_descent} m/s")
    else:
        max_descent = input_float("Maksymalna prędkość opadania [m/s]", L["max_descent"])

    if use_default:
        max_descent_tilted = L["max_descent_tilted"]
        print(f"Maks. prędkość opadania (z przechyłem): {max_descent_tilted} m/s")
    else:
        max_descent_tilted = input_float("Maks. prędkość opadania (z przechyłem) [m/s]", L["max_descent_tilted"])

    return (max_pitch_rate_rad, max_roll_rate_rad, max_yaw_rate_rad,
            max_theta_angle_rad, max_phi_angle_rad, max_psi_angle_rad,
            max_horizontal_vel, max_ascent, max_descent, max_descent_tilted)


def control_parameters(use_default: bool = False):
    C = DEFAULTS_CONTROL
    print("\nPARAMETRY STEROWANIA\n--- Wagi LQR dla stanu (Q) ---")

    if use_default:
        Q_velocity_x = C["Q_velocity_x"]
        print(f"Q vx: {Q_velocity_x}")
    else:
        Q_velocity_x = input_float("Waga dla prędkości (vx)", C["Q_velocity_x"])

    if use_default:
        Q_velocity_y = C["Q_velocity_y"]
        print(f"Q vy: {Q_velocity_y}")
    else:
        Q_velocity_y = input_float("Waga dla prędkości (vy)", C["Q_velocity_y"])

    if use_default:
        Q_velocity_z = C["Q_velocity_z"]
        print(f"Q vz: {Q_velocity_z}")
    else:
        Q_velocity_z = input_float("Waga dla prędkości (vz)", C["Q_velocity_z"])

    if use_default:
        Q_angular_rate_p = C["Q_angular_rate_p"]
        print(f"Q p: {Q_angular_rate_p}")
    else:
        Q_angular_rate_p = input_float("Waga dla prędkości kątowej (p)", C["Q_angular_rate_p"])

    if use_default:
        Q_angular_rate_q = C["Q_angular_rate_q"]
        print(f"Q q: {Q_angular_rate_q}")
    else:
        Q_angular_rate_q = input_float("Waga dla prędkości kątowej (q)", C["Q_angular_rate_q"])

    if use_default:
        Q_angular_rate_r = C["Q_angular_rate_r"]
        print(f"Q r: {Q_angular_rate_r}")
    else:
        Q_angular_rate_r = input_float("Waga dla prędkości kątowej (r)", C["Q_angular_rate_r"])

    if use_default:
        Q_position_X = C["Q_position_X"]
        print(f"Q X: {Q_position_X}")
    else:
        Q_position_X = input_float("Waga dla pozycji (X)", C["Q_position_X"])

    if use_default:
        Q_position_Y = C["Q_position_Y"]
        print(f"Q Y: {Q_position_Y}")
    else:
        Q_position_Y = input_float("Waga dla pozycji (Y)", C["Q_position_Y"])

    if use_default:
        Q_position_Z = C["Q_position_Z"]
        print(f"Q Z: {Q_position_Z}")
    else:
        Q_position_Z = input_float("Waga dla pozycji (Z)", C["Q_position_Z"])

    if use_default:
        Q_phi = C["Q_phi"]
        print(f"Q phi: {Q_phi}")
    else:
        Q_phi = input_float("Waga dla kątów (phi)", C["Q_phi"])

    if use_default:
        Q_theta = C["Q_theta"]
        print(f"Q theta: {Q_theta}")
    else:
        Q_theta = input_float("Waga dla kątów (theta)", C["Q_theta"])

    if use_default:
        Q_psi = C["Q_psi"]
        print(f"Q psi: {Q_psi}")
    else:
        Q_psi = input_float("Waga dla kątów (psi)", C["Q_psi"])

    if use_default:
        Q_thrust = C["Q_thrust"]
        print(f"Q thrust: {Q_thrust}")
    else:
        Q_thrust = input_float("Waga dla ciągów (T1-T4, dla n=16)", C["Q_thrust"])

    if use_default:
        R_weight = C["R_weight"]
        print(f"R: {R_weight}")
    else:
        R_weight = input_float("Waga dla sterowania (R)", C["R_weight"])

    return (Q_velocity_x, Q_velocity_y, Q_velocity_z,
            Q_angular_rate_p, Q_angular_rate_q, Q_angular_rate_r,
            Q_position_X, Q_position_Y, Q_position_Z,
            Q_phi, Q_theta, Q_psi, Q_thrust, R_weight)


def flight_parameters(use_default: bool = False):
    F = DEFAULTS_FLIGHT
    print("\nPARAMETRY LOTU")

    if use_default:
        g = F["g"]
        print(f"g: {g}")
    else:
        g = input_float("Przyspieszenie ziemskie", F["g"])

    if use_default:
        RO_0 = F["RO_0"]
        print(f"Gęstość powietrza: {RO_0} kg/m^3")
    else:
        RO_0 = input_float("Gęstość powietrza [kg/m^3]", F["RO_0"])

    if use_default:
        velocity = F["velocity"]
        print(f"Prędkość referencyjna: {velocity} m/s")
    else:
        velocity = input_float("Prędkość referencyjna drona [m/s]", F["velocity"])

    print("\n--- Wysokości lotu ---")

    if use_default:
        altitude_start = F["altitude_start"]
        print(f"Wysokość startowa: {altitude_start} m")
    else:
        altitude_start = input_float("Wysokość startowa [m]", F["altitude_start"])

    if use_default:
        altitude_cruise_min = F["altitude_cruise_min"]
        print(f"Wysokość przelotowa min: {altitude_cruise_min} m")
    else:
        altitude_cruise_min = input_float("Wysokość przelotowa min [m]", F["altitude_cruise_min"])

    if use_default:
        altitude_cruise_max = F["altitude_cruise_max"]
        print(f"Wysokość przelotowa max: {altitude_cruise_max} m")
    else:
        altitude_cruise_max = input_float("Wysokość przelotowa max [m]", F["altitude_cruise_max"])

    if use_default:
        altitude_end = F["altitude_end"]
        print(f"Wysokość lądowania: {altitude_end} m")
    else:
        altitude_end = input_float("Wysokość lądowania [m]", F["altitude_end"])

    if use_default:
        avoid_distance = F["avoid_distance"]
        print(f"Bezpieczna odległość: {avoid_distance} m")
    else:
        avoid_distance = input_float("Bezpieczna odległość [m]", F["avoid_distance"])

    print("\n--- Turbulencje i wiatr ---")

    if use_default:
        ax_wind = F["ax_wind"]
        print(f"ax_wind: {ax_wind} m/s²")
    else:
        ax_wind = input_float("Przyspieszenie wiatru poziomego [m/s²]", F["ax_wind"])

    if use_default:
        ay_wind = F["ay_wind"]
        print(f"ay_wind: {ay_wind} m/s²")
    else:
        ay_wind = input_float("Przyspieszenie wiatru poprzecznego [m/s²]", F["ay_wind"])

    if use_default:
        az_wind = F["az_wind"]
        print(f"az_wind: {az_wind} m/s²")
    else:
        az_wind = input_float("Przyspieszenie wiatru pionowego [m/s²]", F["az_wind"])

    if use_default:
        c_turb = F["c_turb"]
        print(f"c_turb: {c_turb}")
    else:
        c_turb = input_float("Siła turbulencji c_turb", F["c_turb"])

    if use_default:
        x_turb_1 = F["x_turb_1"]
        print(f"Początek strefy turbulencji: {x_turb_1} m")
    else:
        x_turb_1 = input_float("Początek strefy turbulencji [m]", F["x_turb_1"])

    if use_default:
        x_turb_2 = F["x_turb_2"]
        print(f"Koniec strefy turbulencji: {x_turb_2} m")
    else:
        x_turb_2 = input_float("Koniec strefy turbulencji [m]", F["x_turb_2"])

    if use_default:
        engine_on = F["engine_on"]
        print(f"Silniki włączone na starcie: {'tak' if engine_on else 'nie'}")
    else:
        engine_on = input_yes_no(
            f"Start na wysokości {altitude_start} m. Silniki włączone?",
            F["engine_on"]
        )

    return (g, RO_0, velocity,
            altitude_start, altitude_cruise_min, altitude_cruise_max, altitude_end,
            avoid_distance, ax_wind, ay_wind, az_wind, c_turb, x_turb_1, x_turb_2, engine_on)


def path_analise_parameters(use_default: bool = False):
    P = DEFAULTS_PATH
    print("\nPARAMETRY ANALIZY TRAJEKTORII")

    if use_default:
        include_thrust_dynamics = P["include_thrust_dynamics"]
        n = 16 if include_thrust_dynamics else 12
        print(f"Model: {'z dynamiką (n=16)' if include_thrust_dynamics else 'prosty (n=12)'}")
    else:
        print("\nCzy uwzględnić dynamikę silników?")
        print("  t/tak - model z dynamiką (n=16, dokładniejszy)")
        print("  n/nie - model prosty (n=12, szybszy)")
        include_thrust_dynamics = input_yes_no("Wybór", True)
        if include_thrust_dynamics:
            n = 16
        else:
            n = 12

    m = P["m"]
    if use_default:
        print(f"Liczba silników (m): {m}")

    if use_default:
        t = P["t"]
        print(f"Czas początkowy t: {t} s")
    else:
        t = input_float("Podaj czas początkowy t [s]", P["t"])

    if use_default:
        dt = P["dt"]
        print(f"Krok czasowy dt: {dt} s")
    else:
        dt = input_float("Krok czasowy dt [s]", P["dt"])

    if use_default:
        grid_resolution = P["grid_resolution"]
        print(f"Rozdzielczość grid: {grid_resolution}")
    else:
        grid_resolution = input_float("Podaj rozdzielczość grid", P["grid_resolution"])

    return n, include_thrust_dynamics, m, t, dt, grid_resolution


def input_parameters():

    use_custom = input_yes_no("Czy chcesz wpisać własne dane [domyślne]?", False)
    use_default = not use_custom

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
