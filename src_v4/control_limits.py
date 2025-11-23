import numpy as np
from config import config


def limit_angular_velocities(p, q, r, phi, theta, psi):
    """Parametry:
        p, q, r : float
            Prędkości kątowe wokół osi X/Y/Z (rad/s)
        phi, theta, psi : float
            Aktualne kąty przechylenia, pochylenia i odchylenia (rad)

    Zwraca:
        (p_lim, q_lim, r_lim): tuple
            Ograniczone prędkości kątowe
    """
    max_rates = np.array([
        config.max_roll_rate_rad,
        config.max_pitch_rate_rad,
        config.max_yaw_rate_rad
    ])

    max_angles = np.array([
        config.max_phi_angle_rad,
        config.max_theta_angle_rad,
        config.max_psi_angle_rad
    ])

    rates = np.array([p, q, r])
    angles = np.array([phi, theta, psi])

    # Podstawowe ograniczenie prędkości kątowych
    rates_limited = np.clip(rates, -max_rates, max_rates)

    # Miękkie nasycenie w pobliżu limitów kątowych
    margin = 0.087  # ~5 stopni strefa przejściowa

    for i in range(3):
        # Sprawdzanie, czy zbliżamy się do limitów kątowych
        if angles[i] > max_angles[i] - margin and rates_limited[i] > 0:
            # Zbliżanie się do górnego limitu przy dodatniej prędkości
            scale = np.clip((max_angles[i] - angles[i]) / margin, 0.0, 1.0)
            rates_limited[i] *= scale

        elif angles[i] < -max_angles[i] + margin and rates_limited[i] < 0:
            # Zbliżanie się do dolnego limitu przy ujemnej prędkości
            scale = np.clip((max_angles[i] + angles[i]) / margin, 0.0, 1.0)
            rates_limited[i] *= scale

    return rates_limited[0], rates_limited[1], rates_limited[2]


def limit_angles(phi, theta, psi):
    """Ogranicza kąty do wartości fizycznie dopuszczalnych."""
    phi_lim = np.clip(phi, -config.max_phi_angle_rad, config.max_phi_angle_rad)
    theta_lim = np.clip(theta, -config.max_theta_angle_rad, config.max_theta_angle_rad)
    psi_lim = np.clip(psi, -config.max_psi_angle_rad, config.max_psi_angle_rad)
    return phi_lim, theta_lim, psi_lim


def limit_linear_velocities(vx, vy, vz, phi, theta):
    """
    Parametry:
        vx, vy, vz: prędkości w układzie ciała [m/s]
        phi, theta: kąty przechylenia i pochylenia [rad]

    Zwraca:
        Ograniczone prędkości
    """

    # Ograniczenie prędkości poziomej
    v_horizontal = np.sqrt(vx ** 2 + vy ** 2)

    if v_horizontal > config.max_horizontal_vel:
        scale = config.max_horizontal_vel / v_horizontal
        vx_limited = vx * scale
        vy_limited = vy * scale
    else:
        vx_limited = vx
        vy_limited = vy

    # Ograniczenia prędkości pionowej (NED: dodatnie vz = w dół)
    # Sprawdzanie przechylenia dla wyboru maks. prędkości opadania
    tilt_magnitude = np.sqrt(phi ** 2 + theta ** 2)
    is_tilted = tilt_magnitude > np.deg2rad(5)

    # Wybór odpowiedniego limitu opadania
    max_desc = config.max_descent_tilted if is_tilted else config.max_descent
    vz_limited = np.clip(vz, -config.max_ascent, max_desc)

    return vx_limited, vy_limited, vz_limited


def apply_all_limits(x, z_ground=None, protection_config=None):
    """
    Parametry:
        x: wektor stanu [vx, vy, vz, p, q, r, X, Y, Z, φ, θ, ψ, ...]
        z_ground: bezwzględny poziom gruntu w układzie NED (Z=0 standardowo)
        protection_config: słownik zawierający:
            - 'z_start': wysokość początkowa
            - 'z_initial_protection': limit strefy ochronnej
            - 'initial_protection_active': czy ochrona początkowa jest aktywna

    Zwraca:
        Ograniczony wektor stanu
    """
    x_limited = x.copy()

    # Ograniczenia prędkości liniowych
    x_limited[0:3] = limit_linear_velocities(x[0], x[1], x[2], x[9], x[10])

    # Ograniczenia prędkości kątowych (uwzględniające bieżące kąty)
    x_limited[3:6] = limit_angular_velocities(x[3], x[4], x[5], x[9], x[10], x[11])

    # Ograniczenia kątów
    x_limited[9:12] = limit_angles(x[9], x[10], x[11])

    # Zapobieganie kolizji z ziemią – logika wielopoziomowa
    if z_ground is not None:
        # PRIORYTET 1: bezwzględny poziom gruntu (Z=0) – nigdy nie można spaść poniżej
        if x_limited[8] > z_ground:
            x_limited[8] = z_ground
            if x_limited[2] > 0:  # zatrzymanie prędkości opadania
                x_limited[2] = 0.0

        # PRIORYTET 2: strefa ochrony początkowej (pierwsze ~10 m lotu)
        elif protection_config is not None and protection_config.get('initial_protection_active', False):
            z_initial_protection = protection_config['z_initial_protection']

            # Jeśli dron próbuje spaść poniżej strefy ochronnej
            if x_limited[8] > z_initial_protection:
                x_limited[8] = z_initial_protection
                if x_limited[2] > 0:  # zatrzymanie opadania
                    x_limited[2] = 0.0

    return x_limited
