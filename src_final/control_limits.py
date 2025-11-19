import numpy as np
from config import config


def limit_angular_velocities(p, q, r, phi, theta, psi):
    """
    Parameters:
        p, q, r: angular rates [rad/s]
        phi, theta, psi: current angles [rad]

    Returns:
        Limited angular velocities
    """
    # Get limits from config
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

    # Basic rate limiting
    rates_limited = np.clip(rates, -max_rates, max_rates)

    # Soft saturation near angle limits
    margin = 0.087  # ~5 degrees transition zone

    for i in range(3):
        # Check if approaching limits
        if angles[i] > max_angles[i] - margin and rates_limited[i] > 0:
            # Approaching upper limit while rate is positive
            scale = np.clip((max_angles[i] - angles[i]) / margin, 0.0, 1.0)
            rates_limited[i] *= scale

        elif angles[i] < -max_angles[i] + margin and rates_limited[i] < 0:
            # Approaching lower limit while rate is negative
            scale = np.clip((max_angles[i] + angles[i]) / margin, 0.0, 1.0)
            rates_limited[i] *= scale

    return rates_limited[0], rates_limited[1], rates_limited[2]


def limit_angles(phi, theta, psi):
    """Limit angles to physical constraints."""
    phi_lim = np.clip(phi, -config.max_phi_angle_rad, config.max_phi_angle_rad)
    theta_lim = np.clip(theta, -config.max_theta_angle_rad, config.max_theta_angle_rad)
    psi_lim = np.clip(psi, -config.max_psi_angle_rad, config.max_psi_angle_rad)
    return phi_lim, theta_lim, psi_lim


def limit_linear_velocities(vx, vy, vz, phi, theta):
    """
    Parameters:
        vx, vy, vz: velocities in body frame [m/s]
        phi, theta: roll and pitch angles [rad]

    Returns:
        Limited velocities
    """
    # Horizontal velocity limit
    v_horizontal = np.sqrt(vx ** 2 + vy ** 2)

    if v_horizontal > config.max_horizontal_vel:
        scale = config.max_horizontal_vel / v_horizontal
        vx_limited = vx * scale
        vy_limited = vy * scale
    else:
        vx_limited = vx
        vy_limited = vy

    # Vertical velocity limits (NED: positive vz = downward)
    # Check tilt for descent rate selection
    tilt_magnitude = np.sqrt(phi ** 2 + theta ** 2)
    is_tilted = tilt_magnitude > np.deg2rad(5)

    # Select appropriate descent limit
    max_desc = config.max_descent_tilted if is_tilted else config.max_descent
    vz_limited = np.clip(vz, -config.max_ascent, max_desc)

    return vx_limited, vy_limited, vz_limited


def apply_all_limits(x, z_ground=None, protection_config=None):
    """
    Parameters:
        x: state vector [vx, vy, vz, p, q, r, X, Y, Z, φ, θ, ψ, ...]
        z_ground: absolute ground level in NED coordinates (Z=0 typically)
        protection_config: dict with keys:
            - 'z_start': initial altitude
            - 'z_initial_protection': initial protection zone limit
            - 'initial_protection_active': whether initial protection is still active

    Returns:
        Limited state vector
    """
    x_limited = x.copy()

    # Apply velocity limits
    x_limited[0:3] = limit_linear_velocities(x[0], x[1], x[2], x[9], x[10])

    # Apply angular rate limits (considering current angles)
    x_limited[3:6] = limit_angular_velocities(x[3], x[4], x[5], x[9], x[10], x[11])

    # Apply angle limits
    x_limited[9:12] = limit_angles(x[9], x[10], x[11])

    # Ground collision prevention with multi-level protection
    if z_ground is not None:
        # PRIORITY 1: Absolute ground level (Z=0) - NEVER go below
        if x_limited[8] > z_ground:
            x_limited[8] = z_ground
            if x_limited[2] > 0:  # Stop downward velocity
                x_limited[2] = 0.0

        # PRIORITY 2: Initial protection zone (first 10m from start)
        elif protection_config is not None and protection_config.get('initial_protection_active', False):
            z_initial_protection = protection_config['z_initial_protection']

            # If drone tries to fall below initial protection zone
            if x_limited[8] > z_initial_protection:
                x_limited[8] = z_initial_protection
                if x_limited[2] > 0:  # Stop downward velocity
                    x_limited[2] = 0.0

    return x_limited