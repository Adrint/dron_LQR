import numpy as np
from config import config

def limit_angular_velocities(p, q, r):
    """
    Parameters:
    -----------
    p : float
        Roll rate [rad/s]
    q : float
        Pitch rate [rad/s]
    r : float
        Yaw rate [rad/s]

    """
    max_roll_rate_rad = config.max_roll_rate_rad
    max_pitch_rate_rad = config.max_pitch_rate_rad
    max_yaw_rate_rad = config.max_yaw_rate_rad

    p_limited = np.clip(p, -max_roll_rate_rad, max_roll_rate_rad)
    q_limited = np.clip(q, -max_pitch_rate_rad, max_pitch_rate_rad)
    r_limited = np.clip(r, -max_yaw_rate_rad, max_yaw_rate_rad)
    
    return p_limited, q_limited, r_limited

def limit_angles(phi, theta, psi):
    """
    
    phi : float
        Roll angle [rad]
    theta : float
        Pitch angle [rad]
    psi : float
        Yaw angle [rad]
   
    """
    max_phi_angle_rad = config.max_phi_angle_rad
    max_theta_angle_rad = config.max_theta_angle_rad
    max_psi_angle_rad = config.max_psi_angle_rad

    phi_limited = np.clip(phi, -max_phi_angle_rad, max_phi_angle_rad)
    theta_limited = np.clip(theta, -max_theta_angle_rad, max_theta_angle_rad)
    psi_limited = np.clip(psi, -max_psi_angle_rad, max_psi_angle_rad)
    
    
    return phi_limited, theta_limited, psi_limited

def limit_linear_velocities(vx, vy, vz, phi, theta):
    """
    Limit linear velocities to physical constraints
    
    Parameters:
    -----------
    vx, vy, vz : float
        Velocities in body frame [m/s]
    phi, theta : float
        Current roll and pitch angles [rad] (for descent rate calculation)
    
    Returns:
    --------
    vx_limited, vy_limited, vz_limited : float
        Limited velocities
    """
    max_horizontal_vel = config.max_horizontal_vel
    max_ascent = config.max_ascent
    max_descent = config.max_descent
    max_descent_tilted = config.max_descent_tilted

    # Horizontal velocity limit (in body frame XY plane)
    v_horizontal = np.sqrt(vx**2 + vy**2)
    
    if v_horizontal > max_horizontal_vel:
        scale = max_horizontal_vel / v_horizontal
        vx_limited = vx * scale
        vy_limited = vy * scale
    else:
        vx_limited = vx
        vy_limited = vy
    
    # Vertical velocity limits (NED: positive vz = downward)
    # Ascent: vz negative (upward motion)
    # Descent: vz positive (downward motion)
    
    # Check if drone is tilted (for descent rate selection)
    tilt_magnitude = np.sqrt(phi**2 + theta**2)
    is_tilted = tilt_magnitude > np.deg2rad(5)  # Consider tilted if >5°
    
    if is_tilted:
        # Allow faster descent when tilted
        vz_limited = np.clip(vz, -max_ascent, max_descent_tilted)
    else:
        # Normal vertical limits
        vz_limited = np.clip(vz, -max_ascent, max_descent)
    
    return vx_limited, vy_limited, vz_limited

def apply_all_limits(x):
    """
    Apply all physical limits to state vector
    
    Parameters:
    -----------
    x : ndarray
        State vector [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi, ...]
    
    Returns:
    --------
    x_limited : ndarray
        State vector with limits applied
    """
    x_limited = x.copy()
    
    # Extract current state
    vx, vy, vz = x[0], x[1], x[2]
    p, q, r = x[3], x[4], x[5]
    phi, theta, psi = x[9], x[10], x[11]
    
    # Apply limits

    vx_lim, vy_lim, vz_lim = limit_linear_velocities(vx, vy, vz, phi, theta)
    p_lim, q_lim, r_lim = limit_angular_velocities(p, q, r)
    phi_lim, theta_lim, psi_lim = limit_angles(phi, theta, psi)
    
    # Update state
    x_limited[0], x_limited[1], x_limited[2] = vx_lim, vy_lim, vz_lim
    x_limited[3], x_limited[4], x_limited[5] = p_lim, q_lim, r_lim
    x_limited[9], x_limited[10], x_limited[11] = phi_lim, theta_lim, psi_lim
    
    return x_limited

def check_limits_exceeded(x, verbose = False):
    """
    Check if any limits are being exceeded
    
    Parameters:
    -----------
    x : ndarray
        State vector
    verbose : bool
        Print warnings for exceeded limits
    
    Returns:
    --------
    exceeded : bool
        True if any limit is exceeded
    violations : list
        List of violated limit descriptions
    """
    max_roll_rate_rad = config.max_roll_rate_rad
    max_pitch_rate_rad = config.max_pitch_rate_rad
    max_yaw_rate_rad = config.max_yaw_rate_rad
    max_horizontal_vel = config.max_horizontal_vel
    max_ascent = config.max_ascent
    max_descent = config.max_descent
    max_descent_tilted = config.max_descent_tilted
    max_phi_angle_rad = config.max_phi_angle_rad
    max_theta_angle_rad = config.max_theta_angle_rad
    max_psi_angle_rad = config.max_psi_angle_rad

    violations = []
    
    vx, vy, vz = x[0], x[1], x[2]
    p, q, r = x[3], x[4], x[5]
    phi, theta, psi = x[9], x[10], x[11]
    
    # Check angular velocities
    if abs(p) > max_roll_rate_rad:
        violations.append(f"Roll rate exceeded: {np.rad2deg(abs(p)):.1f}°/s > {np.rad2deg(max_roll_rate_rad):.1f}°/s")
    
    if abs(q) > max_pitch_rate_rad:
        violations.append(f"Pitch rate exceeded: {np.rad2deg(abs(q)):.1f}°/s > {np.rad2deg(max_pitch_rate_rad):.1f}°/s")
    
    if abs(r) > max_yaw_rate_rad:
        violations.append(f"Yaw rate exceeded: {np.rad2deg(abs(r)):.1f}°/s > {np.rad2deg(max_yaw_rate_rad):.1f}°/s")
    
    # Check angles
    if abs(phi) > max_phi_angle_rad:
        violations.append(f"Roll angle exceeded: {np.rad2deg(abs(phi)):.1f}° > {np.rad2deg(max_phi_angle_rad):.1f}°")
    
    if abs(theta) > max_theta_angle_rad:
        violations.append(f"Pitch angle exceeded: {np.rad2deg(abs(theta)):.1f}° > {np.rad2deg(max_theta_angle_rad):.1f}°")
    
    # Check velocities
    v_horizontal = np.sqrt(vx**2 + vy**2)
    if v_horizontal > max_horizontal_vel:
        violations.append(f"Horizontal velocity exceeded: {v_horizontal:.1f} m/s > {max_horizontal_vel:.1f} m/s")
    
    if vz < -max_ascent:
        violations.append(f"Ascent rate exceeded: {abs(vz):.1f} m/s > {max_ascent:.1f} m/s")
    
    tilt_magnitude = np.sqrt(phi**2 + theta**2)
    is_tilted = tilt_magnitude > np.deg2rad(5)
    max_descent = max_descent_tilted if is_tilted else max_descent
    
    if vz > max_descent:
        violations.append(f"Descent rate exceeded: {vz:.1f} m/s > {max_descent:.1f} m/s")
    
    if verbose and violations:
        print("\n⚠️  PHYSICAL LIMITS EXCEEDED:")
        for v in violations:
            print(f"   {v}")
    
    return len(violations) > 0, violations


