"""
DRONE PHYSICAL LIMITS - Based on Real Quadcopter Specifications
All limits are enforced during simulation to ensure realistic behavior
"""

import numpy as np

# ============================================================================
# ANGULAR VELOCITY LIMITS
# ============================================================================

# Maximum angular velocities [rad/s]
MAX_PITCH_RATE = np.deg2rad(300)  # 300°/s - forward/backward tilt rate
MAX_ROLL_RATE = np.deg2rad(300)   # 300°/s - left/right tilt rate
MAX_YAW_RATE = np.deg2rad(100)    # 100°/s - rotation rate

# ============================================================================
# ATTITUDE LIMITS
# ============================================================================

# Maximum tilt angles [rad]
MAX_TILT_ANGLE_P = np.deg2rad(25) # 25° - maximum tilt (P mode with vision system)

# Use P mode limit by default (safer)
MAX_ROLL_ANGLE = MAX_TILT_ANGLE_P
MAX_PITCH_ANGLE = MAX_TILT_ANGLE_P

# Yaw can rotate freely (360°)
MAX_YAW_ANGLE = np.deg2rad(180)   # ±180° for normalization

# ============================================================================
# LINEAR VELOCITY LIMITS
# ============================================================================

# Maximum horizontal velocity [m/s]
MAX_HORIZONTAL_VELOCITY = 23.0  # 23 m/s (~82.8 km/h)
MAX_ASCENT_RATE = 6.0    # 6 m/s upward (vz = -6 in NED)
MAX_DESCENT_RATE = 5.0   # 5 m/s downward (vz = +5 in NED) - vertical
MAX_DESCENT_RATE_TILTED = 7.0  # 7 m/s downward when tilted

# ============================================================================
# COMBINED VELOCITY LIMITS
# ============================================================================

# Maximum total velocity (magnitude)
MAX_TOTAL_VELOCITY = MAX_HORIZONTAL_VELOCITY

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def limit_angular_velocities(p, q, r):
    """
    Limit angular velocities to physical constraints
    
    Parameters:
    -----------
    p : float
        Roll rate [rad/s]
    q : float
        Pitch rate [rad/s]
    r : float
        Yaw rate [rad/s]
    
    Returns:
    --------
    p_limited, q_limited, r_limited : float
        Limited angular velocities
    """
    p_limited = np.clip(p, -MAX_ROLL_RATE, MAX_ROLL_RATE)
    q_limited = np.clip(q, -MAX_PITCH_RATE, MAX_PITCH_RATE)
    r_limited = np.clip(r, -MAX_YAW_RATE, MAX_YAW_RATE)
    
    return p_limited, q_limited, r_limited


def limit_angles(phi, theta, psi):
    """
    Limit attitude angles to physical constraints
    
    Parameters:
    -----------
    phi : float
        Roll angle [rad]
    theta : float
        Pitch angle [rad]
    psi : float
        Yaw angle [rad]
    
    Returns:
    --------
    phi_limited, theta_limited, psi_limited : float
        Limited angles
    """
    phi_limited = np.clip(phi, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE)
    theta_limited = np.clip(theta, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE)
    
    # Normalize yaw to [-π, π]
    psi_limited = np.arctan2(np.sin(psi), np.cos(psi))
    
    return phi_limited, theta_limited, psi_limited


def limit_linear_velocities(vx, vy, vz, phi=0.0, theta=0.0):
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
    # Horizontal velocity limit (in body frame XY plane)
    v_horizontal = np.sqrt(vx**2 + vy**2)
    
    if v_horizontal > MAX_HORIZONTAL_VELOCITY:
        scale = MAX_HORIZONTAL_VELOCITY / v_horizontal
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
        vz_limited = np.clip(vz, -MAX_ASCENT_RATE, MAX_DESCENT_RATE_TILTED)
    else:
        # Normal vertical limits
        vz_limited = np.clip(vz, -MAX_ASCENT_RATE, MAX_DESCENT_RATE)
    
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


def check_limits_exceeded(x, verbose=False):
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
    violations = []
    
    vx, vy, vz = x[0], x[1], x[2]
    p, q, r = x[3], x[4], x[5]
    phi, theta, psi = x[9], x[10], x[11]
    
    # Check angular velocities
    if abs(p) > MAX_ROLL_RATE:
        violations.append(f"Roll rate exceeded: {np.rad2deg(abs(p)):.1f}°/s > {np.rad2deg(MAX_ROLL_RATE):.1f}°/s")
    
    if abs(q) > MAX_PITCH_RATE:
        violations.append(f"Pitch rate exceeded: {np.rad2deg(abs(q)):.1f}°/s > {np.rad2deg(MAX_PITCH_RATE):.1f}°/s")
    
    if abs(r) > MAX_YAW_RATE:
        violations.append(f"Yaw rate exceeded: {np.rad2deg(abs(r)):.1f}°/s > {np.rad2deg(MAX_YAW_RATE):.1f}°/s")
    
    # Check angles
    if abs(phi) > MAX_ROLL_ANGLE:
        violations.append(f"Roll angle exceeded: {np.rad2deg(abs(phi)):.1f}° > {np.rad2deg(MAX_ROLL_ANGLE):.1f}°")
    
    if abs(theta) > MAX_PITCH_ANGLE:
        violations.append(f"Pitch angle exceeded: {np.rad2deg(abs(theta)):.1f}° > {np.rad2deg(MAX_PITCH_ANGLE):.1f}°")
    
    # Check velocities
    v_horizontal = np.sqrt(vx**2 + vy**2)
    if v_horizontal > MAX_HORIZONTAL_VELOCITY:
        violations.append(f"Horizontal velocity exceeded: {v_horizontal:.1f} m/s > {MAX_HORIZONTAL_VELOCITY:.1f} m/s")
    
    if vz < -MAX_ASCENT_RATE:
        violations.append(f"Ascent rate exceeded: {abs(vz):.1f} m/s > {MAX_ASCENT_RATE:.1f} m/s")
    
    tilt_magnitude = np.sqrt(phi**2 + theta**2)
    is_tilted = tilt_magnitude > np.deg2rad(5)
    max_descent = MAX_DESCENT_RATE_TILTED if is_tilted else MAX_DESCENT_RATE
    
    if vz > max_descent:
        violations.append(f"Descent rate exceeded: {vz:.1f} m/s > {max_descent:.1f} m/s")
    
    if verbose and violations:
        print("\n⚠️  PHYSICAL LIMITS EXCEEDED:")
        for v in violations:
            print(f"   {v}")
    
    return len(violations) > 0, violations


# ============================================================================
# PRINT LIMITS SUMMARY
# ============================================================================

def print_limits():
    """Print summary of all physical limits"""
    print("="*70)
    print("DRONE PHYSICAL LIMITS")
    print("="*70)
    print("\nANGULAR VELOCITY LIMITS:")
    print(f"  Max pitch rate:  ±{np.rad2deg(MAX_PITCH_RATE):.0f}°/s")
    print(f"  Max roll rate:   ±{np.rad2deg(MAX_ROLL_RATE):.0f}°/s")
    print(f"  Max yaw rate:    ±{np.rad2deg(MAX_YAW_RATE):.0f}°/s")
    
    print("\nATTITUDE LIMITS:")
    print(f"  Max pitch angle: ±{np.rad2deg(MAX_PITCH_ANGLE):.0f}°")
    print(f"  Max roll angle:  ±{np.rad2deg(MAX_ROLL_ANGLE):.0f}°")
    
    print("\nLINEAR VELOCITY LIMITS:")
    print(f"  Max horizontal velocity:  {MAX_HORIZONTAL_VELOCITY:.0f} m/s ({MAX_HORIZONTAL_VELOCITY*3.6:.1f} km/h)")
    print(f"  Max ascent rate:          {MAX_ASCENT_RATE:.0f} m/s")
    print(f"  Max descent rate (vert):  {MAX_DESCENT_RATE:.0f} m/s")
    print(f"  Max descent rate (tilt):  {MAX_DESCENT_RATE_TILTED:.0f} m/s")
    print("="*70)


if __name__ == "__main__":
    # Test limits
    print_limits()
    
    # Test limit functions
    print("\n\nTEST 1: Angular velocity limiting")
    p, q, r = np.deg2rad(400), np.deg2rad(-350), np.deg2rad(150)
    print(f"Input:  p={np.rad2deg(p):.0f}°/s, q={np.rad2deg(q):.0f}°/s, r={np.rad2deg(r):.0f}°/s")
    p_lim, q_lim, r_lim = limit_angular_velocities(p, q, r)
    print(f"Output: p={np.rad2deg(p_lim):.0f}°/s, q={np.rad2deg(q_lim):.0f}°/s, r={np.rad2deg(r_lim):.0f}°/s")
    
    print("\n\nTEST 2: Angle limiting")
    phi, theta, psi = np.deg2rad(35), np.deg2rad(-40), np.deg2rad(270)
    print(f"Input:  φ={np.rad2deg(phi):.0f}°, θ={np.rad2deg(theta):.0f}°, ψ={np.rad2deg(psi):.0f}°")
    phi_lim, theta_lim, psi_lim = limit_angles(phi, theta, psi)
    print(f"Output: φ={np.rad2deg(phi_lim):.0f}°, θ={np.rad2deg(theta_lim):.0f}°, ψ={np.rad2deg(psi_lim):.0f}°")
    
    print("\n\nTEST 3: Velocity limiting")
    vx, vy, vz = 25.0, 10.0, -8.0
    print(f"Input:  vx={vx:.1f} m/s, vy={vy:.1f} m/s, vz={vz:.1f} m/s")
    vx_lim, vy_lim, vz_lim = limit_linear_velocities(vx, vy, vz)
    print(f"Output: vx={vx_lim:.1f} m/s, vy={vy_lim:.1f} m/s, vz={vz_lim:.1f} m/s")
    print(f"Horizontal speed: {np.sqrt(vx_lim**2 + vy_lim**2):.1f} m/s")
