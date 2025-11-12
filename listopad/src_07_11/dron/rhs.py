import numpy as np
from config import config

def rotation_matrix_zyx(phi, theta, psi):
    """
    Compute full 3D rotation matrix using ZYX Euler angles (yaw-pitch-roll).
    Uses AEROSPACE CONVENTION for NED coordinate system.

    phi: roll angle (rotation around X)
    theta: pitch angle (rotation around Y)
        - theta > 0: nose up
        - theta < 0: nose down
    psi: yaw angle (rotation around Z)

    Returns: 3x3 rotation matrix from body frame to NED inertial frame
    """
    # Roll (X-axis rotation)
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    Rx = np.array([
        [1, 0, 0],
        [0, c_phi, -s_phi],
        [0, s_phi, c_phi]
    ])

    # Pitch (Y-axis rotation)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    Ry = np.array([
        [c_theta, 0, s_theta],
        [0, 1, 0],
        [-s_theta, 0, c_theta]
    ])

    # Yaw (Z-axis rotation)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    Rz = np.array([
        [c_psi, -s_psi, 0],
        [s_psi, c_psi, 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx

def aa_rhs(x, t, u, az_turbulence):
    """
    QUADCOPTER X CONFIGURATION - 3D DYNAMICS WITH IMPROVED AERODYNAMICS

    State vector x:
    - n = 12: [vx, vy, vz, p, q, r, X, Y, Z, φ, θ, ψ]
    - n = 16: [vx, vy, vz, p, q, r, X, Y, Z, φ, θ, ψ, T1, T2, T3, T4]

    Improvements:
    - Ground effect
    - Improved drag model
    - Induced drag from thrust
    """
    mass = config.mass
    S = config.S
    CD_0 = config.CD_0
    F_g = config.F_g
    RO_0 = config.RO_0
    IX = config.IX
    IY = config.IY
    IZ = config.IZ
    motor_arm_x = config.motor_arm_x
    motor_arm_y = config.motor_arm_y
    k_torque = config.k_torque
    C_Lp = config.C_Lp
    CM_Q = config.CM_Q
    C_Nr = config.C_Nr
    tau = config.tau
    F_g_array = config.g_ned


    ax_wind = config.ax_wind
    ay_wind = config.ay_wind
    az_wind = config.az_wind

    n = config.n
    dx_dt = np.zeros(n)

    # === STATE EXTRACTION ===
    vx = x[0]  # Forward velocity (body frame)
    vy = x[1]  # Right velocity (body frame)
    vz = x[2]  # Down velocity (body frame)
    p = x[3]  # Roll rate
    q = x[4]  # Pitch rate
    r = x[5]  # Yaw rate
    X = x[6]  # Position X (NED)
    Y = x[7]  # Position Y (NED)
    Z = x[8]  # Position Z (NED, positive = down)
    phi = x[9]  # Roll angle
    theta = x[10]  # Pitch angle
    psi = x[11]  # Yaw angle

    # Total velocity for aerodynamics
    V = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

    # Air density (varies with altitude)
    altitude = -Z  # Altitude above ground (positive upward)
    ro = RO_0 * (1.0 - altitude / 44300.0) ** 4.256
    ro = max(ro, 0.1 * RO_0)  # Safety limit minimum density

    # Dynamic pressure
    Q_dyn = 0.5 * ro * V ** 2

    # ========================================================================
    # THRUST FORCES
    # ========================================================================

    if n == 16:
        # n = 16: State includes thrust dynamics
        T1 = x[12]  # Front-right
        T2 = x[13]  # Front-left
        T3 = x[14]  # Rear-left
        T4 = x[15]  # Rear-right
    else:
        # n = 12: Direct thrust commands
        T1 = u[0]
        T2 = u[1]
        T3 = u[2]
        T4 = u[3]

    T_total = T1 + T2 + T3 + T4

    # ========================================================================
    # IMPROVED AERODYNAMIC MODEL
    # ========================================================================

    # 1. Profile drag (existing)
    if V > 0.001:
        Dx_profile = -Q_dyn * S * CD_0 * (vx / V)
        Dy_profile = -Q_dyn * S * CD_0 * (vy / V)
        Dz_profile = -Q_dyn * S * CD_0 * (vz / V)
    else:
        Dx_profile = Dy_profile = Dz_profile = 0.0

    # Total drag
    Dx = Dx_profile
    Dy = Dy_profile
    Dz = Dz_profile

    # ========================================================================
    # GRAVITY IN BODY FRAME
    # ========================================================================

    R = rotation_matrix_zyx(phi, theta, psi)
    g_body = R.T @ F_g_array  # ✅ Bezpośrednio, bez dodatkowej funkcji

    Gx = g_body[0]
    Gy = g_body[1]
    Gz = g_body[2]

    # ========================================================================
    # TRANSLATIONAL DYNAMICS (Body Frame)
    # ========================================================================

    # Coriolis terms: ω × v
    ax = (Dx + Gx) / mass + (q * vz - r * vy) + ax_wind
    ay = (Dy + Gy) / mass + (r * vx - p * vz) + ay_wind
    az = (Dz + Gz - T_total) / mass + (p * vy - q * vx) + az_turbulence + az_wind

    dx_dt[0] = ax
    dx_dt[1] = ay
    dx_dt[2] = az

    # ========================================================================
    # MOMENTS FROM THRUST
    # ========================================================================

    # Motor positions and groups
    T_front = T1 + T2  # Motors at +X
    T_rear = T3 + T4  # Motors at -X
    T_right = T1 + T4  # Motors at +Y
    T_left = T2 + T3  # Motors at -Y

    # Roll moment (about X-axis)
    L = motor_arm_y * (T_right - T_left)

    # Pitch moment (about Y-axis)
    M = motor_arm_x * (T_front - T_rear)

    # Yaw moment (about Z-axis) from propeller torques
    # Motors 1,3 rotate CW (positive torque), Motors 2,4 rotate CCW (negative torque)
    N = k_torque * ((T1 + T3) - (T2 + T4))

    # Damping moments (velocity-dependent, includes quadratic term)
    L_damp = C_Lp * p * (1 + 0.1 * abs(p))
    M_damp = CM_Q * q * (1 + 0.1 * abs(q))
    N_damp = C_Nr * r * (1 + 0.1 * abs(r))

    # ========================================================================
    # ROTATIONAL DYNAMICS (Euler equations)
    # ========================================================================

    dp_dt = (L + L_damp + (IY - IZ) * q * r) / IX
    dq_dt = (M + M_damp + (IZ - IX) * r * p) / IY
    dr_dt = (N + N_damp + (IX - IY) * p * q) / IZ

    dx_dt[3] = dp_dt
    dx_dt[4] = dq_dt
    dx_dt[5] = dr_dt

    # ========================================================================
    # POSITION UPDATE (NED frame)
    # ========================================================================

    # DCM from body to NED (3-2-1 Euler angles)
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    # Transform velocities from body to NED
    v_body = np.array([vx, vy, vz])
    v_ned = R @ v_body

    dx_dt[6] = v_ned[0]  # X_dot (North velocity)
    dx_dt[7] = v_ned[1]  # Y_dot (East velocity)
    dx_dt[8] = v_ned[2]  # Z_dot (Down velocity)

    # ========================================================================
    # EULER ANGLE RATES
    # ========================================================================

    # Transform angular velocities to Euler angle rates
    # Singularity avoidance at theta = ±90°

    if abs(np.cos(theta)) > 0.01:
        dx_dt[9] = p + sphi * np.tan(theta) * q + cphi * np.tan(theta) * r
        dx_dt[10] = cphi * q - sphi * r
        dx_dt[11] = (sphi / np.cos(theta)) * q + (cphi / np.cos(theta)) * r
    else:
        # Near singularity - use approximation
        dx_dt[9] = p
        dx_dt[10] = cphi * q - sphi * r
        dx_dt[11] = np.sign(np.cos(theta)) * r  # Keep yaw rate contribution

    # ========================================================================
    # MOTOR DYNAMICS (only for n=16)
    # ========================================================================

    if n == 16:
        # First-order lag model for 4 motors
        dx_dt[12] = (1.0 / tau) * (u[0] - x[12])
        dx_dt[13] = (1.0 / tau) * (u[1] - x[13])
        dx_dt[14] = (1.0 / tau) * (u[2] - x[14])
        dx_dt[15] = (1.0 / tau) * (u[3] - x[15])

    return dx_dt

