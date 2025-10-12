import numpy as np
from constants import (CD_0, RO_0, MASS, g, S, CM_Q, IY, IX, IZ, TAU, MOTOR_ARM_LENGTH)


def aa_rhs(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """
    Right-hand side of the quadrocopter differential equations

    QUADROCOPTER CONFIGURATION (X-configuration, 45 degrees):
        F1 (front-left)      F2 (front-right)
            ↓                    ↓
             \                  /
              \       CG       /
              /                \
             /                  \
            ↓                    ↓
        F3 (rear-left)       F4 (rear-right)

    State vector x:
    - n=12: [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
    - n=16: [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi, T1, T2, T3, T4]

    where:
    vx, vy, vz = linear velocities in inertial frame
    p, q, r = angular velocities (roll, pitch, yaw rates)
    X, Y, Z = position
    phi, theta, psi = Euler angles (roll, pitch, yaw)
    T1, T2, T3, T4 = actual thrust states [N] (only for n=16)

    Control input u:
    u = [T1_cmd, T2_cmd, T3_cmd, T4_cmd] = thrust command signals [N]
    """
    n = len(x)
    dx_dt = np.zeros(n)

    # Extract state (common for both n=12 and n=16)
    vx = x[0]
    vy = x[1]
    vz = x[2]
    p = x[3]  # roll rate
    q = x[4]  # pitch rate
    r = x[5]  # yaw rate
    X = x[6]
    Y = x[7]
    Z = x[8]
    phi = x[9]  # roll angle
    theta = x[10]  # pitch angle
    psi = x[11]  # yaw angle

    # Get actual thrusts depending on state dimension
    if n == 16:
        # Thrusts are state variables
        Thrust_1 = x[12]
        Thrust_2 = x[13]
        Thrust_3 = x[14]
        Thrust_4 = x[15]
    else:
        # n=12: Direct thrust from control input (no motor dynamics)
        Thrust_1 = u[0]
        Thrust_2 = u[1]
        Thrust_3 = u[2]
        Thrust_4 = u[3]

    # Add wind effects
    vx_wind = vx + ax_wind
    vy_wind = vy + az_wind  # Note: using az_wind as side wind for now
    vz_wind = vz + az_wind

    # === AERODYNAMICS ===
    V = np.sqrt(vx_wind ** 2 + vy_wind ** 2 + vz_wind ** 2)

    # Air density variation with altitude
    ro = RO_0 * (1.0 - abs(Z) / 44300.0) ** 4.256
    if ro < 0:
        ro = 0.0

    # Drag calculation (simple linear drag)
    if V > 0.01:
        Q_dyn = 0.5 * ro * V ** 2
        D_mag = Q_dyn * S * CD_0

        Dx = D_mag * vx_wind / V
        Dy = D_mag * vy_wind / V
        Dz = D_mag * vz_wind / V
    else:
        Dx = 0.0
        Dy = 0.0
        Dz = 0.0

    # === THRUST FORCES ===
    G = MASS * g

    T_total = Thrust_1 + Thrust_2 + Thrust_3 + Thrust_4

    # === ROTATIONAL MATRIX (Body to Inertial) ===
    # This transforms forces from body frame to inertial frame
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    # Rotation matrix elements
    R11 = c_psi * c_theta
    R12 = c_psi * s_theta * s_phi - s_psi * c_phi
    R13 = c_psi * s_theta * c_phi + s_psi * s_phi

    R21 = s_psi * c_theta
    R22 = s_psi * s_theta * s_phi + c_psi * c_phi
    R23 = s_psi * s_theta * c_phi - c_psi * s_phi

    R31 = -s_theta
    R32 = c_theta * s_phi
    R33 = c_theta * c_phi

    # Thrust force in body frame (all upward in Z_body direction)
    F_body_x = 0.0
    F_body_y = 0.0
    F_body_z = T_total

    # Transform to inertial frame
    Fx_inertial = R11 * F_body_x + R12 * F_body_y + R13 * F_body_z
    Fy_inertial = R21 * F_body_x + R22 * F_body_y + R23 * F_body_z
    Fz_inertial = R31 * F_body_x + R32 * F_body_y + R33 * F_body_z

    # === EQUATIONS OF MOTION (Inertial Frame) ===
    # Linear motion
    dx_dt[0] = (Fx_inertial - Dx) / MASS
    dx_dt[1] = (Fy_inertial - Dy) / MASS
    dx_dt[2] = (Fz_inertial - G - Dz) / MASS + az_turbulence

    # === ROTATIONAL DYNAMICS ===
    # Calculate moments from differential thrusts
    # Configuration X (45 degrees):
    # Moment arm for each motor: d = MOTOR_ARM_LENGTH / sqrt(2)
    arm = MOTOR_ARM_LENGTH / np.sqrt(2)

    # Roll moment (around X-axis):
    # M_roll = (T3 + T4 - T1 - T2) * arm
    M_roll = (Thrust_3 + Thrust_4 - Thrust_1 - Thrust_2) * arm

    # Pitch moment (around Y-axis):
    # M_pitch = (T2 + T4 - T1 - T3) * arm
    M_pitch = (Thrust_2 + Thrust_4 - Thrust_1 - Thrust_3) * arm

    # Yaw moment (around Z-axis):
    # Counter-rotating pairs: (T1 + T3 vs T2 + T4)
    M_yaw = (Thrust_1 + Thrust_3 - Thrust_2 - Thrust_4) * 0.1  # scaled for realistic yaw

    # Angular accelerations (with damping)
    dp_dt = (M_roll + CM_Q * p) / IX
    dq_dt = (M_pitch + CM_Q * q) / IY
    dr_dt = (M_yaw + CM_Q * r) / IZ

    dx_dt[3] = dp_dt
    dx_dt[4] = dq_dt
    dx_dt[5] = dr_dt

    # === POSITION UPDATES (Inertial Frame) ===
    dx_dt[6] = vx
    dx_dt[7] = vy
    dx_dt[8] = vz

    # === ATTITUDE KINEMATICS (Euler angles) ===
    tan_theta = np.tan(theta)
    cos_theta = np.cos(theta)
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)

    if abs(cos_theta) > 0.01:  # Avoid singularity
        dx_dt[9] = p + q * sin_phi * tan_theta + r * cos_phi * tan_theta
        dx_dt[10] = q * cos_phi - r * sin_phi
        dx_dt[11] = (q * sin_phi + r * cos_phi) / cos_theta
    else:
        # Gimbal lock handling
        dx_dt[9] = p
        dx_dt[10] = q * cos_phi - r * sin_phi
        dx_dt[11] = r

    # === MOTOR THRUST DYNAMICS (only for n=16) ===
    if n == 16:
        # First-order dynamics for each motor: dT/dt = (T_cmd - T) / TAU
        dx_dt[12] = (u[0] - Thrust_1) / TAU
        dx_dt[13] = (u[1] - Thrust_2) / TAU
        dx_dt[14] = (u[2] - Thrust_3) / TAU
        dx_dt[15] = (u[3] - Thrust_4) / TAU

    return dx_dt