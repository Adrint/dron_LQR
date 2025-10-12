import numpy as np
from constants import (CD_0, RO_0, MASS, g, S, CM_Q, IY, IX, IZ,
                              TAU, MOTOR_ARM_LENGTH)  # JUŻ POPRAWNE

def quadrocopter_dynamics(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """
    Right-hand side of quadrocopter equations of motion

    State: x = [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
    Control: u = [T1, T2, T3, T4]
    """
    n = len(x)
    dx_dt = np.zeros(n)

    # Extract state
    vx, vy, vz = x[0:3]
    p, q, r = x[3:6]
    X, Y, Z = x[6:9]
    phi, theta, psi = x[9:12]

    # Get thrusts
    Thrust_1, Thrust_2, Thrust_3, Thrust_4 = u[0], u[1], u[2], u[3]

    # Wind effects
    vx_wind = vx + ax_wind
    vy_wind = vy + az_wind
    vz_wind = vz + az_wind

    # Aerodynamics
    V = np.sqrt(vx_wind ** 2 + vy_wind ** 2 + vz_wind ** 2)
    ro = RO_0 * (1.0 - abs(Z) / 44300.0) ** 4.256
    ro = max(ro, 0.0)

    if V > 0.01:
        Q_dyn = 0.5 * ro * V ** 2
        D_mag = Q_dyn * S * CD_0
        Dx = D_mag * vx_wind / V
        Dy = D_mag * vy_wind / V
        Dz = D_mag * vz_wind / V
    else:
        Dx, Dy, Dz = 0.0, 0.0, 0.0

    # Thrust
    G = MASS * g
    T_total = Thrust_1 + Thrust_2 + Thrust_3 + Thrust_4

    # Rotation matrix (body to inertial)
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    R11 = c_psi * c_theta
    R12 = c_psi * s_theta * s_phi - s_psi * c_phi
    R13 = c_psi * s_theta * c_phi + s_psi * s_phi

    R21 = s_psi * c_theta
    R22 = s_psi * s_theta * s_phi + c_psi * c_phi
    R23 = s_psi * s_theta * c_phi - c_psi * s_phi

    R31 = -s_theta
    R32 = c_theta * s_phi
    R33 = c_theta * c_phi

    # Transform thrust to inertial frame
    F_body_z = T_total
    Fx_inertial = R13 * F_body_z
    Fy_inertial = R23 * F_body_z
    Fz_inertial = R33 * F_body_z

    # Linear motion
    dx_dt[0] = (Fx_inertial - Dx) / MASS
    dx_dt[1] = (Fy_inertial - Dy) / MASS
    dx_dt[2] = (Fz_inertial - G - Dz) / MASS + az_turbulence

    # Rotational dynamics
    arm = MOTOR_ARM_LENGTH / np.sqrt(2)
    M_roll = (Thrust_3 + Thrust_4 - Thrust_1 - Thrust_2) * arm
    M_pitch = (Thrust_2 + Thrust_4 - Thrust_1 - Thrust_3) * arm
    M_yaw = (Thrust_1 + Thrust_3 - Thrust_2 - Thrust_4) * 0.1

    dx_dt[3] = (M_roll + CM_Q * p) / IX
    dx_dt[4] = (M_pitch + CM_Q * q) / IY
    dx_dt[5] = (M_yaw + CM_Q * r) / IZ

    # Position
    dx_dt[6] = vx
    dx_dt[7] = vy
    dx_dt[8] = vz

    # Attitude kinematics
    tan_theta = np.tan(theta)
    cos_theta = np.cos(theta)
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)

    if abs(cos_theta) > 0.01:
        dx_dt[9] = p + q * sin_phi * tan_theta + r * cos_phi * tan_theta
        dx_dt[10] = q * cos_phi - r * sin_phi
        dx_dt[11] = (q * sin_phi + r * cos_phi) / cos_theta
    else:
        dx_dt[9] = p
        dx_dt[10] = q * cos_phi - r * sin_phi
        dx_dt[11] = r

    return dx_dt