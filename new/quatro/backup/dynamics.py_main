import numpy as np
from constants import (CD_0, RO_0, MASS, g, S, CM_Q, IX, IY, IZ,
                       TAU, MOTOR_ARM_LENGTH)


def quadrocopter_dynamics_simple(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """
    Simplified quadrocopter dynamics similar to bicopter approach
    
    State: x = [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
    Control: u = [T1, T2, T3, T4] (motor thrusts in Newtons)
    
    Quadrocopter X-configuration:
         1 (front-right)
        / \\
       /   \\
      4     2 (back-right, front-left)
       \\   /
        \\ /
         3 (back-left)
    """
    n = len(x)
    dx_dt = np.zeros(n)

    # Extract state
    vx = x[0]
    vy = x[1]
    vz = x[2]
    p = x[3]   # roll rate
    q = x[4]   # pitch rate
    r = x[5]   # yaw rate
    X = x[6]
    Y = x[7]
    Z = x[8]
    phi = x[9]     # roll angle
    theta = x[10]  # pitch angle
    psi = x[11]    # yaw angle

    # Wind effects (inertial frame)
    vx_wind = vx + ax_wind
    vy_wind = vy
    vz_wind = vz + az_wind

    # === AERODYNAMICS ===
    V = np.sqrt(vx_wind ** 2 + vy_wind ** 2 + vz_wind ** 2)

    # Air density variation with altitude
    ro = RO_0 * (1.0 - abs(Z) / 44300.0) ** 4.256
    if ro < 0:
        ro = 0.0

    # Drag calculation
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
    
    # Extract motor thrusts
    Thrust_1, Thrust_2, Thrust_3, Thrust_4 = u[0], u[1], u[2], u[3]
    T_total = Thrust_1 + Thrust_2 + Thrust_3 + Thrust_4

    # === ROTATION MATRIX (Body to Inertial) ===
    # Using ZYX Euler angles (yaw-pitch-roll)
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)

    # Rotation matrix elements (body Z-axis in inertial frame)
    R13 = c_psi * s_theta * c_phi + s_psi * s_phi
    R23 = s_psi * s_theta * c_phi - c_psi * s_phi
    R33 = c_theta * c_phi

    # Transform thrust from body frame to inertial frame
    # In body frame: thrust acts along positive Z-axis (upward)
    Fx_inertial = R13 * T_total
    Fy_inertial = R23 * T_total
    Fz_inertial = R33 * T_total

    # === LINEAR ACCELERATION (Inertial Frame) ===
    dx_dt[0] = (Fx_inertial - Dx) / MASS
    dx_dt[1] = (Fy_inertial - Dy) / MASS
    dx_dt[2] = (Fz_inertial - G - Dz) / MASS + az_turbulence

    # === ROTATIONAL DYNAMICS ===
    # Motor positions in X-configuration (normalized coordinates)
    # arm = MOTOR_ARM_LENGTH / sqrt(2) for X-config
    arm = MOTOR_ARM_LENGTH / np.sqrt(2)
    
    # Moments from differential thrust
    # Roll moment (around X-axis)
    M_roll = (Thrust_3 + Thrust_4 - Thrust_1 - Thrust_2) * arm
    
    # Pitch moment (around Y-axis)
    M_pitch = (Thrust_2 + Thrust_4 - Thrust_1 - Thrust_3) * arm
    
    # Yaw moment (around Z-axis) - from reaction torques
    # Assuming motors 1,3 spin CW and motors 2,4 spin CCW
    k_yaw = 0.1  # Torque-to-thrust ratio
    M_yaw = (Thrust_1 + Thrust_3 - Thrust_2 - Thrust_4) * k_yaw

    # Angular acceleration with aerodynamic damping
    dx_dt[3] = (M_roll + CM_Q * p) / IX
    dx_dt[4] = (M_pitch + CM_Q * q) / IY
    dx_dt[5] = (M_yaw + CM_Q * r) / IZ

    # === POSITION UPDATE (Inertial Frame) ===
    dx_dt[6] = vx
    dx_dt[7] = vy
    dx_dt[8] = vz

    # === ATTITUDE KINEMATICS ===
    # Euler angle rates from body angular velocities
    # Using standard kinematic equations
    tan_theta = np.tan(theta)
    cos_theta = np.cos(theta)
    
    if abs(cos_theta) > 0.01:  # Avoid singularity at theta = ±90°
        dx_dt[9] = p + q * s_phi * tan_theta + r * c_phi * tan_theta    # dphi/dt
        dx_dt[10] = q * c_phi - r * s_phi                                 # dtheta/dt
        dx_dt[11] = (q * s_phi + r * c_phi) / cos_theta                   # dpsi/dt
    else:
        # Gimbal lock protection
        dx_dt[9] = p
        dx_dt[10] = q * c_phi - r * s_phi
        dx_dt[11] = 0.0

    return dx_dt
