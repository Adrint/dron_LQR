import numpy as np
from constants import (CD_0, RO_0, MASS, g, S, MOTOR_ARM_LENGTH, CM_Q, IY, TAU)


def aa_rhs(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """
    Right-hand side of the drone differential equations

    State vector x:
    - n=6: [vx, vz, omega, X, Z, theta]
    - n=8: [vx, vz, omega, X, Z, theta, Thrust_1, Thrust_2]

    Control input u:
    - [T1, T2] = thrust commands (in Newtons)
    """
    n = len(x)
    dx_dt = np.zeros(n)

    # Extract state
    vx = x[0]
    vz = x[1]
    omega = x[2]
    X = x[3]
    Z = x[4]
    theta = x[5]

    # Add wind effects
    vx_wind = vx + ax_wind
    vz_wind = vz + az_wind

    # === AERODYNAMICS ===
    V = np.sqrt(vx_wind ** 2 + vz_wind ** 2)

    # Air density variation with altitude
    ro = RO_0 * (1.0 - abs(Z) / 44300.0) ** 4.256
    if ro < 0:
        ro = 0.0

    # Drag calculation
    if V > 0.01:
        # Dynamic pressure
        Q_dyn = 0.5 * ro * V ** 2

        # Total drag force magnitude
        D_mag = Q_dyn * S * CD_0

        # Drag components (opposing motion)
        Dx = D_mag * vx_wind / V
        Dz = D_mag * vz_wind / V
    else:
        Dx = 0.0
        Dz = 0.0

    # === THRUST FORCES ===
    G = MASS * g

    # Get actual thrusts
    if n == 8:
        # n=8: State includes thrust dynamics
        Thrust_1 = x[6]
        Thrust_2 = x[7]
    else:
        # n=6: Direct thrust from control input
        Thrust_1 = u[0]
        Thrust_2 = u[1]

    T_total = Thrust_1 + Thrust_2

    # Thrust components in inertial frame (body-fixed to inertial transformation)
    # Positive theta = nose up
    Fx = T_total * np.sin(theta)  # horizontal component
    Fz = T_total * np.cos(theta)  # vertical component (upward)

    # === EQUATIONS OF MOTION (Inertial Frame) ===
    # No Coriolis terms - we're in inertial frame

    # Horizontal acceleration
    dx_dt[0] = (Fx - Dx) / MASS

    # Vertical acceleration (gravity acts downward)
    dx_dt[1] = (Fz - G - Dz) / MASS + az_turbulence

    # === ROTATIONAL DYNAMICS ===
    # Moment from differential thrust
    Mt = MOTOR_ARM_LENGTH * (Thrust_2 - Thrust_1)

    # Angular acceleration (with damping term)
    dx_dt[2] = (Mt + CM_Q * omega) / IY

    # === POSITION UPDATES ===
    # Transform velocity from body frame to inertial frame
    # Actually, vx and vz are already in inertial frame in this model
    # So position update is direct:
    dx_dt[3] = vx
    dx_dt[4] = vz

    # === ATTITUDE ===
    dx_dt[5] = omega

    # === THRUST DYNAMICS (if n=8) ===
    if n == 8:
        # First-order lag model for thrusts
        # dx/dt = (1/tau) * (u - x)
        # This models servo/motor response
        dx_dt[6] = (1.0 / TAU) * (u[0] - x[6])
        dx_dt[7] = (1.0 / TAU) * (u[1] - x[7])

    return dx_dt