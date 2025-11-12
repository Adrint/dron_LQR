import numpy as np
#global variables
from constants import (CD_0, RO_0, MASS, g, S, MOTOR_ARM_LENGTH, CM_Q, IY,TAU)

def aa_rhs(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """
    Right-hand side of the drone differential equations

    Parameters:
    -----------
    x : array_like
        State vector
    t : float
        Time
    u : array_like
        Control input vector
    az_turbulence : float
        Vertical turbulence acceleration
    ax_wind : float
        Horizontal wind acceleration
    az_wind : float
        Vertical wind acceleration

    Returns:
    --------
    dx_dt : ndarray
        Time derivative of state vector
    """
    n = len(x)
    dx_dt = np.zeros(n)

    vx = x[0] + ax_wind
    vz = x[1] + az_wind

    alpha = np.arctan2(vz, vx)
    V = np.sqrt(vz ** 2 + vx ** 2)
    CD = CD_0


    ro = RO_0 * (1.0 - abs(x[4]) / 44300.0) ** 4.256

    Q_dyn = 0.5 * ro * V ** 2

    L = 0.0
    D = Q_dyn * S * CD
    G = MASS * g

    Th = 1.0

    if n == 8:
        Thrust_1 = x[6]
        Thrust_2 = x[7]
    else:
        Thrust_1 = 0.5 * G + u[0]
        Thrust_2 = 0.5 * G + u[1]



    # --- SIŁY CIĄGU OBRÓCONE O KĄT THETA ---
    theta = x[5]   # kąt pochylenia
    T_total = Thrust_1 + Thrust_2
    Fx = T_total * np.sin(theta)   # składowa pozioma ciągu
    Fz = T_total * np.cos(theta)   # składowa pionowa ciągu

    # --- Równania ruchu ---
    dx_dt[0] = ((-D * np.cos(alpha) + L * np.sin(alpha) + Fx - G * np.sin(theta)) / MASS
                - x[2] * vz)
    dx_dt[1] = ((-D * np.sin(alpha) - L * np.cos(alpha) + Fz + G * np.cos(theta)) / MASS
                + x[2] * vx + az_turbulence)

    # moment obrotowy od różnicy ciągów
    Mt = MOTOR_ARM_LENGTH * (Thrust_2 - Thrust_1)
    dx_dt[2] = (Mt + CM_Q * x[2]) / IY

    # pozycja w globalnym układzie
    dx_dt[3] = np.cos(theta) * vx + np.sin(theta) * vz
    dx_dt[4] = -np.sin(theta) * vx + np.cos(theta) * vz
    dx_dt[5] = x[2]  # d(theta)/dt = omega

    if n == 8:
        dx_dt[6] = (1.0 / TAU) * (-x[6] + Th * u[0])
        dx_dt[7] = (1.0 / TAU) * (-x[7] + Th * u[1])

    return dx_dt
