import numpy as np
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

    deg2rad = np.pi / 180.0
    g = 9.81

    S = 1.0
    mass = 25.0
    Iy = 100

    vx = x[0] + ax_wind
    vz = x[1] + az_wind

    alpha = np.arctan2(vz, vx)
    V = np.sqrt(vz ** 2 + vx ** 2)

    CD_0 = 0.30
    CD = CD_0

    ro_0 = 1.225
    ro = ro_0 * (1.0 - abs(x[4]) / 44300.0) ** 4.256

    Q_dyn = 0.5 * ro * V ** 2

    L = 0.0
    D = Q_dyn * S * CD
    G = mass * g

    Th = 1.0

    if n == 8:
        Thrust_1 = x[6]
        Thrust_2 = x[7]
    else:
        Thrust_1 = 0.5 * G + u[0]
        Thrust_2 = 0.5 * G + u[1]

    cm_q = -0.01
    Tau = 0.05

    beta = 0.0 * deg2rad
    cb = np.cos(beta)
    sb = np.sin(beta)

    dx_dt[0] = ((-D * np.cos(alpha) + L * np.sin(alpha) - G * np.sin(x[5])
                 - Thrust_1 * sb + Thrust_2 * sb) / mass - x[2] * vz)
    dx_dt[1] = ((-D * np.sin(alpha) - L * np.cos(alpha) + G * np.cos(x[5])
                 - Thrust_1 * cb - Thrust_2 * cb) / mass + x[2] * vx + az_turbulence)
    dx_dt[2] = (0.5 * (Thrust_2 * cb - Thrust_1 * cb) + cm_q * x[2]) / Iy
    dx_dt[3] = np.cos(x[5]) * vx + np.sin(x[5]) * vz
    dx_dt[4] = -np.sin(x[5]) * vx + np.cos(x[5]) * vz
    dx_dt[5] = x[2]

    if n == 8:
        dx_dt[6] = (1.0 / Tau) * (-x[6] + Th * u[0])
        dx_dt[7] = (1.0 / Tau) * (-x[7] + Th * u[1])

    return dx_dt