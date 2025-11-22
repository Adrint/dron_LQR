import numpy as np
from config import config

def rotation_matrix_zyx(phi, theta, psi):
    """
    Oblicza pełną macierz obrotu 3D używając kątów Eulera ZYX (yaw–pitch–roll).
    Zgodne z konwencją lotniczą dla układu współrzędnych NED.

    phi: kąt przechylenia (obrót wokół osi X)
    theta: kąt pochylenia (obrót wokół osi Y)
        - theta > 0: nos do góry
        - theta < 0: nos w dół
    psi: kąt odchylenia (obrót wokół osi Z)

    Zwraca: macierz obrotu 3×3 z układu ciała do układu NED.
    """
    # Obrót wokół osi X (roll)
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    Rx = np.array([
        [1, 0, 0],
        [0, c_phi, -s_phi],
        [0, s_phi, c_phi]
    ])

    # Obrót wokół osi Y (pitch)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    Ry = np.array([
        [c_theta, 0, s_theta],
        [0, 1, 0],
        [-s_theta, 0, c_theta]
    ])

    # Obrót wokół osi Z (yaw)
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
    - n = 12: [vx, vy, vz, p, q, r, X, Y, Z, φ, θ, ψ]
    - n = 16: [vx, vy, vz, p, q, r, X, Y, Z, φ, θ, ψ, T1, T2, T3, T4]
    """
    mass = config.mass
    S = config.S
    CD_0 = config.CD_0
    RO_0 = config.RO_0
    IX = config.IX
    IY = config.IY
    IZ = config.IZ
    motor_arm = config.motor_arm
    k_torque = config.k_torque
    C_Lp = config.C_Lp
    CM_Q = config.CM_Q
    C_Nr = config.C_Nr
    tau = config.tau
    F_g_array = config.g_ned
    n = config.n

    ax_wind = config.ax_wind
    ay_wind = config.ay_wind
    az_wind = config.az_wind

    dx_dt = np.zeros(n)

    # === WYODRĘBNIENIE STANU ===
    vx = x[0]
    vy = x[1]
    vz = x[2]
    p = x[3]
    q = x[4]
    r = x[5]
    X = x[6]
    Y = x[7]
    Z = x[8]
    phi = x[9]
    theta = x[10]
    psi = x[11]

    # Gęstość powietrza (zmienia się z wysokością)
    altitude = -Z
    ro = RO_0 * (1.0 - altitude / 44300.0) ** 4.256
    ro = max(ro, 0.1 * RO_0)

    # Prędkość całkowita
    V = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

    # Ciśnienie dynamiczne
    Q_dyn = 0.5 * ro * V ** 2

    # ========================================================================
    # SIŁY CIĄGU
    # ========================================================================

    if n == 16:
        # n = 16: stan zawiera dynamikę silników
        T1 = x[12]
        T2 = x[13]
        T3 = x[14]
        T4 = x[15]
    else:
        # n = 12: ciągi bezpośrednio z wejść sterowania
        T1 = u[0]
        T2 = u[1]
        T3 = u[2]
        T4 = u[3]

    T_total = T1 + T2 + T3 + T4

    # Siły aerodynamiczne (opory)
    if V > 0.01:
        drag_coeff = Q_dyn * S * CD_0 / V
        Dx = -drag_coeff * vx
        Dy = -drag_coeff * vy
        Dz = -drag_coeff * vz
    else:
        Dx = Dy = Dz = 0.0

    # ========================================================================
    # SIŁA CIĘŻKOŚCI W UKŁADZIE CIAŁA
    # ========================================================================

    R = rotation_matrix_zyx(phi, theta, psi)
    g_body = R.T @ F_g_array

    Gx = g_body[0]
    Gy = g_body[1]
    Gz = g_body[2]

    # ========================================================================
    # RÓWNANIA RUCHU POSTĘPOWEGO (Body Frame)
    # ========================================================================

    # Człon Coriolisa: ω × v
    ax = (Dx + Gx) / mass + (q * vz - r * vy) + ax_wind
    ay = (Dy + Gy) / mass + (r * vx - p * vz) + ay_wind
    az = (Dz + Gz - T_total) / mass + (p * vy - q * vx) + az_turbulence + az_wind

    dx_dt[0] = ax
    dx_dt[1] = ay
    dx_dt[2] = az

    # ========================================================================
    # MOMENTY OD SILNIKÓW
    # ========================================================================

    T_front = T1 + T2
    T_rear = T3 + T4
    T_right = T1 + T4
    T_left = T2 + T3

    # Moment przechylenia (X)
    L = motor_arm * (T_right - T_left)

    # Moment pochylenia (Y)
    M = motor_arm * (T_front - T_rear)

    # Moment odchylenia (Z)
    N = k_torque * ((T1 + T3) - (T2 + T4))

    # Moment tłumiący (aerodynamiczny)
    L_damp = C_Lp * p * (1 + 0.05 * abs(p))
    M_damp = CM_Q * q * (1 + 0.05 * abs(q))
    N_damp = C_Nr * r * (1 + 0.05 * abs(r))

    # ========================================================================
    # RÓWNANIA ROTACJI (równania Eulera)
    # ========================================================================

    dp_dt = (L + L_damp + (IY - IZ) * q * r) / IX
    dq_dt = (M + M_damp + (IZ - IX) * r * p) / IY
    dr_dt = (N + N_damp + (IX - IY) * p * q) / IZ

    dx_dt[3] = dp_dt
    dx_dt[4] = dq_dt
    dx_dt[5] = dr_dt

    # ========================================================================
    # AKTUALIZACJA POZYCJI (układ NED)
    # ========================================================================

    v_body = np.array([vx, vy, vz])
    v_ned = R @ v_body

    dx_dt[6] = v_ned[0]
    dx_dt[7] = v_ned[1]
    dx_dt[8] = v_ned[2]

    # ========================================================================
    # PRĘDKOŚCI KĄTOWE KĄTÓW EULERA
    # ========================================================================

    cphi = np.cos(phi)
    sphi = np.sin(phi)

    if abs(np.cos(theta)) > 0.01:
        dx_dt[9] = p + sphi * np.tan(theta) * q + cphi * np.tan(theta) * r
        dx_dt[10] = cphi * q - sphi * r
        dx_dt[11] = (sphi / np.cos(theta)) * q + (cphi / np.cos(theta)) * r
    else:
        # Blisko osobliwości — stosujemy przybliżenie
        dx_dt[9] = p
        dx_dt[10] = cphi * q - sphi * r
        dx_dt[11] = np.sign(np.cos(theta)) * r

    # ========================================================================
    # DYNAMIKA SILNIKÓW (tylko dla n = 16)
    # ========================================================================

    if n == 16:
        dx_dt[12] = (1.0 / tau) * (u[0] - x[12])
        dx_dt[13] = (1.0 / tau) * (u[1] - x[13])
        dx_dt[14] = (1.0 / tau) * (u[2] - x[14])
        dx_dt[15] = (1.0 / tau) * (u[3] - x[15])

    return dx_dt
