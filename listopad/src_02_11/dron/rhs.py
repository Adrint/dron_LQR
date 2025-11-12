import numpy as np
from config import config
def aa_rhs(x, t, u, az_turbulence):
    """
    QUADCOPTER X CONFIGURATION - 3D DYNAMICS
    
    State vector x:
    - n = 12: [vx, vy, vz, p, q, r, X, Y, Z, φ, θ, ψ]
    - n = 16: [vx, vy, vz, p, q, r, X, Y, Z, φ, θ, ψ, T1, T2, T3, T4]

    gdzie:
    - vx, vy, vz = prędkości w układzie CIAŁA (body frame: forward, right, down)
    - p, q, r = prędkości kątowe (roll rate, pitch rate, yaw rate)
    - X, Y, Z = pozycja w układzie INERCJALNYM (NED: Z dodatnie W DÓŁ)
    - φ, θ, ψ = kąty Eulera (roll, pitch, yaw)
    - T1, T2, T3, T4 = rzeczywiste ciągi silników [N]

    Control input u:
    - [T1_cmd, T2_cmd, T3_cmd, T4_cmd] = komendy ciągu (w Newtonach)
    
    COORDINATE SYSTEM: NED (North-East-Down)
    - X positive = forward (North)
    - Y positive = right (East)
    - Z positive = downward
    - Ground at Z = 0
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

    ax_wind = config.ax_wind
    ay_wind = config.ay_wind
    az_wind = config.az_wind

    n = config.n
    dx_dt = np.zeros(n)

    # === EKSTRAKCJA STANU ===
    vx = x[0]      # Prędkość forward (w układzie ciała)
    vy = x[1]      # Prędkość right (w układzie ciała)
    vz = x[2]      # Prędkość down (w układzie ciała)
    p = x[3]       # Roll rate (angular velocity about X-axis)
    q = x[4]       # Pitch rate (angular velocity about Y-axis)
    r = x[5]       # Yaw rate (angular velocity about Z-axis)
    X = x[6]       # Pozycja X inercjalna (forward)
    Y = x[7]       # Pozycja Y inercjalna (right)
    Z = x[8]       # Pozycja Z inercjalna (NED: dodatnie W DÓŁ)
    phi = x[9]     # Roll angle
    theta = x[10]  # Pitch angle
    psi = x[11]    # Yaw angle

    # Całkowita prędkość dla obliczeń aerodynamicznych
    V = np.sqrt(vx**2 + vy**2 + vz**2)

    # Gęstość powietrza (zmienia się z wysokością)
    altitude = -Z  # Altitude above ground (positive upward)
    ro = RO_0 * (1.0 - altitude / 44300.0)**4.256
    if ro < 0.0:
        ro = 0.0

    # Ciśnienie dynamiczne
    Q_dyn = 0.5 * ro * V**2

    # Siły aerodynamiczne (opór) w układzie ciała
    if V > 0.001:
        Dx = -Q_dyn * S * CD_0 * (vx / V)
        Dy = -Q_dyn * S * CD_0 * (vy / V)
        Dz = -Q_dyn * S * CD_0 * (vz / V)
    else:
        Dx = Dy = Dz = 0.0

    # ========================================================================
    # SIŁY CIĄGU
    # ========================================================================
    
    # Pobierz rzeczywiste ciągi (QUADCOPTER: 4 motors)
    if n == 16:
        # n = 16: Stan zawiera dynamikę ciągu dla 4 silników
        T1 = x[12]  # Front-right
        T2 = x[13]  # Front-left
        T3 = x[14]  # Rear-left
        T4 = x[15]  # Rear-right
    else:
        # n = 12: Bezpośrednie komendy ciągu
        T1 = u[0]
        T2 = u[1]
        T3 = u[2]
        T4 = u[3]

    T_total = T1 + T2 + T3 + T4

    # ========================================================================
    # SKŁADOWE CIĘŻARU W UKŁADZIE CIAŁA
    # ========================================================================
    
       
    # Transformacja grawitacji z NED do układu ciała
    Gx = -F_g * np.sin(theta)
    Gy = F_g * np.cos(theta) * np.sin(phi)
    Gz = F_g * np.cos(theta) * np.cos(phi)

    # ========================================================================
    # RÓWNANIA RUCHU POSTĘPOWEGO (Body Frame)
    # ========================================================================
    
    # Przyspieszenia liniowe (z wyrazami Coriolisa od obracającego się układu)
    # Coriolis terms: ω × v where ω = [p, q, r], v = [vx, vy, vz]
    # (ω × v)_x = q*vz - r*vy
    # (ω × v)_y = r*vx - p*vz  
    # (ω × v)_z = p*vy - q*vx
    ax = (Dx + Gx) / mass + (q * vz - r * vy) + ax_wind
    ay = (Dy + Gy) / mass + (r * vx - p * vz) + ay_wind
    az = (Dz + Gz - T_total) / mass + (p * vy - q * vx) + az_turbulence + az_wind
    
    dx_dt[0] = ax
    dx_dt[1] = ay
    dx_dt[2] = az

    # ========================================================================
    # MOMENTY OD CIĄGÓW
    # ========================================================================
    
    # Motor positions and groups
    T_front = T1 + T2  # Motors at +X
    T_rear = T3 + T4   # Motors at -X
    T_right = T1 + T4  # Motors at +Y
    T_left = T2 + T3   # Motors at -Y
    
    # Roll moment (about X-axis)
    L = motor_arm_y * (T_right - T_left)
    
    # Pitch moment (about Y-axis)
    M = motor_arm_x * (T_front - T_rear)
    
    # Yaw moment (about Z-axis) from propeller torques
    # Motors 1,3 rotate CW (positive torque), Motors 2,4 rotate CCW (negative torque)
    N = k_torque * ((T1 + T3) - (T2 + T4))

    # Moment tłumienia
    L_damp = C_Lp * p
    M_damp = CM_Q * q
    N_damp = C_Nr * r

    # ========================================================================
    # RÓWNANIA DYNAMIKI OBROTOWEJ (z sprzężeniem żyroskopowym)
    # ========================================================================
    
    # Przyspieszenia kątowe (równania Eulera dla ciała sztywnego)
    dp_dt = (L + L_damp + (IY - IZ) * q * r) / IX
    dq_dt = (M + M_damp + (IZ - IX) * r * p) / IY
    dr_dt = (N + N_damp + (IX - IY) * p * q) / IZ
    
    dx_dt[3] = dp_dt
    dx_dt[4] = dq_dt
    dx_dt[5] = dr_dt

    # ========================================================================
    # AKTUALIZACJA POZYCJI (układ inercjalny NED)
    # ========================================================================
    
    # Macierz rotacji z układu ciała do NED (3-2-1 Euler angles)
    # R = Rz(ψ) * Ry(θ) * Rx(φ)
    
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)
    
    # Transformacja prędkości z układu ciała do NED
    dx_dt[6] = ctheta * cpsi * vx + (sphi * stheta * cpsi - cphi * spsi) * vy + (cphi * stheta * cpsi + sphi * spsi) * vz
    dx_dt[7] = ctheta * spsi * vx + (sphi * stheta * spsi + cphi * cpsi) * vy + (cphi * stheta * spsi - sphi * cpsi) * vz
    dx_dt[8] = -stheta * vx + sphi * ctheta * vy + cphi * ctheta * vz

    # ========================================================================
    # AKTUALIZACJA KĄTÓW EULERA
    # ========================================================================
    
    # Transformacja prędkości kątowych z układu ciała do pochodnych kątów Eulera
    # Uwaga: osobliwość przy theta = ±90°
    
    if abs(np.cos(theta)) > 0.01:  # Avoid singularity
        dx_dt[9] = p + sphi * np.tan(theta) * q + cphi * np.tan(theta) * r  # dphi/dt
        dx_dt[10] = cphi * q - sphi * r  # dtheta/dt
        dx_dt[11] = (sphi / np.cos(theta)) * q + (cphi / np.cos(theta)) * r  # dpsi/dt
    else:
        # Near singularity - use approximation or limit rates
        dx_dt[9] = p
        dx_dt[10] = cphi * q - sphi * r
        dx_dt[11] = 0.0

    # ========================================================================
    # DYNAMIKA SILNIKÓW (tylko dla n=16 - quadcopter)
    # ========================================================================
    
    if n == 16:
        # Model opóźnienia pierwszego rzędu dla 4 silników
        dx_dt[12] = (1.0 / tau) * (u[0] - x[12])  # Motor 1
        dx_dt[13] = (1.0 / tau) * (u[1] - x[13])  # Motor 2
        dx_dt[14] = (1.0 / tau) * (u[2] - x[14])  # Motor 3
        dx_dt[15] = (1.0 / tau) * (u[3] - x[15])  # Motor 4

    return dx_dt




