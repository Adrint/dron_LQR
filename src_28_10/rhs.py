import numpy as np
from constants import (CD_0, RO_0, MASS, g, S, MOTOR_ARM_LENGTH_X, MOTOR_ARM_LENGTH_Y, 
                       CM_Q, C_Lp, C_Nr, IX, IY, IZ, TAU, k_torque)


def aa_rhs(x, t, u, az_turbulence=0.0, ax_wind=0.0, ay_wind=0.0, az_wind=0.0):
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

    n = len(x)
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
    
    G = MASS * g
    
    # Transformacja grawitacji z NED do układu ciała
    Gx = -G * np.sin(theta)
    Gy = G * np.cos(theta) * np.sin(phi)
    Gz = G * np.cos(theta) * np.cos(phi)

    # ========================================================================
    # RÓWNANIA RUCHU POSTĘPOWEGO (Body Frame)
    # ========================================================================
    
    # Przyspieszenia liniowe (z wyrazami Coriolisa od obracającego się układu)
    ax = (Dx + Gx) / MASS + r * vy - q * vz + ax_wind
    ay = (Dy + Gy) / MASS + p * vz - r * vx + ay_wind
    az = (Dz + Gz - T_total) / MASS + q * vx - p * vy + az_turbulence + az_wind
    
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
    L = MOTOR_ARM_LENGTH_Y * (T_right - T_left)
    
    # Pitch moment (about Y-axis)
    M = MOTOR_ARM_LENGTH_X * (T_front - T_rear)
    
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
        dx_dt[12] = (1.0 / TAU) * (u[0] - x[12])  # Motor 1
        dx_dt[13] = (1.0 / TAU) * (u[1] - x[13])  # Motor 2
        dx_dt[14] = (1.0 / TAU) * (u[2] - x[14])  # Motor 3
        dx_dt[15] = (1.0 / TAU) * (u[3] - x[15])  # Motor 4

    return dx_dt


# ============================================================================
# FUNKCJA POMOCNICZA - DIAGNOSTYKA
# ============================================================================

def diagnostyka_sil(x, u, az_turbulence=0.0, ax_wind=0.0, ay_wind=0.0):
    """
    Wyświetla dekompozycję sił działających na drona (QUADCOPTER 3D)
    """
    n = len(x)
    vx, vy, vz = x[0], x[1], x[2]
    p, q, r = x[3], x[4], x[5]
    X, Y, Z = x[6], x[7], x[8]
    phi, theta, psi = x[9], x[10], x[11]
    
    # Aerodynamika
    V = np.sqrt(vx**2 + vy**2 + vz**2)
    altitude = -Z
    ro = RO_0 * (1.0 - altitude / 44300.0)**4.256
    if ro < 0.0:
        ro = 0.0
    Q_dyn = 0.5 * ro * V**2
    
    if V > 0.001:
        Dx = -Q_dyn * S * CD_0 * (vx / V)
        Dy = -Q_dyn * S * CD_0 * (vy / V)
        Dz = -Q_dyn * S * CD_0 * (vz / V)
    else:
        Dx = Dy = Dz = 0.0
    
    # Ciągi
    if n == 16:
        T1, T2, T3, T4 = x[12], x[13], x[14], x[15]
    else:
        T1, T2, T3, T4 = u[0], u[1], u[2], u[3]
    
    T_total = T1 + T2 + T3 + T4
    T_front = T1 + T2
    T_rear = T3 + T4
    T_right = T1 + T4
    T_left = T2 + T3
    
    G = MASS * g
    Gx = -G * np.sin(theta)
    Gy = G * np.cos(theta) * np.sin(phi)
    Gz = G * np.cos(theta) * np.cos(phi)
    
    print("=" * 80)
    print("DIAGNOSTYKA SIŁ (układ ciała) - QUADCOPTER 3D X-CONFIG")
    print("=" * 80)
    print(f"Pozycja: X = {X:.2f}m, Y = {Y:.2f}m, Z_NED = {Z:.2f}m, AGL = {altitude:.2f}m")
    print(f"Prędkość V = {V:.2f} m/s")
    print(f"Kąty: φ = {np.degrees(phi):.2f}°, θ = {np.degrees(theta):.2f}°, ψ = {np.degrees(psi):.2f}°")
    print(f"Prędkości kątowe: p = {np.degrees(p):.2f}°/s, q = {np.degrees(q):.2f}°/s, r = {np.degrees(r):.2f}°/s")
    print()
    
    print("SIŁY W OSI X CIAŁA (forward):")
    print(f"  Opór:         {Dx:.2f} N")
    print(f"  Ciężar:       {Gx:.2f} N")
    print(f"  Suma:         {Dx + Gx:.2f} N")
    print(f"  Przyspieszenie: {(Dx + Gx)/MASS:.2f} m/s²")
    print()
    
    print("SIŁY W OSI Y CIAŁA (right):")
    print(f"  Opór:         {Dy:.2f} N")
    print(f"  Ciężar:       {Gy:.2f} N")
    print(f"  Suma:         {Dy + Gy:.2f} N")
    print(f"  Przyspieszenie: {(Dy + Gy)/MASS:.2f} m/s²")
    print()
    
    print("SIŁY W OSI Z CIAŁA (down):")
    print(f"  Opór:         {Dz:.2f} N")
    print(f"  Ciężar:       {Gz:.2f} N (w dół = +Z)")
    print(f"  Ciąg T1 (FR): {-T1:.2f} N (w górę = -Z)")
    print(f"  Ciąg T2 (FL): {-T2:.2f} N (w górę = -Z)")
    print(f"  Ciąg T3 (RL): {-T3:.2f} N (w górę = -Z)")
    print(f"  Ciąg T4 (RR): {-T4:.2f} N (w górę = -Z)")
    print(f"  Ciąg całkowity: {-T_total:.2f} N")
    print(f"  Suma sił:     {Dz + Gz - T_total:.2f} N")
    print(f"  Przyspieszenie: {(Dz + Gz - T_total)/MASS + az_turbulence:.2f} m/s²")
    print()
    
    print("MOMENTY:")
    print(f"  Ciąg prawy (T1+T4):  {T_right:.2f} N")
    print(f"  Ciąg lewy (T2+T3):   {T_left:.2f} N")
    print(f"  Roll moment:         {MOTOR_ARM_LENGTH_Y * (T_right - T_left):.2f} N·m")
    print()
    print(f"  Ciąg przedni (T1+T2): {T_front:.2f} N")
    print(f"  Ciąg tylny (T3+T4):   {T_rear:.2f} N")
    print(f"  Pitch moment:         {MOTOR_ARM_LENGTH_X * (T_front - T_rear):.2f} N·m")
    print()
    print(f"  Moment yaw (torque):  {k_torque * ((T1 + T3) - (T2 + T4)):.2f} N·m")
    print("=" * 80)
    print()


# ============================================================================
# BACKWARD COMPATIBILITY - 2D VERSION
# ============================================================================

def aa_rhs_2d(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """
    2D version for backward compatibility with existing code.
    Maps 2D state to 3D, runs 3D dynamics, extracts 2D state.
    
    State vector x (2D):
    - n = 6: [vx, vz, omega, X, Z, theta]
    - n = 10: [vx, vz, omega, X, Z, theta, T1, T2, T3, T4]
    """
    n_2d = len(x)
    
    if n_2d == 6:
        # Map to 3D (n=12)
        x_3d = np.zeros(12)
        x_3d[0] = x[0]  # vx
        x_3d[1] = 0.0   # vy = 0
        x_3d[2] = x[1]  # vz
        x_3d[3] = 0.0   # p = 0
        x_3d[4] = x[2]  # q = omega
        x_3d[5] = 0.0   # r = 0
        x_3d[6] = x[3]  # X
        x_3d[7] = 0.0   # Y = 0
        x_3d[8] = x[4]  # Z
        x_3d[9] = 0.0   # phi = 0
        x_3d[10] = x[5] # theta
        x_3d[11] = 0.0  # psi = 0
        
        dx_3d = aa_rhs(x_3d, t, u, az_turbulence, ax_wind, 0.0, az_wind)
        
        # Extract 2D
        dx_2d = np.zeros(6)
        dx_2d[0] = dx_3d[0]  # dvx/dt
        dx_2d[1] = dx_3d[2]  # dvz/dt
        dx_2d[2] = dx_3d[4]  # dq/dt
        dx_2d[3] = dx_3d[6]  # dX/dt
        dx_2d[4] = dx_3d[8]  # dZ/dt
        dx_2d[5] = dx_3d[10] # dtheta/dt
        
        return dx_2d
        
    elif n_2d == 10:
        # Map to 3D (n=16)
        x_3d = np.zeros(16)
        x_3d[0] = x[0]  # vx
        x_3d[1] = 0.0   # vy = 0
        x_3d[2] = x[1]  # vz
        x_3d[3] = 0.0   # p = 0
        x_3d[4] = x[2]  # q = omega
        x_3d[5] = 0.0   # r = 0
        x_3d[6] = x[3]  # X
        x_3d[7] = 0.0   # Y = 0
        x_3d[8] = x[4]  # Z
        x_3d[9] = 0.0   # phi = 0
        x_3d[10] = x[5] # theta
        x_3d[11] = 0.0  # psi = 0
        x_3d[12] = x[6] # T1
        x_3d[13] = x[7] # T2
        x_3d[14] = x[8] # T3
        x_3d[15] = x[9] # T4
        
        dx_3d = aa_rhs(x_3d, t, u, az_turbulence, ax_wind, 0.0, az_wind)
        
        # Extract 2D
        dx_2d = np.zeros(10)
        dx_2d[0] = dx_3d[0]  # dvx/dt
        dx_2d[1] = dx_3d[2]  # dvz/dt
        dx_2d[2] = dx_3d[4]  # dq/dt
        dx_2d[3] = dx_3d[6]  # dX/dt
        dx_2d[4] = dx_3d[8]  # dZ/dt
        dx_2d[5] = dx_3d[10] # dtheta/dt
        dx_2d[6] = dx_3d[12] # dT1/dt
        dx_2d[7] = dx_3d[13] # dT2/dt
        dx_2d[8] = dx_3d[14] # dT3/dt
        dx_2d[9] = dx_3d[15] # dT4/dt
        
        return dx_2d
    
    else:
        raise ValueError(f"Unsupported state dimension: {n_2d}")


# ============================================================================
# TEST RÓWNOWAGI
# ============================================================================

if __name__ == "__main__":
    """
    Test: Czy dron w poziomie z równymi ciągami = ciężar pozostaje w równowadze?
    """
    print("TEST RÓWNOWAGI DRONA 3D (NED Coordinate System) - QUADCOPTER\n")
    
    # Stan: dron w poziomie, bez ruchu, na wysokości -2m (2m nad ziemią w NED)
    x_hover = np.array([
        0.0,           # vx = 0
        0.0,           # vy = 0
        0.0,           # vz = 0
        0.0,           # p = 0
        0.0,           # q = 0
        0.0,           # r = 0
        0.0,           # X = 0
        0.0,           # Y = 0
        -2.0,          # Z = -2m (2m nad ziemią)
        0.0,           # phi = 0
        0.0,           # theta = 0
        0.0,           # psi = 0
        MASS * g / 4,  # T1 = 1/4 ciężaru
        MASS * g / 4,  # T2 = 1/4 ciężaru
        MASS * g / 4,  # T3 = 1/4 ciężaru
        MASS * g / 4   # T4 = 1/4 ciężaru
    ])
    
    u_hover = np.array([MASS * g / 4, MASS * g / 4, MASS * g / 4, MASS * g / 4])
    
    # Oblicz pochodne
    dx = aa_rhs(x_hover, 0.0, u_hover)
    
    print(f"Masa drona: {MASS} kg")
    print(f"Ciężar: {MASS * g:.2f} N")
    print(f"Ciąg na silnik: {MASS * g / 4:.2f} N")
    print(f"Całkowity ciąg: {MASS * g:.2f} N")
    print()
    
    print("POCHODNE STANU (powinny być ~0 dla równowagi):")
    labels = ['dvx/dt', 'dvy/dt', 'dvz/dt', 'dp/dt', 'dq/dt', 'dr/dt',
              'dX/dt', 'dY/dt', 'dZ/dt', 'dφ/dt', 'dθ/dt', 'dψ/dt',
              'dT1/dt', 'dT2/dt', 'dT3/dt', 'dT4/dt']
    
    for i, (label, val) in enumerate(zip(labels[:len(dx)], dx)):
        print(f"  {label:8s} = {val:+.6f}")
    
    print()
    
    # Sprawdź czy suma sił jest bliska zeru
    if abs(dx[2]) < 1e-6:  # dvz/dt should be ~0
        print("✓ TEST ZALICZONY: Dron w równowadze!")
    else:
        print("✗ TEST NIEZALICZONY: Dron nie jest w równowadze!")
    
    print("\n" + "=" * 80)
    diagnostyka_sil(x_hover, u_hover)
