import numpy as np
from constants import (CD_0, RO_0, MASS, g, S, MOTOR_ARM_LENGTH_X, CM_Q, IY, TAU)


def aa_rhs(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """
    QUADCOPTER X CONFIGURATION (4 motors)
    
    State vector x:
    - n = 6: [vx, vz, omega, X, Z, theta]
    - n = 10: [vx, vz, omega, X, Z, theta, Thrust_1, Thrust_2, Thrust_3, Thrust_4]

    gdzie:
    - vx, vz = prędkości w układzie CIAŁA (body frame)
    - omega = prędkość kątowa (pitch rate)
    - X, Z = pozycja w układzie INERCJALNYM (NED: Z dodatnie W DÓŁ, ground = Z=0)
    - theta = kąt pochylenia (pitch angle, positive = nose up)
    - Thrust_1, Thrust_2 = rzeczywiste ciągi silników przednich [N] (at +X)
    - Thrust_3, Thrust_4 = rzeczywiste ciągi silników tylnych [N] (at -X)

    Control input u:
    - [T1_cmd, T2_cmd, T3_cmd, T4_cmd] = komendy ciągu (w Newtonach)
    
    COORDINATE SYSTEM: NED (North-East-Down)
    - Z positive = downward
    - Ground at Z = 0
    - Flying altitude: Z < 0
    - Gravity acts in +Z direction (downward)
    - Thrust acts in -Z direction (upward)
    """

    n = len(x)
    dx_dt = np.zeros(n)

    # === EKSTRAKCJA STANU ===
    vx = x[0]      # Prędkość w osi X ciała
    vz = x[1]      # Prędkość w osi Z ciała
    omega = x[2]   # Prędkość kątowa
    X = x[3]       # Pozycja X inercjalna
    Z = x[4]       # Pozycja Z inercjalna (NED: dodatnie W DÓŁ)
    theta = x[5]   # Kąt pochylenia

    # Prędkości w układzie ciała (bez modyfikacji wiatrem dla obliczeń aerodynamicznych)
    # Wiatr będzie dodany jako przyspieszenia w równaniach ruchu
    
    # Całkowita prędkość dla obliczeń aerodynamicznych
    V = np.sqrt(vx**2 + vz**2)

    # Gęstość powietrza (zmienia się z wysokością)
    # W NED: Z jest dodatnie w dół, więc altitude = -Z
    altitude = -Z  # Altitude above ground (positive upward)
    ro = RO_0 * (1.0 - altitude / 44300.0)**4.256
    if ro < 0.0:
        ro = 0.0

    # Ciśnienie dynamiczne
    Q_dyn = 0.5 * ro * V**2

    # Siły aerodynamiczne w układzie prędkości
    CD = CD_0
    D = Q_dyn * S * CD  # Opór (Drag)

    # ========================================================================
    # SIŁY CIĄGU
    # ========================================================================
    
    G = MASS * g  # Ciężar
    alpha = np.arctan2(vz,vx)
    # Pobierz rzeczywiste ciągi (QUADCOPTER: 4 motors)
    if n == 10:
        # n = 10: Stan zawiera dynamikę ciągu dla 4 silników
        Thrust_1 = x[6]  # Front-left
        Thrust_2 = x[7]  # Front-right
        Thrust_3 = x[8]  # Rear-left
        Thrust_4 = x[9]  # Rear-right
    else:
        # n = 6: Bezpośrednie komendy ciągu
        Thrust_1 = u[0]
        Thrust_2 = u[1]
        Thrust_3 = u[2]
        Thrust_4 = u[3]

    # ========================================================================
    # RÓWNANIA RUCHU W UKŁADZIE CIAŁA (Body Frame)
    # ========================================================================
    
    # --- Przyspieszenie w osi X ciała ---
    # Składowe sił:
    # 1. -D*cos(alpha): opór przeciwko ruchowi w kierunku X
    # 2. -G*sin(theta): składowa ciężaru w kierunku X ciała (hamuje przy theta > 0)
    # 3. -omega*vz: wyraz Coriolisa (obrót układu)
    # 4. +ax_wind: wiatr poziomy
    
    ax = ((-D * np.cos(alpha) +
        -G * np.sin(theta)) / MASS -
          omega * vz + ax_wind)

    dx_dt[0] = ax

    # --- Przyspieszenie w osi Z ciała (NED: Z w dół) ---
    # Składowe sił:
    # 1. -D*sin(alpha): opór w kierunku Z (przeciw ruchowi pionowemu)
    # 2. +G*cos(theta): ciężar działa w dół (+Z w NED)
    # 3. -Thrust_total: ciągi wszystkich 4 silników (w górę, czyli -Z)
    # 4. +omega*vx: wyraz Coriolisa (obrót układu)
    # 5. +az_turbulence: turbulencja
    # 6. +az_wind: wiatr pionowy
    
    Thrust_total = Thrust_1 + Thrust_2 + Thrust_3 + Thrust_4
    
    az = ((-D * np.sin(alpha)  + G * np.cos(theta) - Thrust_total) / MASS +
          omega * vx + az_turbulence + az_wind)
    
    dx_dt[1] = az

    # ========================================================================
    # DYNAMIKA OBROTOWA (Pitch control for quadcopter)
    # ========================================================================
    
    # Moment od różnicy ciągów (front vs rear motors)
    # Front motors (1,2) at +X: positive thrust → nose down moment
    # Rear motors (3,4) at -X: positive thrust → nose up moment
    # Pitch moment = arm_x * (Thrust_rear - Thrust_front)
    
    Thrust_front = Thrust_1 + Thrust_2  # Motors at +X
    Thrust_rear = Thrust_3 + Thrust_4   # Motors at -X
    
    Mt = MOTOR_ARM_LENGTH_X * (Thrust_rear - Thrust_front)
    
    # Moment tłumienia (pitch damping)
    Mq = CM_Q * omega
    
    # Przyspieszenie kątowe
    dx_dt[2] = (Mt + Mq) / IY

    # ========================================================================
    # AKTUALIZACJA POZYCJI (układ inercjalny NED)
    # ========================================================================
    
    # Transformacja prędkości z układu ciała do inercjalnego (NED)
    # Macierz rotacji (pitch angle theta):
    # [dX/dt]   [cos(θ)   sin(θ)]   [vx]
    # [dZ/dt] = [-sin(θ)  cos(θ)]   [vz]
    # 
    # W NED: Z dodatnie w dół
    # - Gdy theta > 0 (nos w górę) i vx > 0: dZ/dt < 0 (wznoszenie, Z maleje)
    # - Gdy vz > 0 (ruch w dół w ciała): dZ/dt > 0 (Z rośnie)
    
    dx_dt[3] = np.cos(theta) * vx + np.sin(theta) * vz      # dX/dt
    dx_dt[4] = -np.sin(theta) * vx + np.cos(theta) * vz     # dZ/dt (Z w dół)

    # ========================================================================
    # AKTUALIZACJA KĄTA
    # ========================================================================
    
    dx_dt[5] = omega  # dtheta/dt = omega

    # ========================================================================
    # DYNAMIKA SILNIKÓW (tylko dla n=10 - quadcopter)
    # ========================================================================
    
    if n == 10:
        # Model opóźnienia pierwszego rzędu dla 4 silników:
        # dT/dt = (1/tau) * (T_cmd - T_actual)
        # Modeluje bezwładność silników/serw
        
        dx_dt[6] = (1.0 / TAU) * (u[0] - x[6])  # Motor 1 (front-left)
        dx_dt[7] = (1.0 / TAU) * (u[1] - x[7])  # Motor 2 (front-right)
        dx_dt[8] = (1.0 / TAU) * (u[2] - x[8])  # Motor 3 (rear-left)
        dx_dt[9] = (1.0 / TAU) * (u[3] - x[9])  # Motor 4 (rear-right)

    return dx_dt


# ============================================================================
# FUNKCJA POMOCNICZA - DIAGNOSTYKA
# ============================================================================

def diagnostyka_sil(x, u, az_turbulence=0.0):
    """
    Wyświetla dekompozycję sił działających na drona (QUADCOPTER)
    Przydatne do debugowania
    
    NED: Z positive = downward, altitude = -Z
    """
    n = len(x)
    vx, vz, omega = x[0], x[1], x[2]
    X, Z, theta = x[3], x[4], x[5]
    
    # Aerodynamika
    V = np.sqrt(vx**2 + vz**2)
    alpha = np.arctan2(vz, vx)
    altitude = -Z
    ro = RO_0 * (1.0 - altitude / 44300.0)**4.256
    if ro < 0.0:
        ro = 0.0
    Q_dyn = 0.5 * ro * V**2
    D = Q_dyn * S * CD_0
    
    # Ciągi (quadcopter: 4 motors)
    if n == 10:
        T1, T2, T3, T4 = x[6], x[7], x[8], x[9]
    else:
        T1, T2, T3, T4 = u[0], u[1], u[2], u[3]
    
    Thrust_total = T1 + T2 + T3 + T4
    Thrust_front = T1 + T2
    Thrust_rear = T3 + T4
    
    G = MASS * g
    
    print("=" * 70)
    print("DIAGNOSTYKA SIŁ (układ ciała) - QUADCOPTER X-CONFIG")
    print("=" * 70)
    print(f"Pozycja: X = {X:.2f}m, Z_NED = {Z:.2f}m, AGL = {altitude:.2f}m")
    print(f"Prędkość V = {V:.2f} m/s, kąt natarcia α = {np.degrees(alpha):.2f}°")
    print(f"Kąt pochylenia θ = {np.degrees(theta):.2f}°")
    print()
    
    print("SIŁY W OSI X CIAŁA:")
    print(f"  Opór:         {-D * np.cos(alpha):.2f} N")
    print(f"  Ciężar:       {-G * np.sin(theta):.2f} N")
    print(f"  Suma:         {-D * np.cos(alpha) - G * np.sin(theta):.2f} N")
    print(f"  Przyspieszenie: {(-D * np.cos(alpha) - G * np.sin(theta))/MASS:.2f} m/s²")
    print()
    
    print("SIŁY W OSI Z CIAŁA (NED: +Z = w dół):")
    print(f"  Opór:         {-D * np.sin(alpha):.2f} N")
    print(f"  Ciężar:       {+G * np.cos(theta):.2f} N (w dół = +Z)")
    print(f"  Ciąg T1 (F-L): {-T1:.2f} N (w górę = -Z)")
    print(f"  Ciąg T2 (F-R): {-T2:.2f} N (w górę = -Z)")
    print(f"  Ciąg T3 (R-L): {-T3:.2f} N (w górę = -Z)")
    print(f"  Ciąg T4 (R-R): {-T4:.2f} N (w górę = -Z)")
    print(f"  Ciąg całkowity: {-Thrust_total:.2f} N")
    print(f"  Turbulencja:  {az_turbulence:.2f} m/s²")
    print(f"  Suma sił:     {-D * np.sin(alpha) + G * np.cos(theta) - Thrust_total:.2f} N")
    print(f"  Przyspieszenie: {(-D * np.sin(alpha) + G * np.cos(theta) - Thrust_total)/MASS + az_turbulence:.2f} m/s²")
    print()
    
    print("MOMENTY:")
    print(f"  Ciąg przedni (T1+T2): {Thrust_front:.2f} N")
    print(f"  Ciąg tylny (T3+T4): {Thrust_rear:.2f} N")
    print(f"  Moment od ciągów: {MOTOR_ARM_LENGTH_X * (Thrust_rear - Thrust_front):.2f} N·m")
    print(f"  Moment tłumienia: {CM_Q * omega:.2f} N·m")
    print(f"  Przyspieszenie kątowe: {(MOTOR_ARM_LENGTH_X * (Thrust_rear - Thrust_front) + CM_Q * omega) / IY:.2f} rad/s²")
    print("=" * 70)
    print()


# ============================================================================
# TEST RÓWNOWAGI
# ============================================================================

if __name__ == "__main__":
    """
    Test: Czy dron w poziomie z równymi ciągami = ciężar pozostaje w równowadze?
    """
    print("TEST RÓWNOWAGI DRONA (NED Coordinate System) - QUADCOPTER\n")
    
    # Stan: dron w poziomie, bez ruchu, na wysokości -2m (2m nad ziemią w NED)
    # Quadcopter: 4 motors, each carries 1/4 of weight
    x_hover = np.array([
        0.0,           # vx = 0
        0.0,           # vz = 0
        0.0,           # omega = 0
        0.0,           # X = 0
        -2.0,          # Z = -2m (2m nad ziemią, ground = Z=0)
        0.0,           # theta = 0 (poziomo)
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
    print(f"Całkowity ciąg: {MASS * g:.2f} N (4 motors)")
    print()
    
    print("POCHODNE STANU (powinny być ~0 dla równowagi):")
    print(f"  dvx/dt = {dx[0]:+.6f} m/s²")
    print(f"  dvz/dt = {dx[1]:+.6f} m/s²")
    print(f"  dω/dt  = {dx[2]:+.6f} rad/s²")
    print(f"  dX/dt  = {dx[3]:+.6f} m/s")
    print(f"  dZ/dt  = {dx[4]:+.6f} m/s")
    print(f"  dθ/dt  = {dx[5]:+.6f} rad/s")
    
    if len(x_hover) == 10:
        print(f"  dT1/dt = {dx[6]:+.6f} N/s")
        print(f"  dT2/dt = {dx[7]:+.6f} N/s")
        print(f"  dT3/dt = {dx[8]:+.6f} N/s")
        print(f"  dT4/dt = {dx[9]:+.6f} N/s")
    
    print()
    
    # Sprawdź czy suma sił jest bliska zeru
    suma_sil_z = -MASS * g + (MASS * g / 4) * 4
    print(f"Suma sił w osi Z: {suma_sil_z:.6f} N (powinno być ~0)")
    
    if abs(dx[1]) < 1e-6:
        print("✓ TEST ZALICZONY: Dron w równowadze!")
    else:
        print("✗ TEST NIEZALICZONY: Dron nie jest w równowadze!")
    
    print("\n" + "=" * 70)
    diagnostyka_sil(x_hover, u_hover)
