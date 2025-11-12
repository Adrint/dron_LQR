"""
Równania ruchu drona - UKŁAD NED (Z w dół)
"""

import numpy as np
from constants import (CD_0, RO_0, MASS, g, S, MOTOR_ARM_LENGTH, CM_Q, IY, TAU)


def aa_rhs(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """
    Równania ruchu drona w układzie NED (North-East-Down)
    
    State: [vx, vz, omega, X, Z, theta, T1, T2]
    - Z ujemne = wysoko nad ziemią
    - vz dodatnie = ruch w dół
    - theta dodatnie = nos w górę
    """

    n = len(x)
    dx_dt = np.zeros(n)

    vx = x[0]
    vz = x[1]
    omega = x[2]
    X = x[3]
    Z = x[4]
    theta = x[5]

    if n == 8:
        Thrust_1 = x[6]
        Thrust_2 = x[7]
    else:
        Thrust_1 = u[0]
        Thrust_2 = u[1]

    # Aerodynamika
    V = np.sqrt(vx**2 + vz**2)
    altitude = -Z
    
    ro = RO_0 * (1.0 - altitude / 44300.0)**4.256
    if ro < 0.0:
        ro = 0.0

    Q_dyn = 0.5 * ro * V**2

    if V > 0.001:
        D = Q_dyn * S * CD_0
        Dx = -D * (vx / V)
        Dz = -D * (vz / V)
    else:
        Dx = 0.0
        Dz = 0.0

    # Grawitacja w układzie ciała (NED)
    Gx = MASS * g * np.sin(theta)
    Gz = MASS * g * np.cos(theta)

    # Ciągi (działają w górę = przeciwnie do Z)
    Tx = 0.0
    Tz = -(Thrust_1 + Thrust_2)

    # Równania ruchu
    Fx_total = Dx + Gx + Tx
    Fz_total = Dz + Gz + Tz
    
    ax = (Fx_total / MASS) - omega * vz
    az = (Fz_total / MASS) + omega * vx + az_turbulence

    dx_dt[0] = ax
    dx_dt[1] = az

    # Dynamika obrotowa
    Mt = MOTOR_ARM_LENGTH * (Thrust_2 - Thrust_1)
    Mq = CM_Q * omega
    dx_dt[2] = (Mt + Mq) / IY

    # Transformacja do układu inercjalnego (NED)
    # Dla theta > 0 (nos w górę): vx > 0 → leci w górę i do przodu
    dx_dt[3] = np.cos(theta) * vx + np.sin(theta) * vz
    dx_dt[4] = -np.sin(theta) * vx + np.cos(theta) * vz

    dx_dt[5] = omega

    if n == 8:
        dx_dt[6] = (1.0 / TAU) * (u[0] - x[6])
        dx_dt[7] = (1.0 / TAU) * (u[1] - x[7])

    return dx_dt


def diagnostyka_sil(x, u, az_turbulence=0.0):
    """Diagnostyka sił - układ NED"""
    n = len(x)
    vx, vz, omega = x[0], x[1], x[2]
    X, Z, theta = x[3], x[4], x[5]
    
    altitude = -Z
    V = np.sqrt(vx**2 + vz**2)
    ro = RO_0 * (1.0 - altitude / 44300.0)**4.256
    Q_dyn = 0.5 * ro * V**2
    D = Q_dyn * S * CD_0
    
    if V > 0.001:
        Dx = -D * (vx / V)
        Dz = -D * (vz / V)
    else:
        Dx = 0.0
        Dz = 0.0
    
    if n == 8:
        T1, T2 = x[6], x[7]
    else:
        T1, T2 = u[0], u[1]
    
    Gx = MASS * g * np.sin(theta)
    Gz = MASS * g * np.cos(theta)
    
    print("=" * 70)
    print("DIAGNOSTYKA SIŁ (NED - Z w dół)")
    print("=" * 70)
    print(f"Wysokość h = {altitude:.3f} m (Z = {Z:.3f} m)")
    print(f"Prędkość V = {V:.3f} m/s (vx={vx:.3f}, vz={vz:.3f})")
    print(f"Kąt θ = {np.degrees(theta):.3f}°")
    print()
    
    print("SIŁY W OSI X CIAŁA:")
    print(f"  Opór Dx:        {Dx:+8.4f} N")
    print(f"  Grawitacja Gx:  {Gx:+8.4f} N")
    print(f"  Suma Fx:        {Dx + Gx:+8.4f} N")
    print(f"  Przyspieszenie: {(Dx + Gx)/MASS:+8.4f} m/s²")
    print()
    
    print("SIŁY W OSI Z CIAŁA:")
    print(f"  Opór Dz:        {Dz:+8.4f} N")
    print(f"  Grawitacja Gz:  {Gz:+8.4f} N")
    print(f"  Ciągi (w górę): {-(T1 + T2):+8.4f} N")
    print(f"  Suma Fz:        {Dz + Gz - (T1 + T2):+8.4f} N")
    print(f"  Przyspieszenie: {(Dz + Gz - (T1 + T2))/MASS:+8.4f} m/s²")
    print()
    
    print("MOMENTY:")
    print(f"  M_thrust: {MOTOR_ARM_LENGTH * (T2 - T1):+8.4f} N·m")
    print(f"  M_damp:   {CM_Q * omega:+8.4f} N·m")
    print("=" * 70)
    print()


if __name__ == "__main__":
    print("TEST RÓWNOWAGI - NED")
    print()
    
    # Test 1: Równowaga
    x_hover = np.array([0.0, 0.0, 0.0, 0.0, -2.0, 0.0, MASS*g/2, MASS*g/2])
    u_hover = np.array([MASS*g/2, MASS*g/2])
    
    dx = aa_rhs(x_hover, 0.0, u_hover)
    
    print(f"TEST 1: Dron w poziomie na h=2m (Z=-2m)")
    print(f"  dvx/dt = {dx[0]:+.6f} m/s² {'✓' if abs(dx[0])<1e-6 else '✗'}")
    print(f"  dvz/dt = {dx[1]:+.6f} m/s² {'✓' if abs(dx[1])<1e-6 else '✗'}")
    print()
    diagnostyka_sil(x_hover, u_hover)
    
    # Test 2: Pochylenie
    x_tilt = np.array([0.0, 0.0, 0.0, 0.0, -2.0, -5*np.pi/180, MASS*g/2, MASS*g/2])
    dx_tilt = aa_rhs(x_tilt, 0.0, u_hover)
    
    print(f"TEST 2: Dron pochylony θ=-5°")
    print(f"  ax = {dx_tilt[0]:+.6f} m/s² {'✓ przyspiesza' if dx_tilt[0]>0 else '✗'}")
    print()
    diagnostyka_sil(x_tilt, u_hover)
