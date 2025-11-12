import numpy as np
from constants import MASS, g

# Test prostego przypadku
print("TEST RÓWNAŃ RUCHU\n")
print("="*60)

# Stan: dron pochylony lekko do przodu
theta = -0.04 * np.pi/180  # -0.04 stopnia w radianach
vx = 0.0
vz = 0.0
omega = 0.0

print(f"Pochylenie: theta = {np.degrees(theta):.4f}°")
print(f"Prędkości początkowe: vx={vx}, vz={vz}")
print()

# Siły w układzie ciała
print("SIŁY W UKŁADZIE CIAŁA:")
print("-"*40)

# Grawitacja w układzie ciała
Fx_grav = -MASS * g * np.sin(theta)
Fz_grav = MASS * g * np.cos(theta)

print(f"Składowa X grawitacji: {Fx_grav:.6f} N")
print(f"Składowa Z grawitacji: {Fz_grav:.6f} N")
print()

# Ciągi (równe)
T1 = MASS * g / 2
T2 = MASS * g / 2
Fz_thrust = -(T1 + T2)

print(f"Ciąg T1: {T1:.2f} N")
print(f"Ciąg T2: {T2:.2f} N")
print(f"Suma ciągów (w osi Z ciała): {Fz_thrust:.2f} N")
print()

# Przyspieszenia w układzie ciała
ax = Fx_grav / MASS
az = (Fz_grav + Fz_thrust) / MASS

print("PRZYSPIESZENIA W UKŁADZIE CIAŁA:")
print("-"*40)
print(f"ax = {ax:.6f} m/s²")
print(f"az = {az:.6f} m/s²")
print()

# Po czasie dt
dt = 0.01
vx_new = vx + ax * dt
vz_new = vz + az * dt

print(f"Po czasie dt={dt}s:")
print(f"vx_new = {vx_new:.6f} m/s")
print(f"vz_new = {vz_new:.6f} m/s")
print()

# Transformacja do układu inercjalnego
vX_inertial = np.cos(theta) * vx_new + np.sin(theta) * vz_new
vZ_inertial = -np.sin(theta) * vx_new + np.cos(theta) * vz_new

print("PRĘDKOŚCI W UKŁADZIE INERCJALNYM:")
print("-"*40)
print(f"vX = cos({np.degrees(theta):.4f}°)*{vx_new:.6f} + sin({np.degrees(theta):.4f}°)*{vz_new:.6f}")
print(f"vX = {vX_inertial:.6f} m/s")
print(f"vZ = {vZ_inertial:.6f} m/s")
print()

# Pozycje po dt
X_new = vX_inertial * dt
Z_new = vZ_inertial * dt

print(f"Zmiana pozycji:")
print(f"ΔX = {X_new:.8f} m")
print(f"ΔZ = {Z_new:.8f} m")
print()

print("="*60)
print("WNIOSEK:")
print("="*60)
if abs(vx_new) > 0.0001:
    print(f"Dron przyspiesza do przodu w układzie ciała z ax={ax:.6f} m/s²")
    print(f"To daje prędkość vx={vx_new:.6f} m/s po {dt}s")
    print(f"Przy kącie {np.degrees(theta):.4f}° to przekłada się na ruch poziomy")
    print(f"vX_inertial = {vX_inertial:.6f} m/s")
else:
    print("Dron NIE przyspiesza do przodu")

print()
print("Sprawdźmy czy składowa grawitacji jest poprawnie obliczona:")
print(f"sin({np.degrees(theta):.4f}°) = {np.sin(theta):.8f}")
print(f"Siła Fx_grav = -MASS * g * sin(theta) = -{MASS} * {g} * {np.sin(theta):.8f}")
print(f"Siła Fx_grav = {Fx_grav:.6f} N")
print(f"Przyspieszenie ax = Fx_grav/MASS = {Fx_grav/MASS:.8f} m/s²")
