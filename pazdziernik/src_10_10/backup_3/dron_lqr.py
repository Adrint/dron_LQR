import numpy as np
import math
import matplotlib.pyplot as plt
from rk45 import aa_rk45
from matrices import aa_matrices_AB
from model_draw import aa_mdl
from lqr import lqr_m
from trajectory import generate_reference_profile

# global variables
from constants import (z0, Vel, X_turb_1, X_turb_2, c_turb, MASS, g)

# Dimensions
n = 8  # With engines
m = 2  # dimension of u

u = np.array([0.0, 0.0])
dt = 0.01
t = 0.0

# Initial state
if n == 6:
    x = np.array([0.0, 0.0, 0.0, 0.0, -z0, 0.0])
elif n == 8:
    x = np.array([0.0, 0.0, 0.0, 0.0, -z0, 0.0, 0.0, 0.0])

# --- Stałe referencje:
X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)

# Storage for plotting
tp = []
yp = []
up = []
gp = []
zp = []

# Przechowywanie danych do rysowania tła
terrain_x = []
terrain_z = []
ref_z = []

plt.ion()
fig = plt.figure(1, figsize=(11, 6.4))
plt.plot(X_ref_all, Z_ref_all, 'r', label='Reference')
plt.plot(X_ref_all, Z_terr_all, 'g', label='Terrain')

plt.legend()
plt.axis([0, 50.0, 0.0, 20.0])
plt.draw()
plt.pause(1)

x_ref = 0.0

# ======================== FIZYCZNE LIMITY SIŁY CIĄGU ========================
T_baseline = MASS * g / 2.0  # Ciąg baseline dla jednego silnika
T_max_absolute = 300.0  # Maksymalny ciąg fizycznie możliwy
T_min_absolute = 0.0  # Minimalny ciąg (silnik nie może pracować wstecz)
# ===========================================================================

for i in range(10000):
    if t >= 5.0:
        break

    X = x[3]  # aktualna pozycja drona

    # === 1. Znajdź punkt referencyjny najbliżej x_ref ===
    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))

    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]

    # === 2. Składowe prędkości w kierunku trajektorii ===
    Vx = Vel * np.cos(alfa)
    Vz = Vel * np.sin(alfa)

    # === 3. Zaktualizuj pozycję referencyjną na podstawie kąta nachylenia ===
    x_ref += Vel * np.cos(alfa) * dt
    vx_ref = np.cos(x[5]) * Vx + np.sin(x[5]) * Vz
    vz_ref = np.sin(x[5]) * Vx - np.cos(x[5]) * Vz

    # === 4. Zbieraj dane do wykresów i analizy ===
    terrain_x.append(X)
    terrain_z.append(z_terr)
    ref_z.append(z_ref)

    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())

    # ======================== POPRAWKA: TUNING MACIERZY Q ========================
    # MNIEJ AGRESYWNE WAGI - kontroler nie będzie szalony!
    R = np.eye(m) * 1.0  # Większa kara na sterowanie (łagodniej)

    Q = np.eye(n)
    Q[0, 0] = 10.0  # vx - zmniejszone z 1000
    Q[1, 1] = 10.0  # vz - zmniejszone z 1000
    Q[2, 2] = 0.1  # omega - zmniejszone z 1
    Q[3, 3] = 1.0  # X - zmniejszone z 10
    Q[4, 4] = 10.0  # Z - zmniejszone z 100 (NAJWAŻNIEJSZE!)
    Q[5, 5] = 1.0  # theta - zmniejszone z 10

    if n == 8:
        Q = 100 * Q  # Zmniejszone z 10000!
    # ========================================================================

    e = np.zeros(n)
    e[0] = x[0] - vx_ref
    e[1] = x[1] - vz_ref
    e[2] = x[2] - 0
    e[3] = x[3] - x_ref
    e[4] = x[4] - (-z_ref)
    e[5] = x[5]

    # ----------------------------
    # Diagnostyka
    # ----------------------------
    omega_ref = 0.0
    T1 = u[0]
    T2 = u[1]
    T = T1 + T2
    theta = x[5]
    theta_ref = 0.0
    Fx = T * np.sin(theta)
    Fz = T * np.cos(theta)

    # if i < 200:
    print("========== DIAGNOSTYKA ==========")
    print(f"{'Czas':<10} | {t:9.2f}sek |")
    print(f"{'Zmienna':<10} | {'Referencja':>12} | {'Aktualna':>12} | {'Błąd':>12} | Opis")
    print("-" * 80)

    print(f"{'vx':<10} | {vx_ref:12.5f} | {x[0]:12.5f} | {e[0]:12.5f} | "
          f"prędkość wzdłuż kadłuba (dodatnia - do przodu)")

    print(f"{'vz':<10} | {-vz_ref:12.5f} | {-x[1]:12.5f} | {-e[1]:12.5f} | "
          f"wznoszenie/opadanie (dodatnia - w górę)")

    print(f"{'omega':<10} | {omega_ref:12.5f} | {x[2]:12.5f} | {e[2]:12.5f} | "
          f"prędkość obrotu wokół środka (rad/s)")

    print(f"{'X':<10} | {x_ref:12.5f} | {x[3]:12.5f} | {e[3]:12.5f} | "
          f"pozycja pozioma wzdłuż trajektorii")

    print(f"{'Z':<10} | {z_ref:12.5f} | {-x[4]:12.5f} | {-e[4]:12.5f} | "
          f"wysokość nad terenem (dodatnia)")

    print(
        f"{'theta':<10} | {math.degrees(theta_ref):12.5f} | {math.degrees(x[5]):12.5f} | {math.degrees(e[5]):12.5f} | "
        f"nachylenie drona (deg)")

    print(f"\n{'T1':<10} = {T1:12.5f}   | ciąg lewego silnika")
    print(f"{'T2':<10} = {T2:12.5f}   | ciąg prawego silnika")
    print(f"{'T (sum)':<10} = {T:12.5f}   | łączna siła ciągu")

    print(f"{'Fx':<10} = {Fx:12.5f}   | składowa pozioma (wpływa na vx)")
    print(f"{'Fz':<10} = {Fz:12.5f}   | składowa pionowa (wpływa na vz)")
    print("==================================\n")

    A, B = aa_matrices_AB("rhs", x, t, u, n, m)
    K, P = lqr_m(A, B, Q, R)

    # Obliczamy perturbacje z LQR
    u_pert = -K @ e

    # Obliczamy rzeczywisty ciąg dla każdego silnika
    T1_desired = T_baseline + u_pert[0]
    T2_desired = T_baseline + u_pert[1]

    # OGRANICZENIE FIZYCZNE
    T1 = np.clip(T1_desired, T_min_absolute, T_max_absolute)
    T2 = np.clip(T2_desired, T_min_absolute, T_max_absolute)

    u = np.array([T1, T2])

    print(f"DEBUG: u_pert[0]={u_pert[0]:10.5f}, u_pert[1]={u_pert[1]:10.5f}")
    print(f"DEBUG: T1_desired={T1_desired:10.5f}, T2_desired={T2_desired:10.5f}")
    print(f"DEBUG: T1_actual={T1:10.5f}, T2_actual={T2:10.5f} (po clip)")
    print(f"DEBUG: T_baseline={T_baseline:10.5f}, T_sum={T1 + T2:10.5f}")
    print()

    az_turbulence = 0.0
    ax_wind = 0.0
    az_wind = 0.0

    if X > X_turb_1 and X < X_turb_2:
        az_turbulence = c_turb * (1.0 - 2.0 * np.random.rand())
        ax_wind = 0.0
        az_wind = 0.0

    x = aa_rk45("rhs", x, t, dt, u, az_turbulence, ax_wind, az_wind)

    if i % 20 == 0:
        v_x = Vx
        v_z = Vz
        V = np.sqrt(x[0] ** 2 + x[1] ** 2)
        e_v = Vel - V
        teta = math.degrees(x[5])
        alt = -x[4]
        T1 = u[0]
        T2 = u[1]

        xs, zs = aa_mdl(x[3], -x[4], x[5], 0.50)

        for line in plt.gca().lines[2:]:
            line.remove()

        yp_array = np.array(yp)

        plt.plot([y[3] for y in yp], [-y[4] for y in yp], 'b', label='Flight path')
        plt.plot(yp_array[i, 3], -yp_array[i, 4], 'bo', markersize=8)
        plt.plot(xs[:5], zs[:5], 'k', linewidth=3)

        txt = (f't={t:7.3f} V={V:10.5f} m/s v_x={v_x:10.5f} v_z={v_z:10.5f} '
               f'teta={teta:10.5f} alfa={math.degrees(alfa):10.5f} '
               f'||| u1={T1:7.4f} u2={T2:7.4f} ||| e_v={e_v:10.5f} e(z)={e[4]:10.5f}')
        plt.title(txt, fontsize=12)

        plt.legend()
        plt.draw()
        plt.pause(0.001)

    t = t + dt

plt.ioff()
plt.show()