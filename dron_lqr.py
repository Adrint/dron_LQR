import numpy as np
import matplotlib.pyplot as plt
from rhs import aa_rhs
from rk45 import aa_rk45
from euler import aa_euler
from matrices import aa_matrices_AB
from trajectory import aa_trajectory
from model_draw import aa_mdl
from lqr import lqr_m

# Global variables
az_turbulence = 0.0
ax_wind = 0.0
az_wind = 0.0

# Dimensions
n = 8   # With engines
m = 2   # dimension of u
z0 = 2.0
h_flight = 1.0  # Over the terrain
c_turb = 1000.0
X_turb_1 = 1500.0
X_turb_2 = 2000.0

rad2deg = 180 / np.pi
deg2rad = np.pi / 180

# Initial state
if n == 6:
    x = np.array([0.0, 0.0, 0.0, 0.0, -z0, 0.0])
elif n == 8:
    x = np.array([0.0, 0.0, 0.0, 0.0, -z0, 0.0, 0.0, 0.0])

u = np.array([0.0, 0.0])
Vel = 0.10
dt = 0.01
t = 0.0

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

# Generuj od razu dane dla terenu i referencji na potrzeby tła
X_preview = np.linspace(0, 5, 1000)  # Zakres X do podgladu
terrain_profile = []
reference_profile = []

for X in X_preview:
    z_terr, _ = aa_trajectory(X, Vel, dt)
    terrain_profile.append(z_terr)
    reference_profile.append(z_terr + h_flight)

# Rysuj tło raz: Reference i Terrain
plt.plot(X_preview, reference_profile, 'r', label='Reference')
plt.plot(X_preview, terrain_profile, 'g', label='Terrain')
plt.legend()
plt.axis([0, 5.0, 0.0, 8.0])
plt.draw()
plt.pause(1)

for i in range(10000):
    if t >= 100.0:
        break

    X = x[3]
    Z0 = 5.0

    z_terr, alfa = aa_trajectory(X, Vel, dt)
    Vx = Vel * np.cos(alfa)
    Vz = Vel * np.sin(alfa)
    z_ref = z_terr + h_flight

    # Zbieraj dane do tła (raz na początku)
    terrain_x.append(X)
    terrain_z.append(z_terr)
    ref_z.append(z_ref)

    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())
    gp.append(z_terr)
    zp.append(z_ref)

    x_ref = X

    R = np.eye(m)
    Q = np.eye(n)
    Q[0, 0] = 1000.0
    Q[1, 1] = 1000.0
    Q[2, 2] = 0.1
    Q[3, 3] = 10.0
    Q[4, 4] = 100.0
    Q[5, 5] = 1.0e+03

    if n == 8:
        Q = 10000 * Q

    e = np.zeros(n)
    e[0] = x[0] - (np.cos(x[5]) * Vx + np.sin(x[5]) * Vz)
    e[1] = x[1] - (np.sin(x[5]) * Vx - np.cos(x[5]) * Vz)
    e[2] = x[2] - 0
    e[3] = x[3] - x_ref
    e[4] = x[4] - (-z_ref)
    e[5] = x[5] - 0.0

    # ----------------------------
    # Diagnostyka - wartosci referencyjne, aktualne, blad
    # ----------------------------
    vx_ref = np.cos(x[5]) * Vx + np.sin(x[5]) * Vz
    vz_ref = np.sin(x[5]) * Vx - np.cos(x[5]) * Vz
    omega_ref = 0.0
    theta_ref = 0.0

    print("========== DIAGNOSTYKA ==========")
    print(f"{'Zmienna':<10} | {'Referencja':>12} | {'Aktualna':>12} | {'Blad':>12}")
    print("-" * 50)
    print(f"{'vx':<10} | {vx_ref:12.5f} | {x[0]:12.5f} | {e[0]:12.5f}")
    print(f"{'vz':<10} | {vz_ref:12.5f} | {x[1]:12.5f} | {e[1]:12.5f}")
    print(f"{'omega':<10} | {omega_ref:12.5f} | {x[2]:12.5f} | {e[2]:12.5f}")
    print(f"{'X':<10} | {x_ref:12.5f} | {x[3]:12.5f} | {e[3]:12.5f}")
    print(f"{'Z':<10} | {-z_ref:12.5f} | {x[4]:12.5f} | {e[4]:12.5f}")
    print(f"{'theta':<10} | {theta_ref:12.5f} | {x[5]:12.5f} | {e[5]:12.5f}")
    print("==================================\n")

    A, B = aa_matrices_AB("rhs", x, t, u, n, m)
    K, P = lqr_m(A, B, Q, R)
    u = -K @ e

    umax = 10000.0
    u[0] = np.clip(u[0], -umax, umax)
    u[1] = np.clip(u[1], -umax, umax)

    az_turbulence = 0.0
    ax_wind = 0.0
    az_wind = 0.0

    if X > X_turb_1 and X < X_turb_2:
        az_turbulence = c_turb * (1.0 - 2.0 * np.random.rand())
        ax_wind = 0.0
        az_wind = 0.0

    x = aa_rk45("rhs", x, t, dt, u, az_turbulence, ax_wind, az_wind)

    if i % 20 == 0:
        xl = 0
        xp_val = 5.0
        zl = 0.0
        zu = 8.0
        v_x = Vx
        v_z = Vz
        V = np.sqrt(x[0] ** 2 + x[1] ** 2)
        e_v = Vel - V
        teta = x[5] * rad2deg
        alt = -x[4]
        T1 = u[0]
        T2 = u[1]
        gamma = np.arctan2(np.sin(x[5]) * x[0] - np.cos(x[5]) * x[1],
                           np.cos(x[5]) * x[0] + np.sin(x[5]) * x[1]) * rad2deg

        xs, zs = aa_mdl(x[3], -x[4], x[5], 0.50)

        # Usuwanie tylko elementów dynamicznych (poprzednich warstw lotu i drona)
        for line in plt.gca().lines[2:]:  # zachowaj 2 pierwsze linie: Reference i Terrain
            line.remove()

        yp_array = np.array(yp)

        # === Rysowanie dynamiczne ===
        plt.plot([y[3] for y in yp], [-y[4] for y in yp], 'b', label='Flight path')
        plt.plot(yp_array[i, 3], -yp_array[i, 4], 'bo', markersize=8)  # pozycja drona
        plt.plot(xs[:5], zs[:5], 'k', linewidth=3)  # model drona

        txt = (f't={t:7.3f} V={V:10.5f} m/s v_x={v_x:10.5f} v_z={v_z:10.5f} '
               f'teta={teta:10.5f} alfa={alfa * rad2deg:10.5f} '
               f'||| u1={T1:7.4f} u2={T2:7.4f} ||| e_v={e_v:10.5f} e(z)={e[4]:10.5f}')

        plt.title(txt, fontsize=12)
        plt.axis([xl, xp_val, zl, zu])
        plt.legend()
        plt.draw()
        plt.pause(0.001)

    t = t + dt

plt.ioff()
plt.show()
