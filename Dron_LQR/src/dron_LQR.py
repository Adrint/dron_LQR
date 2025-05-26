import numpy as np
import matplotlib.pyplot as plt
from rhs import aa_rhs  # prawa strona równań ruchu (ODE)
from trajectory import aa_trajectory  # wyznaczanie trajektorii referencyjnej
from euler import aa_euler  # metoda Eulera (moze byc zamiast RK45 ale mniej dokładna i niestablina)
from matrices_AB import aa_matrices_AB  # linearyzacja układu: wyznaczenie macierzy A, B
from lqr import lqr_with_cross_term  # implementacja regulatora LQR
from mdl import aa_mdl  # rysowanie modelu drona
from scipy.linalg import solve_continuous_are  # rozwiązywanie równania Riccatiego
from rk45 import aa_rk45  # solver RK45
# Globalne zmienne
from rhs import az_turbulence, ax_wind, az_wind  # zakłócenia środowiskowe

'''
Symulacja lotu drona:
-n = 6 - lot 
-n = 8 - lot uwzgledniajacy opoznienia silników
-dron leci 1 metr docelowo nad ziemią, h_flight = 1

-x_array - macierz stanu drona
x[0] = v_x - predkosc x
x[1] = v_z - predkosc z
x[2] = omega - predkosc kątowa [1/s]
x[3] = pozycja X
x[4] = pozycja Z
x[5] = theta - kat drona >0 - wznoszenie, <0 - nurkowanie
x[6] = T1 filtr (np. model reakcji silnika)
x[7] = T2 filtr

| Zmienna      | Wyrażenie na błąd | Dlaczego?                          |
| ------------ | ----------------- | ---------------------------------- |
| Prędkość `x` | `x[0] - Vx_proj`  | v - v_ref, jesli dodatni, dron leci za wolno       |
| Prędkość `z` | `x[1] - Vz_proj`  | j.w.                               |
| Wysokość `z` | `x[4] + z_ref`    | bo `x[4]` jest ujemną wysokością, jesli 0, dobra wysokosc, jesli ujemna to za nisko  |
| Kąt `theta`  | `x[5]`            | bo referencją jest 0 rad = poziomo |


'''
# Konfiguracja systemu
n = 6   # liczba zmiennych stanu bez filtrow silnika
m = 2   # liczba zmiennych sterowania, dwa silniki

# Parametry wysokościowe i zakłóceń
z0 = 2.0
h_flight = 1.0
c_turb = 1000.0
X_turb_1 = 1500.0
X_turb_2 = 2000.0

# Konwersje jednostek
rad2deg = 180.0 / np.pi
deg2rad = np.pi / 180.0

# Inicjalizacja stanu
if n == 6:
    x = np.array([0.0, 0.0, 0.0, 0.0, -z0, 0.0])
elif n == 8:
    x = np.array([0.0, 0.0, 0.0, 0.0, -z0, 0.0, 0.0, 0.0])

# Inicjalizacja sterowania
u = np.zeros((m,)) #jednowymiarowa tablica sterowania

# Parametry symulacji
Vel = 0.10  # zadana prędkość pozioma
dt = 0.01   # krok czasowy
t = 0.0     # czas początkowy

# Bufory na dane do wizualizacji
tp, gp, zp = [], [], []  # czas, teren, referencja
yp = []  # wektory stanu
up = []  # sterowania

# Główna pętla symulacji
for i in range(10000):
    if t >= 100.0:
        break

    '''Wyznaczenie trajektorii terenu; Dron docelowo ma lecieć  1 metr nad ziemią'''

    X = x[3]  # pozycja pozioma
    z_terr, alfa = aa_trajectory(X, Vel, dt)  # wygenerowanie terenu oraz kąta nachylenia zbocza
    Vx = Vel * np.cos(alfa)
    Vz = Vel * np.sin(alfa)
    z_ref = z_terr + h_flight  # docelowa wysokość lotu dronem nad terenem


    # Zapisywanie danych do wykresów
    tp.append(t)
    yp.append(x.copy())
    up.append(u.copy())
    gp.append(z_terr)
    zp.append(z_ref)

    # Macierze wag LQR

    R = np.eye(m) #macierze jednostkowa 2x2
    Q = np.eye(n) #macierze jednostkowa 6x6 lub 8x8

    Q[0, 0] = 1000.0  # duża kara za błąd prędkości X
    Q[1, 1] = 1000.0  # błąd prędkości Z
    Q[2, 2] = 0.1
    Q[3, 3] = 10.0  # błąd pozycji X
    Q[4, 4] = 100.0  # błąd wysokości Z
    Q[5, 5] = 1000.0  # orientacja (kąt)
    if n == 8:
        Q *= 1000  # zwiększenie wag przy dodatkowych stanach

    # Obliczenie błędu stanu względem trajektorii referencyjnej
    e = np.zeros(n)
    e[0] = x[0] - (np.cos(x[5]) * Vx + np.sin(x[5]) * Vz)  # różnica prędkości X
    e[1] = x[1] - (np.sin(x[5]) * Vx - np.cos(x[5]) * Vz)  # różnica prędkości Z
    e[2] = x[2]  # omega - predkosc kątowa [1/s]
    e[3] = 0.0   # ignorujemy błąd pozycji X
    e[4] = x[4] + z_ref  # błąd wysokości
    e[5] = x[5]  # kąt orientacji

    # Linearyzacja modelu
    A, B = aa_matrices_AB(aa_rhs, x, t, u, n, m)

    '''END'''


    # Rozwiązanie równania Riccatiego
    # P - koszt stanów
    # K to macierz optymalnego sterowania, najniszy koszt
    K, P = lqr_with_cross_term(A, B, Q, R)
    u = -K @ e  # jak mocno zaregować na macierz błedow

    # Ograniczenie sterowania
    umax = 10000.0
    u = np.clip(u, -umax, umax)

    # Zakłócenia atmosferyczne
    az_turbulence = 0.0
    ax_wind = 0.0
    az_wind = 0.0

    '''Losowe turbulencje z zakresu [-c_turb,c_turb]'''
    if X_turb_1 < X < X_turb_2:
        az_turbulence = c_turb * (1.0 - 2.0 * np.random.rand())

    # Integracja stanu metodą RK45
    x = aa_rk45(aa_rhs, x, t, dt, u)

    # Rysowanie co 20 kroków
    if i % 20 == 0:
        v_x = Vx
        v_z = Vz
        V = np.linalg.norm(x[:2])  # rzeczywista prędkość
        e_v = Vel - V
        teta = x[5] * rad2deg
        alt = -x[4]
        T1, T2 = u
        gamma = np.arctan2(np.sin(x[5]) * x[0] - np.cos(x[5]) * x[1],
                           np.cos(x[5]) * x[0] + np.sin(x[5]) * x[1]) * rad2deg

        xs, zs = aa_mdl(x[3], -x[4], x[5], 0.5)  # model drona

        plt.figure(1, figsize=(12, 7))
        txt = f"t={t:.3f} V={V:.5f} v_x={v_x:.5f} v_z={v_z:.5f} teta={teta:.5f} alfa={alfa*rad2deg:.5f} ||| u1={T1:.4f} u2={T2:.4f} ||| e_v={e_v:.5f} e(z)={e[4]:.5f}"
        plt.clf()
        plt.plot([x[3] for x in yp], zp, 'r', label="z_ref")
        plt.plot([x[3] for x in yp], gp, 'g', label="terrain")
        plt.plot([x[3] for x in yp], [-x[4] for x in yp], 'b', label="flight path")
        plt.plot(x[3], -x[4], 'bo', label="current position")
        plt.plot(xs[:5], zs[:5], 'k', linewidth=3, label="aircraft shape")
        plt.title(txt)
        plt.axis([0, 5, 0, 8])
        plt.legend()
        plt.pause(0.01)

    t += dt  # aktualizacja czasu

# Pokazanie końcowego wykresu
plt.show()