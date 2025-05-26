import numpy as np

# Te zmienne będą ustawiane globalnie w programie głównym
az_turbulence = 0.0
ax_wind = 0.0
az_wind = 0.0

def aa_rhs(x, t, u):
    """
    Opisuje dynamikę układu dla zadanych warunków (RHS układu ODE).

    Parameters:
        x : ndarray
            Wektor stanu (6- lub 8-elementowy)
        t : float
            Czas (niewykorzystywany bezpośrednio tutaj)
        u : ndarray
            Sterowania (2-elementowe)

    Returns:
        dx_dt : ndarray
            Pochodne stanu (też 6- lub 8-elementowe)

            dx/dt = f(x,u,t) - RHS
    """
    global az_turbulence, ax_wind, az_wind

    n = len(x)
    dx_dt = np.zeros(n)

    deg2rad = np.pi / 180.0
    g = 9.81         # grawitacja

    '''powierzchnia, masa, moment bezwładnosci drona'''
    S = 1.0          # powierzchnia referencyjna
    mass = 25.0
    Iy = 100         # moment bezwładności

    # Wiatry
    vx = x[0] + ax_wind
    vz = x[1] + az_wind

    # Kąt natarcia i prędkość całkowita
    alpha = np.arctan2(vz, vx)
    V = np.sqrt(vz**2 + vx**2)

    # Opory i siły aerodynamiczne
    CD_0 = 0.30
    CD = CD_0
    ro_0 = 1.225
    ro = ro_0 * (1.0 - abs(x[4]) / 44300.0)**4.256
    Q_dyn = 0.5 * ro * V**2
    D = Q_dyn * S * CD  # siła oporu
    L = 0.0             # brak siły nośnej w tym modelu

    G = mass * g  # siła ciężkości

    Th = 1.0  # wzmocnienie ciągu

    if n == 8:
        Thrust_1 = x[6]
        Thrust_2 = x[7]
    else:
        Thrust_1 = 0.5 * G + u[0]
        Thrust_2 = 0.5 * G + u[1]

    cm_q = -0.01  # tłumienie momentu
    Tau = 0.05    # stała czasowa silnika

    beta = 0.0 * deg2rad
    cb = np.cos(beta)
    sb = np.sin(beta)

    """
    -x_array - macierz stanu drona
    x[0] = v_x - predkosc x
    x[1] = v_z - predkosc z
    x[2] = omega - predkosc kątowa [1/s]
    x[3] = pozycja X
    x[4] = pozycja Z
    x[5] = theta - kat drona >0 - wznoszenie, <0 - nurkowanie
    x[6] = T1 filtr (np. model reakcji silnika)
    x[7] = T2 filtr
    
    dx_dt[0] - przyspiesczenie
    dx_dt[1] - przyspiesczenie
    dx_dt[2] - przyspiesczenie
    dx_dt[3] - predkosc
    dx_dt[4] - predkosc
    dx_dt[5] - kąt?
    
    x[6] = T1 filtr (np. model reakcji silnika)
    x[7] = T2 filtr
    
    Tau (τ) mówi, jak szybko silnik osiąga zadany ciąg [s]
    Th - współczynnik przeliczania inputu u na siłe caigu
    """
    # Równania ruchu
    dx_dt[0] = (-D * np.cos(alpha) + L * np.sin(alpha) - G * np.sin(x[5]) - Thrust_1 * sb + Thrust_2 * sb) / mass - x[2] * vz
    dx_dt[1] = (-D * np.sin(alpha) - L * np.cos(alpha) + G * np.cos(x[5]) - Thrust_1 * cb - Thrust_2 * cb) / mass + x[2] * vx + az_turbulence
    dx_dt[2] = (0.5 * (Thrust_2 * cb - Thrust_1 * cb) + cm_q * x[2]) / Iy
    dx_dt[3] = np.cos(x[5]) * vx + np.sin(x[5]) * vz
    dx_dt[4] = -np.sin(x[5]) * vx + np.cos(x[5]) * vz
    dx_dt[5] = x[2]

    # Modele silników (inercja, filtry pierwszego rzędu)
    if n == 8:
        dx_dt[6] = (1.0 / Tau) * (-x[6] + Th * u[0])
        dx_dt[7] = (1.0 / Tau) * (-x[7] + Th * u[1])

    return dx_dt
