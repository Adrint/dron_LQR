import numpy as np
from control_rhs import aa_rhs
from config import config


def aa_matrices_AB(x, t, u, az_turbulence):
    """
    Oblicza zlinearyzowane macierze układu A i B metodą różnic skończonych.

    Parametry:
    x : array_like
        Aktualny wektor stanu
    t : float
        Aktualny czas
    u : array_like
        Wektor sterowań
    n : int
        Wymiar wektora stanu
    m : int
        Wymiar wektora sterowań

    Zwraca:
    A : ndarray
        Zlinearyzowana macierz stanu (n x n)
    B : ndarray
        Zlinearyzowana macierz wejść (n x m)
    """

    n = config.n
    m = config.m

    del_val = 1.0e-6

    f0 = aa_rhs(x, t, u, az_turbulence)

    A = np.zeros((n, n))
    for j in range(n):
        dx = np.zeros(n)
        dx[j] = del_val
        A[:, j] = (aa_rhs(x + dx, t, u, az_turbulence) - f0) / del_val

    B = np.zeros((n, m))
    for j in range(m):
        du = np.zeros(m)
        du[j] = del_val
        B[:, j] = (aa_rhs(x, t, u + du, az_turbulence) - f0) / del_val

    return A, B