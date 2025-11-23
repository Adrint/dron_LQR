import numpy as np
from control_rhs import aa_rhs

def aa_rk45(x, t, dt, u, az_turbulence):
    """
    Metoda Rungego–Kutty rzędu 4–5 do numerycznej integracji równań ruchu.

    Parametry:
    x : array_like
        Aktualny wektor stanu
    t : float
        Aktualny czas
    dt : float
        Krok czasowy integracji
    u : array_like
        Wektor sterowań
    az_turbulence : float
        Pionowe przyspieszenie turbulencyjne
    ax_wind : float
        Poziome przyspieszenie od wiatru
    az_wind : float
        Pionowe przyspieszenie od wiatru

    Zwraca:
    y : ndarray
        Zaktualizowany wektor stanu po wykonaniu kroku integracji
    """

    y = np.zeros(len(x))

    y0 = aa_rhs(x, t, u, az_turbulence)

    t1 = t + dt * 0.25
    vec = x + dt * 0.25 * y0
    y1 = aa_rhs(vec, t1, u, az_turbulence)

    t2 = t + dt * (3.0 / 8.0)
    vec = x + dt * ((3.0 / 32.0) * y0 + (9.0 / 32.0) * y1)
    y2 = aa_rhs(vec, t2, u, az_turbulence)

    t3 = t + dt * (12.0 / 13.0)
    vec = x + dt * ((1932.0 / 2197.0) * y0 + (-7200.0 / 2197.0) * y1 + (7296.0 / 2197.0) * y2)
    y3 = aa_rhs(vec, t3, u, az_turbulence)

    t4 = t + dt
    vec = x + dt * ((439.0 / 216.0) * y0 + (-8.0) * y1 + (3680.0 / 513.0) * y2 + (-845.0 / 4104.0) * y3)
    y4 = aa_rhs(vec, t4, u, az_turbulence)

    t5 = t + dt * (1.0 / 2.0)
    vec = x + dt * (
                -(8.0 / 27.0) * y0 + 2.0 * y1 + (-3544.0 / 2565.0) * y2 + (1859.0 / 4104.0) * y3 + (-11.0 / 40.0) * y4)
    y5 = aa_rhs(vec, t5, u, az_turbulence)

    y = x + dt * ((16.0 / 135.0) * y0 + (6656.0 / 12825.0) * y2 + (28561.0 / 56430.0) * y3 + (-9.0 / 50.0) * y4 + (
                2.0 / 55.0) * y5)

    return y