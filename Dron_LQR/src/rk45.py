import numpy as np

def aa_rk45(RHS, x, t, dt, u):
    """
    Realizuje jedno przejście metody Runge-Kutta-Fehlberga 4(5) dla układu ODE.

    Parametry:
        RHS : funkcja opisująca prawą stronę ODE (dx/dt = f(x,t,u))
        x : aktualny stan układu
        t : aktualny czas
        dt : krok czasowy
        u : wektor wejść sterujących

    Zwraca:
        y : nowy stan układu po kroku czasowym dt
    """
    y0 = RHS(x, t, u)  # pierwszy krok RK

    # Kroki pośrednie z różnymi wagami i czasami
    t1 = t + dt * 0.25
    vec = x + dt * 0.25 * y0
    y1 = RHS(vec, t1, u)

    t2 = t + dt * (3.0 / 8.0)
    vec = x + dt * ((3.0 / 32.0) * y0 + (9.0 / 32.0) * y1)
    y2 = RHS(vec, t2, u)

    t3 = t + dt * (12.0 / 13.0)
    vec = x + dt * ((1932.0 / 2197.0) * y0 - (7200.0 / 2197.0) * y1 + (7296.0 / 2197.0) * y2)
    y3 = RHS(vec, t3, u)

    t4 = t + dt
    vec = x + dt * ((439.0 / 216.0) * y0 - 8.0 * y1 + (3680.0 / 513.0) * y2 - (845.0 / 4104.0) * y3)
    y4 = RHS(vec, t4, u)

    t5 = t + dt * 0.5
    vec = x + dt * (
        -(8.0 / 27.0) * y0
        + 2.0 * y1
        - (3544.0 / 2565.0) * y2
        + (1859.0 / 4104.0) * y3
        - (11.0 / 40.0) * y4
    )
    y5 = RHS(vec, t5, u)

    # Ostateczna kombinacja wag Runge-Kutta 4(5) do wyznaczenia nowego stanu
    y = x + dt * (
        (16.0 / 135.0) * y0
        + (6656.0 / 12825.0) * y2
        + (28561.0 / 56430.0) * y3
        - (9.0 / 50.0) * y4
        + (2.0 / 55.0) * y5
    )

    return y
