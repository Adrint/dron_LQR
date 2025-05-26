import numpy as np

def aa_euler(rhs_func, x, t, dt, u):
    """
    Aproksymacja Eulera ze wzmocnioną dokładnością (quasi-implicitna) dla układów dynamicznych.

    Parameters:
        rhs_func : funkcja zwracająca pochodne stanu (RHS systemu)
        x        : aktualny wektor stanu (numpy array)
        t        : czas
        dt       : krok czasowy
        u        : wejście sterujące

    Returns:
        y        : nowy wektor stanu po czasie dt
    """

    c_bet = 0.90
    del_state = 1.0e-6
    coef = -c_bet / del_state
    n = len(x)

    y0 = rhs_func(x, t, u)
    Jacob = np.zeros((n, n))

    for i in range(n):
        x_perturbed = x.copy()
        x_perturbed[i] += del_state
        vec = rhs_func(x_perturbed, t, u)
        Jacob[:, i] = coef * (vec - y0)

    Jacob += np.eye(n) / dt
    dx = np.linalg.solve(Jacob, y0)
    y = x + dx

    return y

