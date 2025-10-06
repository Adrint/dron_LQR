import numpy as np

def aa_matrices_AB(rhs, x, t, u, n, m, eps=1e-5):
    """
    Numeryczne wyznaczenie macierzy A i B dla układu zdefiniowanego przez rhs.

    rhs : funkcja f(x,t,u)
    x   : punkt pracy (wektor stanu)
    t   : czas
    u   : wejście sterujące
    n   : liczba zmiennych stanu
    m   : liczba wejść
    eps : mała wartość do różniczkowania numerycznego

    Zwraca:
        A, B : macierze równań stanu liniowego

        Macierz A to wpływ poprzedniego stanu drona na kolejne
        Macierz B to macierz wpływu silnikow na lot drona 6x2 lub 8x2 bo 2 silniki

        Linearyzacja fx(x,t,u) do Ax + Bu
    """
    A = np.zeros((n, n))
    B = np.zeros((n, m))
    fx = rhs(x, t, u) #stan drona (pochodne po czasie)

    # Aproksymacja pochodnych ∂f/∂x (macierz A)
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps
        f1 = rhs(x + dx, t, u)
        A[:, i] = (f1 - fx) / eps

    # Aproksymacja pochodnych ∂f/∂u (macierz B)
    for j in range(m):
        du = np.zeros(m)
        du[j] = eps
        f1 = rhs(x, t, u + du)
        B[:, j] = (f1 - fx) / eps

    return A, B
