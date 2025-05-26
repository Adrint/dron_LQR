import numpy as np
from numpy.linalg import eig, matrix_rank, norm
from scipy.linalg import eig as scipy_eig

def lqr_m(a, b, q, r, nn=None):
    """
    LQR regulator (ciągły czas): oblicza macierz wzmocnień K oraz macierz Riccatiego S.

    Parametry:
        a, b : macierze stanu i sterowania układu liniowego
        q, r : macierze wag dla błędów stanu i sterowania
        nn : (opcjonalnie) macierz sprzężenia krzyżowego stanu z wejściem

    Zwraca:
        k : optymalna macierz wzmocnień sterowania
        s : macierz Riccatiego (rozwiązanie ARE)
    """

    m, n = a.shape  # rozmiary macierzy A
    nb = b.shape[1]  # liczba wejść

    # Sprawdzenie poprawności rozmiarów macierzy Q i R
    if q.shape != (m, n):
        raise ValueError("A and Q must be the same size")
    if r.shape[0] != r.shape[1] or b.shape[1] != r.shape[0]:
        raise ValueError("B and R must be consistent")

    # Obsługa przypadku z macierzą NN (sprzężenie krzyżowe)
    if nn is not None:
        if nn.shape != (m, r.shape[0]):
            raise ValueError("N must be consistent with Q and R")
        q = q - nn @ np.linalg.inv(r) @ nn.T
        a = a - b @ np.linalg.inv(r) @ nn.T
    else:
        nn = np.zeros((m, nb))

    # Sprawdzenie czy Q jest symetryczne i dodatnio półokreślone
    if np.any(np.linalg.eigvalsh(q) < -1e-12) or norm(q.T - q, 1) / norm(q, 1) > np.finfo(float).eps:
        raise ValueError("Q must be symmetric and positive semi-definite")

    # Sprawdzenie czy R jest symetryczne i dodatnio określone
    if np.any(np.linalg.eigvalsh(r) <= 0) or norm(r.T - r, 1) / norm(r, 1) > np.finfo(float).eps:
        raise ValueError("R must be symmetric and positive definite")

    # Tworzenie macierzy Hamiltona dla układu (do wyznaczenia rozwiązania ARE)
    H = np.block([
        [a,            b @ np.linalg.inv(r) @ b.T],
        [-q,           -a.T]
    ])

    # Rozkład na wartości własne i wektory własne
    vals, vecs = scipy_eig(H)
    real_parts = np.real(vals)
    idx = np.argsort(real_parts)  # sortowanie wg części rzeczywistych

    # Sprawdzenie czy można poprawnie podzielić bieguny (stabilność)
    if not (real_parts[n-1] < -1e-15 and real_parts[n] > 1e-15):
        print("Can't order eigenvalues, (A,B) may be uncontrollable. Checking rank...")
        # Sprawdzenie sterowalności
        C = b
        for i in range(1, n):
            C = np.hstack((C, np.linalg.matrix_power(a, i) @ b))
        rank_C = matrix_rank(C)
        if rank_C < n:
            raise ValueError(f"rank_C = {rank_C} < {n}, system (A,B) is uncontrollable.")
        else:
            raise ValueError("rank_C = n (OK), but eigenvalue ordering failed.")

    # Ekstrakcja podprzestrzeni stabilnej
    chi = vecs[:n, idx[:n]]
    lam = vecs[n:, idx[:n]]

    # Wyznaczenie macierzy Riccatiego S
    s = -np.real(lam @ np.linalg.inv(chi))

    # Obliczenie macierzy sterowania LQR
    k = np.linalg.inv(r) @ (nn.T + b.T @ s)

    return k, s

def lqr_with_cross_term(A, B, Q, R, N=None):
    if N is None:
        N = np.zeros((A.shape[0], B.shape[1]))  # N = 0, klasyczny LQR

    R_inv = np.linalg.inv(R)
    Q_mod = Q - N @ R_inv @ N.T
    A_mod = A - B @ R_inv @ N.T

    from scipy.linalg import solve_continuous_are
    S = solve_continuous_are(A_mod, B, Q_mod, R)
    K = R_inv @ (N.T + B.T @ S)

    return K, S
