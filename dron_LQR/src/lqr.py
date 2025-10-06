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
        [a, b @ np.linalg.inv(r) @ b.T],
        [-q, -a.T]
    ])

    # Rozkład na wartości własne i wektory własne
    vals, vecs = scipy_eig(H)
    real_parts = np.real(vals)

    # Tolerancja dla wartości na osi urojonej
    tol = 1e-8

    # Podział wartości własnych na 3 grupy:
    # 1. Stabilne: Re < -tol
    # 2. Na osi urojonej: |Re| <= tol
    # 3. Niestabilne: Re > tol

    stable_idx = []  # Re < -tol
    marginal_idx = []  # |Re| <= tol (na osi urojonej)
    unstable_idx = []  # Re > tol

    for i in range(len(vals)):
        if real_parts[i] < -tol:
            stable_idx.append(i)
        elif real_parts[i] > tol:
            unstable_idx.append(i)
        else:
            marginal_idx.append(i)

    # Posortuj każdą grupę
    stable_idx = sorted(stable_idx, key=lambda i: real_parts[i])
    unstable_idx = sorted(unstable_idx, key=lambda i: real_parts[i])

    # Dla wartości marginalnych: sortuj po części urojonej
    # Ujemne części urojone → stabilne, dodatnie → niestabilne
    marginal_idx = sorted(marginal_idx, key=lambda i: np.imag(vals[i]))

    # Oblicz ile potrzebujemy do stabilnej podprzestrzeni
    needed_stable = n - len(stable_idx)

    if needed_stable < 0:
        raise ValueError(f"Za dużo stabilnych wartości własnych: {len(stable_idx)} > {n}")

    if needed_stable > len(marginal_idx):
        raise ValueError(f"Za mało wartości własnych do wypełnienia stabilnej podprzestrzeni")

    # Przypisz pierwsze 'needed_stable' wartości marginalnych do stabilnych
    stable_marginal = marginal_idx[:needed_stable]
    unstable_marginal = marginal_idx[needed_stable:]

    # Złóż pełne listy indeksów
    idx_stable = stable_idx + stable_marginal
    idx_unstable = unstable_marginal + unstable_idx

    # Połącz w jedną posortowaną listę
    idx = idx_stable + idx_unstable

    print(f"\n=== Podział wartości własnych ===")
    print(f"Stabilne ({len(idx_stable)}):")
    for i in idx_stable:
        print(f"  λ = {real_parts[i]:+.6e} {np.imag(vals[i]):+.6e}j")
    print(f"Niestabilne ({len(idx_unstable)}):")
    for i in idx_unstable:
        print(f"  λ = {real_parts[i]:+.6e} {np.imag(vals[i]):+.6e}j")
    print(f"====================================\n")

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