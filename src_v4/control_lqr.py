import numpy as np
from scipy import linalg
import warnings

def lqr_m(a, b, q, r, nn=None):
    """
    Projekt regulatora LQR (Linear Quadratic Regulator) dla układów w czasie ciągłym.

    Oblicza optymalną macierz sprzężenia zwrotnego K tak, aby sterowanie
        u = -Kx
    minimalizowało funkcję kosztu:

        J = ∫ (x'Qx + u'Ru) dt

    przy równaniu dynamiki:
        dx/dt = A x + B u

    Parametry:
    -----------
    a : ndarray
        Macierz stanu (n x n)
    b : ndarray
        Macierz wejść (n x m)
    q : ndarray
        Macierz wag stanu (n x n)
    r : ndarray
        Macierz wag sterowania (m x m)
    nn : ndarray, opcjonalnie
        Macierz członów krzyżowych (n x m)

    Zwraca:
    --------
    k : ndarray
        Optymalna macierz sprzężenia zwrotnego
    s : ndarray
        Rozwiązanie równania Riccatiego
    """

    # Sprawdzenie zgodności wymiarów
    m, n = a.shape
    mb, nb = b.shape
    mq, nq = q.shape

    if m != mq or n != nq:
        raise ValueError('A i Q muszą mieć te same wymiary')

    mr, nr = r.shape
    if mr != nr or nb != mr:
        raise ValueError('B i R muszą być ze sobą zgodne wymiarowo')

    # Obsługa macierzy krzyżowej N
    if nn is not None:
        mn, nnn = nn.shape
        if mn != m or nnn != nr:
            raise ValueError('N musi być zgodne z Q oraz R')
        # Modyfikacja Q i A o składnik krzyżowy
        q = q - nn @ np.linalg.inv(r) @ nn.T
        a = a - b @ np.linalg.inv(r) @ nn.T
    else:
        nn = np.zeros((m, nb))

    # Sprawdzenie Q: symetryczna i dodatnio półokreślona
    if np.any(np.linalg.eigvals(q) < 0) or np.linalg.norm(q.T - q, 1) / np.linalg.norm(q, 1) > np.finfo(float).eps:
        raise ValueError('Q musi być macierzą symetryczną i dodatnio półokreśloną')

    # Sprawdzenie R: symetryczna i dodatnio określona
    if np.any(np.linalg.eigvals(r) <= 0) or np.linalg.norm(r.T - r, 1) / np.linalg.norm(r, 1) > np.finfo(float).eps:
        raise ValueError('R musi być macierzą symetryczną i dodatnio określoną')

    # Konstrukcja macierzy Hamiltona
    hamiltonian = np.block([
        [a, b @ np.linalg.inv(r) @ b.T],
        [q, -a.T]
    ])

    eigvals, eigvecs = np.linalg.eig(hamiltonian)

    # Sortowanie według części rzeczywistej
    idx = np.argsort(np.real(eigvals))
    eigvals = eigvals[idx]
    eigvecs = eigvecs[:, idx]

    # Sprawdzenie poprawnego uporządkowania wartości własnych
    if not (np.real(eigvals[n - 1]) < 1e-15 and np.real(eigvals[n]) > -1e-15):
        print("Nie można uporządkować wartości własnych — układ (A,B) może być niekontrolowalny. Sprawdzam macierz sterowalności C = [B AB ... A^(n-1)B]")

        C = np.zeros((m, n * nb))
        c = b.copy()
        C[:, :nb] = c
        for i in range(n - 1):
            c = a @ c
            C[:, (i + 1) * nb:(i + 2) * nb] = c

        rank_C = np.linalg.matrix_rank(C)
        if rank_C < n:
            raise ValueError(f'Rząd(C) = {rank_C} < n = {n}, układ (A,B) NIE jest sterowalny.')
        else:
            raise ValueError('Rząd(C) = n (OK), ale nie udało się uporządkować wartości własnych — należy to zbadać.')

    # Wybór wektorów własnych odpowiadających wartościom z ujemną częścią rzeczywistą
    chi = eigvecs[:n, :n]
    lambda_mat = eigvecs[n:2 * n, :n]

    if np.linalg.matrix_rank(chi) < chi.shape[0]:
        raise np.linalg.LinAlgError("Macierz chi jest osobliwa — macierz B może mieć zbyt mały wpływ na system.")

    # Rozwiązanie równania Riccatiego
    s = -np.real(lambda_mat @ np.linalg.inv(chi))

    # Wyliczenie K
    k = np.linalg.inv(r) @ (nn.T + b.T @ s)

    # ========================================================================
    # WERYFIKACJA STABILNOŚCI UKŁADU ZAMKNIĘTEGO
    # ========================================================================

    A_cl = a - b @ k
    eigvals_cl = np.linalg.eigvals(A_cl)
    max_real_part = np.max(np.real(eigvals_cl))

    # Ostrzeżenia stabilności
    if max_real_part >= 0:
        warnings.warn(
            f"NIESTABILNY UKŁAD ZAMKNIĘTY! Maksymalna część rzeczywista: {max_real_part:.4f}\n"
            f"Regulator może prowadzić do niestabilności — należy sprawdzić macierze Q oraz R.",
            RuntimeWarning
        )
    elif max_real_part > -0.1:
        warnings.warn(
            f"SŁABA STABILNOŚĆ! Maksymalna część rzeczywista: {max_real_part:.4f}\n"
            f"Układ jest stabilny, ale z bardzo wolnym zanikiem — rozważ korektę wag Q/R.",
            RuntimeWarning
        )

    # Obliczenie współczynników tłumienia
    damping_ratios = []
    natural_frequencies = []

    for ev in eigvals_cl:
        if np.imag(ev) != 0:  # Para sprzężona
            omega_n = abs(ev)
            zeta = -np.real(ev) / omega_n
            damping_ratios.append(zeta)
            natural_frequencies.append(omega_n)

    if damping_ratios:
        min_damping = min(damping_ratios)
        if min_damping < 0.1:
            warnings.warn(
                f"NISKIE TŁUMIENIE! Minimalny współczynnik tłumienia: {min_damping:.3f}\n"
                f"Układ może być oscylacyjny — rozważ zwiększenie wag Q.",
                RuntimeWarning
            )

    return k, s
