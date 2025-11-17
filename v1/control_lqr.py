import numpy as np
from scipy import linalg
import warnings


def lqr_m(a, b, q, r, nn=None):
    """
    Linear Quadratic Regulator design for continuous-time systems.

    Calculates the optimal feedback gain matrix K such that the feedback
    law u = -Kx minimizes the cost function:

        J = Integral {x'Qx + u'Ru} dt

    subject to the constraint equation:
        dx/dt = Ax + Bu

    Parameters:
    -----------
    a : ndarray
        State matrix (n x n)
    b : ndarray
        Input matrix (n x m)
    q : ndarray
        State cost matrix (n x n)
    r : ndarray
        Input cost matrix (m x m)
    nn : ndarray, optional
        Cross-term matrix (n x m)

    Returns:
    --------
    k : ndarray
        Optimal feedback gain matrix
    s : ndarray
        Solution to the algebraic Riccati equation
    """
    m, n = a.shape
    mb, nb = b.shape
    mq, nq = q.shape

    if m != mq or n != nq:
        raise ValueError('A and Q must be the same size')

    mr, nr = r.shape
    if mr != nr or nb != mr:
        raise ValueError('B and R must be consistent')

    if nn is not None:
        mn, nnn = nn.shape
        if mn != m or nnn != nr:
            raise ValueError('N must be consistent with Q and R')
        # Add cross term
        q = q - nn @ np.linalg.inv(r) @ nn.T
        a = a - b @ np.linalg.inv(r) @ nn.T
    else:
        nn = np.zeros((m, nb))

    # Check if q is positive semi-definite and symmetric
    if np.any(np.linalg.eigvals(q) < 0) or np.linalg.norm(q.T - q, 1) / np.linalg.norm(q, 1) > np.finfo(float).eps:
        raise ValueError('Q must be symmetric and positive semi-definite')

    # Check if r is positive definite and symmetric
    if np.any(np.linalg.eigvals(r) <= 0) or np.linalg.norm(r.T - r, 1) / np.linalg.norm(r, 1) > np.finfo(float).eps:
        raise ValueError('R must be symmetric and positive definite')

    # Start eigenvector decomposition by finding eigenvectors of Hamiltonian
    hamiltonian = np.block([
        [a, b @ np.linalg.inv(r) @ b.T],
        [q, -a.T]
    ])

    eigvals, eigvecs = np.linalg.eig(hamiltonian)

    # Sort eigenvalues by real part
    idx = np.argsort(np.real(eigvals))
    eigvals = eigvals[idx]
    eigvecs = eigvecs[:, idx]

    # Check eigenvalue ordering
    if not (np.real(eigvals[n - 1]) < 1e-15 and np.real(eigvals[n]) > -1e-15):
        print("Can't order eigenvalues, (A,B) may be uncontrollable. Checking rank C = [B A*B ... A^(n-1)*B]")
        C = np.zeros((m, n * nb))
        c = b.copy()
        C[:, :nb] = c
        for i in range(n - 1):
            c = a @ c
            C[:, (i + 1) * nb:(i + 2) * nb] = c

        rank_C = np.linalg.matrix_rank(C)
        if rank_C < n:
            raise ValueError(f'rank_C = {rank_C} < n = {n}, (A,B) are uncontrollable.')
        else:
            raise ValueError('rank_C = n (OK), but there is something wrong with ordering - check it out!')

    # Select vectors with negative eigenvalues
    chi = eigvecs[:n, :n]
    lambda_mat = eigvecs[n:2 * n, :n]

    if np.linalg.matrix_rank(chi) < chi.shape[0]:
        raise np.linalg.LinAlgError("Macierz chi jest osobliwa – B ma za mało wpływu na system.")

    s = -np.real(lambda_mat @ np.linalg.inv(chi))
    k = np.linalg.inv(r) @ (nn.T + b.T @ s)

    # ========================================================================
    # STABILITY VERIFICATION OF CLOSED-LOOP SYSTEM
    # ========================================================================

    A_cl = a - b @ k
    eigvals_cl = np.linalg.eigvals(A_cl)

    # Check if all eigenvalues have negative real parts (stable)
    max_real_part = np.max(np.real(eigvals_cl))

    if max_real_part >= 0:
        warnings.warn(
            f"UNSTABLE CLOSED-LOOP! Maximum eigenvalue real part: {max_real_part:.4f}\n"
            f"System may be unstable or marginally stable. Check Q and R matrices.",
            RuntimeWarning
        )
    elif max_real_part > -0.1:
        warnings.warn(
            f"MARGINALLY STABLE! Maximum eigenvalue real part: {max_real_part:.4f}\n"
            f"System is stable but has slow convergence. Consider adjusting Q/R weights.",
            RuntimeWarning
        )

    # Calculate stability margins
    damping_ratios = []
    natural_frequencies = []

    for ev in eigvals_cl:
        if np.imag(ev) != 0:  # Complex eigenvalue pair
            omega_n = abs(ev)  # Natural frequency
            zeta = -np.real(ev) / omega_n  # Damping ratio
            damping_ratios.append(zeta)
            natural_frequencies.append(omega_n)

    if damping_ratios:
        min_damping = min(damping_ratios)
        if min_damping < 0.1:
            warnings.warn(
                f"LOW DAMPING! Minimum damping ratio: {min_damping:.3f}\n"
                f"System may exhibit oscillatory behavior. Consider increasing Q weights.",
                RuntimeWarning
            )

    return k, s