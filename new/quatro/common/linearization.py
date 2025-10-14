"""
System linearization for LQR controller design
"""

import numpy as np
from .dynamics_simple import quadrocopter_dynamics_simple


def linearize_system_simple(x, t, u, n, m):
    """
    Calculate linearized system matrices A and B using finite differences
    
    Parameters:
    -----------
    x : array_like
        Current state vector [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
    t : float
        Current time
    u : array_like
        Control input vector [T1, T2, T3, T4]
    n : int
        Dimension of state vector (12)
    m : int
        Dimension of control vector (4)

    Returns:
    --------
    A : ndarray
        State matrix (n x n) - how states affect state derivatives
    B : ndarray
        Input matrix (n x m) - how controls affect state derivatives
    """
    del_val = 1.0e-6

    # Nominal dynamics
    f0 = quadrocopter_dynamics_simple(x, t, u)

    # State matrix A (partial derivatives w.r.t. states)
    A = np.zeros((n, n))
    for j in range(n):
        dx = np.zeros(n)
        dx[j] = del_val
        A[:, j] = (quadrocopter_dynamics_simple(x + dx, t, u) - f0) / del_val

    # Input matrix B (partial derivatives w.r.t. controls)
    B = np.zeros((n, m))
    for j in range(m):
        du = np.zeros(m)
        du[j] = del_val
        B[:, j] = (quadrocopter_dynamics_simple(x, t, u + du) - f0) / del_val

    return A, B
