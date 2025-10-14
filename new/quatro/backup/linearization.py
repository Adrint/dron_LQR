import numpy as np
from dynamics import quadrocopter_dynamics_simple


def linearize_system_simple(x, t, u, n, m):
    """
    Calculate linearized system matrices A and B using finite differences
    
    Parameters:
    -----------
    x : array_like
        Current state vector
    t : float
        Current time
    u : array_like
        Control input vector
    n : int
        Dimension of state vector
    m : int
        Dimension of control vector

    Returns:
    --------
    A : ndarray
        State matrix (n x n)
    B : ndarray
        Input matrix (n x m)
    """
    del_val = 1.0e-6

    f0 = quadrocopter_dynamics_simple(x, t, u)

    A = np.zeros((n, n))
    for j in range(n):
        dx = np.zeros(n)
        dx[j] = del_val
        A[:, j] = (quadrocopter_dynamics_simple(x + dx, t, u) - f0) / del_val

    B = np.zeros((n, m))
    for j in range(m):
        du = np.zeros(m)
        du[j] = del_val
        B[:, j] = (quadrocopter_dynamics_simple(x, t, u + du) - f0) / del_val

    return A, B
