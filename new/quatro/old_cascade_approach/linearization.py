import numpy as np
from src_new.old_cascade_approach.dynamics import quadrocopter_dynamics


def linearize_system(x, t, u, n, m):
    """
    Calculate linearized system matrices A and B using finite differences
    """
    del_val = 1.0e-6

    f0 = quadrocopter_dynamics(x, t, u)

    A = np.zeros((n, n))
    for j in range(n):
        dx = np.zeros(n)
        dx[j] = del_val
        A[:, j] = (quadrocopter_dynamics(x + dx, t, u) - f0) / del_val

    B = np.zeros((n, m))
    for j in range(m):
        du = np.zeros(m)
        du[j] = del_val
        B[:, j] = (quadrocopter_dynamics(x, t, u + du) - f0) / del_val

    return A, B