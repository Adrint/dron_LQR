import numpy as np
from scipy.linalg import solve_continuous_are

def design_lqr(A, B, Q, R):
    """
    Design LQR controller
    Solves: A'P + PA - PBR^-1B'P + Q = 0
    Returns: K = R^-1 B' P
    """
    try:
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        return K, P
    except np.linalg.LinAlgError:
        print("  ⚠️  LQR design failed")
        return np.zeros((B.shape[1], A.shape[0])), np.zeros((A.shape[0], A.shape[0]))