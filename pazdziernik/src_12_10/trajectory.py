import numpy as np


def generate_reference_profile(Vel, dt, X_max=50):
    """Generate reference trajectory including terrain and flight altitude"""
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    for X in X_ref:
        if X <= 10.0:
            Z = 1.0
        elif X < 20.0:
            Z = 1.0 + (X - 10.0) * (5.0 / 10.0)
        elif X <= 30.0:
            Z = 6.0
        elif X <= 40.0:
            Z = 6.0 - (X - 30.0) * (5.0 / 10.0)
        else:
            Z = 1.0
        Z_terr.append(Z)

    Z_terr = np.array(Z_terr)
    alpha = np.gradient(Z_terr, dx)
    alpha = np.arctan(alpha)

    Z_ref = Z_terr + 2.0  # 2m above terrain

    return X_ref, Z_terr, Z_ref, alpha