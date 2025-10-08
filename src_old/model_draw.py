import numpy as np


def aa_mdl(X, Y, teta, c):
    """
    Calculate drone model coordinates for visualization

    Parameters:
    -----------
    X : float
        X position of drone
    Y : float
        Y position of drone
    teta : float
        Pitch angle (radians)
    c : float
        Scale factor

    Returns:
    --------
    xs : ndarray
        X coordinates of drone outline
    ys : ndarray
        Y coordinates of drone outline
    """
    xs0 = np.array([-0.1 * c, +0.1 * c, +0.1 * c, -0.1 * c, -0.1 * c])
    ys0 = np.array([-0.1 * c, -0.1 * c, +0.1 * c, +0.1 * c, -0.1 * c])

    xs = np.zeros(5)
    ys = np.zeros(5)

    for i in range(5):
        xs[i] = X + xs0[i] * np.cos(teta) - ys0[i] * np.sin(teta)
        ys[i] = Y + xs0[i] * np.sin(teta) + ys0[i] * np.cos(teta)

    return xs, ys