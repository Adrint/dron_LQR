"""
Generator trajektorii - UKŁAD NED (Z w dół)
"""

import numpy as np

def generate_reference_profile(Vel, dt, X_max=50):
    """
    Generuje profil referencyjny w układzie NED
    
    Returns:
        X_ref: pozycje X
        Z_terr: wysokości terenu (ujemne w NED!)
        Z_ref: wysokości lotu (ujemne w NED!)
        alpha: kąty nachylenia terenu
    """
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)
    
    h_terrain = []
    
    for X in X_ref:
        if X <= 10.0:
            h = 1.0
        elif X < 20.0:
            h = 1.0 + (X - 10.0) * (5.0 / 10.0)
        elif X <= 30.0:
            h = 6.0
        elif X <= 40.0:
            h = 6.0 - (X - 30.0) * (5.0 / 10.0)
        else:
            h = 1.0
        
        h_terrain.append(h)
    
    h_terrain = np.array(h_terrain)
    Z_terr = -h_terrain  # NED: ujemne Z = wysoko
    
    h_flight = 2.0
    h_ref = h_terrain + h_flight
    Z_ref = -h_ref
    
    alpha = np.gradient(h_terrain, dx)
    alpha = np.arctan(alpha)
    
    return X_ref, Z_terr, Z_ref, alpha


def aa_trajectory(X, Vel, dt):
    """
    Oblicza wysokość terenu i kąt w pozycji X (NED)
    """
    if X <= 10.0:
        h1 = 1.0
    elif X < 20.0:
        h1 = 1.0 + (X - 10.0) * (5.0 / 10.0)
    elif X <= 30.0:
        h1 = 6.0
    elif X <= 40.0:
        h1 = 6.0 - (X - 30.0) * (5.0 / 10.0)
    else:
        h1 = 1.0
    
    dx = Vel * dt
    X_next = X + dx
    
    if X_next <= 10.0:
        h2 = 1.0
    elif X_next < 20.0:
        h2 = 1.0 + (X_next - 10.0) * (5.0 / 10.0)
    elif X_next <= 30.0:
        h2 = 6.0
    elif X_next <= 40.0:
        h2 = 6.0 - (X_next - 30.0) * (5.0 / 10.0)
    else:
        h2 = 1.0
    
    alpha = np.arctan2(h2 - h1, dx)
    Z_terr = -h1
    
    return Z_terr, alpha


if __name__ == "__main__":
    print("TEST TRAJEKTORII (NED)")
    print()
    
    X_ref, Z_terr, Z_ref, alpha = generate_reference_profile(1.0, 0.1, 50)
    
    print(f"Punktów: {len(X_ref)}")
    print()
    print(f"{'X':>8} | {'h_terr':>8} | {'Z_terr':>8} | {'h_ref':>8} | {'α':>8}")
    print("-" * 50)
    
    for X_test in [0, 10, 20, 30, 40]:
        idx = np.argmin(np.abs(X_ref - X_test))
        h_terr = -Z_terr[idx]
        h_ref = -Z_ref[idx]
        print(f"{X_test:8.1f} | {h_terr:8.2f} | {Z_terr[idx]:8.2f} | "
              f"{h_ref:8.2f} | {np.degrees(alpha[idx]):8.2f}")
