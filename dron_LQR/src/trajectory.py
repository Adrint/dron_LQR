import numpy as np

def aa_trajectory(X, Vel, dt):
    """
    Generuje punkt Z trajektorii i kąt nachylenia alpha na podstawie pozycji X, prędkości Vel i kroku dt.

    Parametry:
        X : float - bieżąca pozycja w poziomie
        Vel : float - prędkość w poziomie
        dt : float - krok czasowy

    Zwraca:
        Z : float - wysokość trajektorii nad terenem
        alpha : float - kąt nachylenia trajektorii (rad)
    """

    def Z_at(x):
        # Funkcja pomocnicza: definicja wysokości terenu w zależności od pozycji X
        if x <= 1.0:
            return 1.0
        elif 1.0 < x < 1.5:
            return 1.0 + (x - 1.0) * 10.0  # stromy wzrost
        elif 1.5 <= x <= 2.0:
            return 6.0  # płaskowyż
        elif 2.0 < x <= 2.5:
            return 6.0 - (x - 2.0) * 10.0  # strome zejście
        else:
            return 1.0  # płaskie zakończenie

    

    Z1 = Z_at(X)  # aktualna wysokość terenu
    dx = Vel * dt  # mała zmiana w X
    Z2 = Z_at(X + dx)  # przyszła wysokość terenu

    # Obliczenie kąta nachylenia jako arctan przyrostu wysokości
    alpha = np.arctan2(Z2 - Z1, dx)

    return Z1, alpha
