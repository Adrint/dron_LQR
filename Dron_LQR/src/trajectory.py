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
        if x <= 1.0:
            return 1.0
        elif 1.0 < x < 1.5:
            return 1.0 + (x - 1.0) * 10.0  # stromy wzrost
        elif 1.5 <= x <= 2.0:
            return 6.0  # płaskowyż
        elif 2.0 < x <= 2.5:
            return 6.0 - (x - 2.0) * 10.0  # strome zejście
        elif 2.5 < x <= 3.0:
            return 1.0
        # ▶️ Od tego miejsca teren się zmienia ciekawiej:

        # elif 2.5 < x <= 4.0:
        #     return 2.0 + 10.0 * np.exp(-((x - 3.0) ** 2) / 0.1)  # wieżowiec jak dzwon Gaussa
        #
        # elif 4.0 < x <= 5.5:
        #     return -((x - 4.75) ** 2) + 7.0  # parabola (szczyt w x=4.75, wysokość 7)
        #
        # elif 5.5 < x <= 6.5:
        #     return 2.0 + 1.5 * np.sin((x - 5.5) * np.pi)  # dół: sinusoidalna dolina
        #
        # elif 6.5 < x <= 8.0:
        #     return 3.0 + 0.5 * np.sin(3 * (x - 6.5))  # łagodna fala

        else:
            return 1.0  # zakończenie na wysokości 3

    Z1 = Z_at(X)  # aktualna wysokość terenu
    dx = Vel * dt  # mała zmiana w X
    Z2 = Z_at(X + dx)  # przyszła wysokość terenu

    # Obliczenie kąta nachylenia jako arctan przyrostu wysokości
    alpha = np.arctan2(Z2 - Z1, dx)

    return Z1, alpha
