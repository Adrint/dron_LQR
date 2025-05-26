import numpy as np

def aa_mdl(X, Y, teta, c):
    """
    Generuje punkty modelu prostokątnego (np. pojazdu/drona) obróconego o kąt teta.

    Parametry:
        X : float - współrzędna X środka modelu
        Y : float - współrzędna Y środka modelu
        teta : float - kąt obrotu (w radianach)
        c : float - współczynnik skalowania modelu (rozmiar)

    Zwraca:
        xs, ys : listy float - współrzędne wierzchołków prostokąta po obrocie
    """

    # Współrzędne bazowe prostokąta przed obrotem (lokalne)
    xs0 = np.array([-0.1 * c, 0.1 * c, 0.1 * c, -0.1 * c, -0.1 * c])
    ys0 = np.array([-0.1 * c, -0.1 * c, 0.1 * c, 0.1 * c, -0.1 * c])

    # Obrót i przesunięcie prostokąta do globalnego układu współrzędnych
    xs = X + xs0 * np.cos(teta) - ys0 * np.sin(teta)
    ys = Y + xs0 * np.sin(teta) + ys0 * np.cos(teta)

    return xs.tolist(), ys.tolist()
