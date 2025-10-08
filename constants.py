import numpy as np

# Jednostki
DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi

# Stałe fizyczne i parametry modelu
G = 9.81             # przyspieszenie grawitacyjne [m/s^2]
MASS = 25.0          # masa drona [kg]
S = 1.0              # powierzchnia nośna [m^2]
CD_0 = 0.30          # bazowy współczynnik oporu
IY = 100.0           # moment bezwładności [kg*m^2]
TAU = 0.05           # stała czasowa silnika
THRUST_SCALING = 1.0 # współczynnik przeskalowania ciągu
CM_Q = -0.01         # współczynnik tłumienia
RO_0 = 1.225         # gęstość powietrza na poziomie morza [kg/m^3]
