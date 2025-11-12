import numpy as np

# Jednostki
DEG2RAD = np.pi / 180.0     # konwersja stopni na radiany
RAD2DEG = 180.0 / np.pi     # konwersja radianów na stopnie

# Stałe fizyczne
g = 9.81                    # przyspieszenie ziemskie [m/s^2]
RO_0 = 1.225                # gęstość powietrza na poziomie morza [kg/m^3]

# Parametry modelu drona
MASS = 25.0                 # masa drona [kg]
S = 1.0                     # powierzchnia nośna [m^2]
CD_0 = 0.30                 # współczynnik oporu czołowego
IY = 100.0                  # moment bezwładności względem osi Y [kg·m^2]
TAU = 0.05                  # stała czasowa silników [s]
CM_Q = -0.01                # tłumienie dynamiczne (moment od prędkości kątowej)
THRUST_SCALING = 1.0        # współczynnik skalowania ciągu
MOTOR_ARM_LENGTH = 0.5      # odległość silnika od środka drona [m]

# Warunki środowiskowe
az_turbulence = 0.0         # chwilowe zaburzenie pionowe
ax_wind = 0.0               # wiatr w osi X
az_wind = 0.0               # wiatr w osi Z
c_turb = 1000.0             # siła turbulencji
X_turb_1 = 1500.0           # początek strefy turbulencji
X_turb_2 = 2000.0           # koniec strefy turbulencji

# Warunki początkowe i trajektoria
z0 = 2.0                    # początkowa wysokość [m]
h_flight = 1.0              # wysokość lotu nad terenem [m]
Vel = 1.0                   # prędkość przelotowa [m/s]
