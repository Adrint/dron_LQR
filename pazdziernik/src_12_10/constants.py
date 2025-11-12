import numpy as np

# Physical constants
DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi
g = 9.81              # [m/s^2]
RO_0 = 1.225          # [kg/m^3]

# Quadrocopter parameters
MASS = 25.0           # [kg]
S = 1.0               # Reference area [m^2]
CD_0 = 0.30           # Drag coefficient

# Motor arm length
MOTOR_ARM_LENGTH = 1.0
d_arm = MOTOR_ARM_LENGTH / np.sqrt(2)

# Moments of inertia
I_motor = 0.1
I_frame = 2.0
IX = I_frame + 4 * I_motor * (d_arm ** 2)
IY = I_frame + 4 * I_motor * (d_arm ** 2)
IZ = I_frame + 4 * I_motor * (2 * d_arm ** 2)

# Aerodynamic damping
CM_Q = -0.01

# Motor dynamics
TAU = 0.05            # [s]

# Environment
az_turbulence = 0.0
ax_wind = 0.0
az_wind = 0.0
c_turb = 1000.0
X_turb_1 = 1500.0
X_turb_2 = 2000.0

# Initial conditions
z0 = 2.0              # Initial altitude [m]
h_flight = 1.0        # Flight altitude above terrain [m]
Vel = 1.0             # Reference velocity [m/s]

# Physical limits
T_max = 300.0         # Max thrust per motor [N]
T_min = 0.0           # Min thrust [N]

# Configuration
YAW_SCALE = 0.1

print(f"DEBUG: IX = {IX:.4f}, IY = {IY:.4f}, IZ = {IZ:.4f} kg·m^2")