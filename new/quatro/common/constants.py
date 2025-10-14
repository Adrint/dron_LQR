"""
Physical constants and quadrocopter parameters
"""

import numpy as np

# Physical constants
DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi
g = 9.81              # [m/s^2]
RO_0 = 1.225          # [kg/m^3]

# Quadrocopter parameters
MASS = 2.0            # [kg] - typical small quadrocopter
S = 0.1               # Reference area [m^2]
CD_0 = 0.30           # Drag coefficient

# Motor arm length
MOTOR_ARM_LENGTH = 0.25  # [m] - distance from center to motor
d_arm = MOTOR_ARM_LENGTH / np.sqrt(2)

# Moments of inertia (for 2kg quadrocopter)
I_motor = 0.001       # Small motor inertia
I_frame = 0.01        # Frame inertia
IX = I_frame + 4 * I_motor * (d_arm ** 2)
IY = I_frame + 4 * I_motor * (d_arm ** 2)
IZ = I_frame + 4 * I_motor * (2 * d_arm ** 2)

# Aerodynamic damping
CM_Q = -0.01

# Motor dynamics
TAU = 0.05            # [s] Time constant

# Environment (for disturbances - not used in basic simulation)
az_turbulence = 0.0
ax_wind = 0.0
az_wind = 0.0
c_turb = 1000.0
X_turb_1 = 1500.0
X_turb_2 = 2000.0

# Initial conditions
z0 = 2.0              # Initial altitude [m]
h_flight = 1.0        # Flight altitude above terrain [m]
Vel = 2.0             # Reference velocity [m/s]

# Physical limits
T_max = 20.0          # Max thrust per motor [N] (2kg * 9.81 / 4 * 4 = ~20N)
T_min = 0.0           # Min thrust [N]

# Configuration
YAW_SCALE = 0.1
