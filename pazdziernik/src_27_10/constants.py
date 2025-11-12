import numpy as np

# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

# COORDINATE SYSTEM: NED (North-East-Down)
# - X: Forward (North)
# - Y: Right (East) - not used in 2D simulation
# - Z: Down (positive downward)
# - Ground level: Z = 0
# - Flying altitude: Z < 0 (negative values)
# - Gravity acts in +Z direction (downward)

# Unit conversions
DEG2RAD = np.pi / 180.0      # degrees to radians
RAD2DEG = 180.0 / np.pi      # radians to degrees

# Gravitational acceleration [m/s^2] - acts in +Z direction (down)
g = 9.81

# Air density at sea level [kg/m^3]
RO_0 = 1.225

# ============================================================================
# DRONE PARAMETERS - QUADCOPTER X CONFIGURATION
# ============================================================================
# Configuration: 4 motors in X pattern
#
# Top view (looking down, Z-axis pointing down in NED):
#
#        M2 (CCW)              M1 (CW)
#             \               /
#              \             /
#               \___  +Y   /
#                   \ |  /
#             +X <--- o
#                   / | \
#               ___/     \
#              /           \
#             /             \
#        M3 (CCW)              M4 (CW)
#
# Motor positions (body frame):
# - Motor 1 (Front-Right, CW):  (+X, +Y diagonal) at angle +45°
# - Motor 2 (Front-Left, CCW):  (+X, -Y diagonal) at angle -45°
# - Motor 3 (Rear-Left, CCW):   (-X, -Y diagonal) at angle -135°
# - Motor 4 (Rear-Right, CW):   (-X, +Y diagonal) at angle +135°
#
# Control allocation:
# - Altitude: T1 + T2 + T3 + T4 (all motors)
# - Pitch:    (T1 + T2) - (T3 + T4) (front vs rear)
# - Roll:     (T1 + T4) - (T2 + T3) (right vs left)
# - Yaw:      (T1 + T3) - (T2 + T4) (CW vs CCW propellers)

# Drone mass [kg]
MASS = 15.0

# Reference wing area [m^2]
S = 1.0

# Drag coefficient
CD_0 = 0.30

# Motor arm length (distance from center to motor) [m]
MOTOR_ARM_LENGTH = 0.5  # Keep for backward compatibility
MOTOR_ARM_LENGTH_X = 0.5  # Distance in X direction (front/rear)
MOTOR_ARM_LENGTH_Y = 0.5  # Distance in Y direction (left/right) - not used in 2D

# ============================================================================
# MOMENT OF INERTIA - CORRECTED!
# ============================================================================
#
# For a quadcopter with motors at distance L from center:
# IX (roll inertia) = m * L_y^2 (simplified, assuming point masses)
# IY (pitch inertia) = m * L_x^2 (simplified, assuming point masses)
# IZ (yaw inertia) ≈ m * (L_x^2 + L_y^2) (rotation about vertical axis)
#
# where:
# L_x = MOTOR_ARM_LENGTH_X (distance in X direction)
# L_y = MOTOR_ARM_LENGTH_Y (distance in Y direction)
#
# With MASS = 15 kg and L = 0.5 m:
# IX = 15 * (0.5)^2 = 3.75 kg·m^2
# IY = 15 * (0.5)^2 = 3.75 kg·m^2
# IZ = 15 * (0.5^2 + 0.5^2) = 7.5 kg·m^2

IX = MASS * MOTOR_ARM_LENGTH_Y ** 2  # Roll inertia (about X-axis)
IY = MASS * MOTOR_ARM_LENGTH_X ** 2  # Pitch inertia (about Y-axis)
IZ = MASS * (MOTOR_ARM_LENGTH_X ** 2 + MOTOR_ARM_LENGTH_Y ** 2)  # Yaw inertia (about Z-axis)

# ============================================================================
# AERODYNAMIC DAMPING
# ============================================================================

# Moment damping coefficients (moment due to angular velocities)
# These represent aerodynamic damping of rotation
# Negative values provide damping (energy dissipation)

C_Lp = -0.01  # Roll damping (moment due to roll rate p)
CM_Q = -0.01  # Pitch damping (moment due to pitch rate q)
C_Nr = -0.01  # Yaw damping (moment due to yaw rate r)

# ============================================================================
# MOTOR/ACTUATOR DYNAMICS
# ============================================================================

# Time constant for motor response [s]
# Models first-order lag: dT/dt = (1/TAU) * (T_cmd - T)
# TAU = 0.05 means motor responds quickly (5% settling time ~ 60ms)
TAU = 0.05

# Propeller torque coefficient [N·m/N]
# Ratio of propeller reaction torque to thrust
# k_torque = Q/T where Q is torque, T is thrust
# Typical values: 0.01-0.02 for small quads
# Used for yaw control: opposing motor pairs create yaw moment
k_torque = 0.015

# Thrust scaling factor (use 1.0 for no scaling)
THRUST_SCALING = 1.0

# ============================================================================
# ENVIRONMENTAL CONDITIONS
# ============================================================================

# Turbulence
az_turbulence = 0.0          # vertical turbulence acceleration [m/s^2]
ax_wind = 0.0                # horizontal wind acceleration [m/s^2]
az_wind = 0.0                # vertical wind acceleration [m/s^2]

# Turbulence zone parameters (for simulation)
c_turb = 1000.0              # turbulence strength
X_turb_1 = 1500.0            # turbulence zone start [m]
X_turb_2 = 2000.0            # turbulence zone end [m]

# ============================================================================
# INITIAL CONDITIONS & TRAJECTORY
# ============================================================================

# Initial altitude above ground [m] - NEGATIVE in NED (Z down)
# z0 = -2.0 means flying 2 meters above ground
z0 = -2.0

# Flight altitude above terrain [m]
h_flight = 1.0

# Reference velocity [m/s]
Vel = 2

# ============================================================================
# PHYSICAL LIMITS
# ============================================================================

# Maximum thrust per motor [N]
T_max = 400.0

# Minimum thrust (motors cannot reverse) [N]
T_min = 0.0

# ============================================================================
# SANITY CHECKS - Print calculated values
# ============================================================================

if __name__ == "__main__":
    print("=== DRONE PARAMETERS ===")
    print(f"Mass: {MASS} kg")
    print(f"Motor arm length X: {MOTOR_ARM_LENGTH_X} m")
    print(f"Motor arm length Y: {MOTOR_ARM_LENGTH_Y} m")
    print(f"\nMoments of inertia:")
    print(f"  Ix (roll):  {IX:.4f} kg·m^2")
    print(f"  Iy (pitch): {IY:.4f} kg·m^2")
    print(f"  Iz (yaw):   {IZ:.4f} kg·m^2")
    print(f"\nDamping coefficients:")
    print(f"  C_Lp (roll):  {C_Lp}")
    print(f"  CM_Q (pitch): {CM_Q}")
    print(f"  C_Nr (yaw):   {C_Nr}")
    print(f"\nPropeller torque coefficient: {k_torque} N·m/N")
    print(f"\nWeight: {MASS * g:.2f} N")
    print(f"Thrust per motor for hover: {MASS * g / 4:.2f} N")
    print(f"Max total thrust: {T_max * 4:.2f} N")
    print(f"Thrust-to-weight ratio: {T_max * 4 / (MASS * g):.2f}")
    print(f"Motor time constant: {TAU} s")
    print(f"Reference velocity: {Vel} m/s")