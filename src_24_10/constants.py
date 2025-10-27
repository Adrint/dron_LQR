import numpy as np

# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

# COORDINATE SYSTEM: NED (North-East-Down)
# - X: Forward (North)
# - Y: Right (East)
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
# - Motor 1 (Front-Left):  (+X, +Y)
# - Motor 2 (Front-Right): (+X, -Y)
# - Motor 3 (Rear-Left):   (-X, +Y)
# - Motor 4 (Rear-Right):  (-X, -Y)
# 
# Pitch control: Front pair (1,2) vs Rear pair (3,4)
# Roll control: Left pair (1,3) vs Right pair (2,4)

# Drone mass [kg]
MASS = 15.0

# Reference wing area [m^2]
S = 1.0

# Drag coefficient
CD_0 = 0.30

# Motor arm length (distance from center to motor) [m]
MOTOR_ARM_LENGTH = 0.5  # Keep for backward compatibility
MOTOR_ARM_LENGTH_X = 0.5  # Distance in X direction (front/rear)
MOTOR_ARM_LENGTH_Y = 0.5  # Distance in Y direction (left/right)

# ============================================================================
# MOMENT OF INERTIA
# ============================================================================
#
# For a quadcopter with motors at distance L from center:
# IY (pitch inertia, rotation around Y axis) = m * L_x^2
# IX (roll inertia, rotation around X axis) = m * L_y^2
# where L_x = MOTOR_ARM_LENGTH_X (distance in X direction)
#       L_y = MOTOR_ARM_LENGTH_Y (distance in Y direction)
#
# With MASS = 15 kg and L = 0.5 m:
# IY = 15 * (0.5)^2 = 3.75 kg·m^2
# IX = 15 * (0.5)^2 = 3.75 kg·m^2

IY = MASS * MOTOR_ARM_LENGTH_X ** 2  # Pitch inertia
IX = MASS * MOTOR_ARM_LENGTH_Y ** 2  # Roll inertia

# ============================================================================
# AERODYNAMIC DAMPING
# ============================================================================

# Moment damping coefficient (moment due to angular velocity)
# This represents aerodynamic damping of rotation
# Negative value provides damping (energy dissipation)
CM_Q = -0.01  # Pitch damping
CM_P = -0.01  # Roll damping (same value)

# ============================================================================
# MOTOR/ACTUATOR DYNAMICS
# ============================================================================

# Time constant for motor response [s]
# Models first-order lag: dT/dt = (1/TAU) * (T_cmd - T)
# TAU = 0.05 means motor responds quickly (5% settling time ~ 60ms)
TAU = 0.05

# Thrust scaling factor (use 1.0 for no scaling)
THRUST_SCALING = 1.0

# ============================================================================
# ENVIRONMENTAL CONDITIONS
# ============================================================================

# Turbulence
az_turbulence = 0.0          # vertical turbulence acceleration [m/s^2]
ax_wind = 0.0                # horizontal wind acceleration [m/s^2]
az_wind = 0.0                # vertical wind acceleration [m/s^2]
ay_wind = 0.0                # lateral wind acceleration [m/s^2]

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
    print(f"Moment of inertia Iy (pitch): {IY:.4f} kg·m^2")
    print(f"Moment of inertia Ix (roll): {IX:.4f} kg·m^2")
    print(f"Weight: {MASS * g:.2f} N")
    print(f"Thrust per motor for hover: {MASS * g / 4:.2f} N")
    print(f"Max total thrust: {T_max * 4:.2f} N")
    print(f"Thrust-to-weight ratio: {T_max * 4 / (MASS * g):.2f}")
    print(f"Motor time constant: {TAU} s")
    print(f"Reference velocity: {Vel} m/s")
