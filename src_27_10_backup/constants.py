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
# - Motor 1 (Front-Left):  (+X, +Y) in 3D, but we simulate in 2D (X-Z plane)
# - Motor 2 (Front-Right): (+X, -Y)
# - Motor 3 (Rear-Left):   (-X, +Y)
# - Motor 4 (Rear-Right):  (-X, -Y)
# For 2D pitch control: Front pair (1,2) vs Rear pair (3,4)

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
# IY (pitch inertia) = m * L_x^2 (simplified, assuming point masses)
# where L_x = MOTOR_ARM_LENGTH_X (distance in X direction)
#
# With MASS = 15 kg and L = 0.5 m:
# IY = 15 * (0.5)^2 = 3.75 kg·m^2

IY = MASS * MOTOR_ARM_LENGTH_X ** 2

# ============================================================================
# AERODYNAMIC DAMPING
# ============================================================================

# Moment damping coefficient (moment due to angular velocity)
# This represents aerodynamic damping of rotation
# Negative value provides damping (energy dissipation)
CM_Q = -0.01

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
    print(f"Motor arm length: {MOTOR_ARM_LENGTH} m")
    print(f"Moment of inertia Iy: {IY:.4f} kg·m^2")
    print(f"Weight: {MASS * g:.2f} N")
    print(f"Thrust per motor for hover: {MASS * g / 4:.2f} N")
    print(f"Max total thrust: {T_max * 4:.2f} N")
    print(f"Thrust-to-weight ratio: {T_max * 4 / (MASS * g):.2f}")
    print(f"Motor time constant: {TAU} s")
    print(f"Reference velocity: {Vel} m/s")