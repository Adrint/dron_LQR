import numpy as np

# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

# Unit conversions
DEG2RAD = np.pi / 180.0      # degrees to radians
RAD2DEG = 180.0 / np.pi      # radians to degrees

# Gravitational acceleration [m/s^2]
g = 9.81

# Air density at sea level [kg/m^3]
RO_0 = 1.225

# ============================================================================
# DRONE PARAMETERS
# ============================================================================

# Drone mass [kg]
MASS = 5.0

# Reference wing area [m^2]
S = 1.0

# Drag coefficient
CD_0 = 0.30

# Motor arm length (distance from center to motor) [m]
MOTOR_ARM_LENGTH = 0.5

# ============================================================================
# MOMENT OF INERTIA - CORRECTED!
# ============================================================================
#
# For a symmetric drone:
# IY ≈ m * (L/2)^2
# where L = distance between motors
#
# Original value: IY = 100.0  (100x TOO LARGE!)
# Corrected:

IY = MASS * (MOTOR_ARM_LENGTH / 2.0) ** 2
# IY = 25 * (0.25)^2 = 25 * 0.0625 = 1.5625 kg·m^2

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

# Initial altitude above ground [m]
z0 = 2.0

# Flight altitude above terrain [m]
h_flight = 1.0

# Reference velocity [m/s]
Vel = 10

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
    print(f"Thrust per motor for hover: {MASS * g / 2:.2f} N")
    print(f"Max total thrust: {T_max * 2:.2f} N")
    print(f"Thrust-to-weight ratio: {T_max * 2 / (MASS * g):.2f}")
    print(f"Motor time constant: {TAU} s")
    print(f"Reference velocity: {Vel} m/s")