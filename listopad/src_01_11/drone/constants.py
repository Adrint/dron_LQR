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

g = 9.81 # Gravitational acceleration [m/s^2] - acts in +Z direction (down)
RO_0 = 1.225 # Air density at sea level [kg/m^3]

MASS = 8.0 # Drone mass [kg]
MASS_engine = MASS * 0.5 # Drone single engine mass [kg]
NOMINAL_thrust = MASS * g / 4.0
S = 0.4  # m² - powierzchnia odniesienia

# Motor arm length (distance from center to motor) [m]
MOTOR_ARM_LENGTH_X = 0.6  # Distance in X direction (front/rear)
MOTOR_ARM_LENGTH_Y = 0.6  # Distance in Y direction (left/right)
CD_0 = 0.30 # Drag coefficient

# ============================================================================
# MOMENT OF INERTIA - CORRECTED!
# ============================================================================
# IX (roll inertia) = m * L_y^2 (simplified, assuming point masses)
# IY (pitch inertia) = m * L_x^2 (simplified, assuming point masses)
# IZ (yaw inertia) ≈ m * (L_x^2 + L_y^2) (rotation about vertical axis)

IX = 4 * MASS_engine * MOTOR_ARM_LENGTH_Y ** 2  # Roll inertia (about X-axis)
IY = 4 * MASS_engine * MOTOR_ARM_LENGTH_X ** 2  # Pitch inertia (about Y-axis)
IZ = 4 * MASS_engine * (MOTOR_ARM_LENGTH_X ** 2 + MOTOR_ARM_LENGTH_Y ** 2)  # Yaw inertia (about Z-axis)

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

# ============================================================================
# AERODYNAMIC DAMPING
# ============================================================================

C_Lp = -0.01  # Roll damping (moment due to roll rate p)
CM_Q = -0.01  # Pitch damping (moment due to pitch rate q)
C_Nr = -0.01  # Yaw damping (moment due to yaw rate r)

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
h_flight = 1.0 # Flight altitude above terrain [m]
Vel = 2 # Reference velocity [m/s]

# ============================================================================
# PHYSICAL LIMITS
# ============================================================================
T_max = 130.0 # Maximum thrust per motor [N]
T_min = 0.0 # Minimum thrust (motors cannot reverse) [N]

# ============================================================================
# ANGULAR VELOCITY LIMITS
# ============================================================================

# Maximum angular velocities [rad/s]
MAX_PITCH_RATE = np.deg2rad(300)  # 300°/s - forward/backward tilt rate
MAX_ROLL_RATE = np.deg2rad(300)   # 300°/s - left/right tilt rate
MAX_YAW_RATE = np.deg2rad(100)    # 100°/s - rotation rate

# ============================================================================
# ATTITUDE LIMITS
# ============================================================================

# Maximum tilt angles [rad]
MAX_TILT_ANGLE_P = np.deg2rad(25) # 25° - maximum tilt (P mode with vision system)

# Use P mode limit by default (safer)
MAX_ROLL_ANGLE = MAX_TILT_ANGLE_P
MAX_PITCH_ANGLE = MAX_TILT_ANGLE_P

# Yaw can rotate freely (360°)
MAX_YAW_ANGLE = np.deg2rad(180)   # ±180° for normalization

# ============================================================================
# LINEAR VELOCITY LIMITS
# ============================================================================

# Maximum horizontal velocity [m/s]
MAX_HORIZONTAL_VELOCITY = 23.0  # 23 m/s (~82.8 km/h)
MAX_ASCENT_RATE = 6.0    # 6 m/s upward (vz = -6 in NED)
MAX_DESCENT_RATE = 5.0   # 5 m/s downward (vz = +5 in NED) - vertical
MAX_DESCENT_RATE_TILTED = 7.0  # 7 m/s downward when tilted

# ============================================================================
# COMBINED VELOCITY LIMITS
# ============================================================================

# Maximum total velocity (magnitude)
MAX_TOTAL_VELOCITY = MAX_HORIZONTAL_VELOCITY

# ============================================================================
# ANGULAR VELOCITY LIMITS
# ============================================================================

# Maximum angular velocities [rad/s]
MAX_PITCH_RATE = np.deg2rad(300)  # 300°/s - forward/backward tilt rate
MAX_ROLL_RATE = np.deg2rad(300)   # 300°/s - left/right tilt rate
MAX_YAW_RATE = np.deg2rad(100)    # 100°/s - rotation rate

# ============================================================================
# ATTITUDE LIMITS
# ============================================================================

# Maximum tilt angles [rad]
MAX_TILT_ANGLE_P = np.deg2rad(25) # 25° - maximum tilt (P mode with vision system)

# Use P mode limit by default (safer)
MAX_ROLL_ANGLE = MAX_TILT_ANGLE_P
MAX_PITCH_ANGLE = MAX_TILT_ANGLE_P

# Yaw can rotate freely (360°)
MAX_YAW_ANGLE = np.deg2rad(180)   # ±180° for normalization



# Maximum horizontal velocity [m/s]
MAX_HORIZONTAL_VELOCITY = 23.0  # 23 m/s (~82.8 km/h)
MAX_ASCENT_RATE = 6.0    # 6 m/s upward (vz = -6 in NED)
MAX_DESCENT_RATE = 5.0   # 5 m/s downward (vz = +5 in NED) - vertical
MAX_DESCENT_RATE_TILTED = 7.0  # 7 m/s downward when tilted

MAX_TOTAL_VELOCITY = MAX_HORIZONTAL_VELOCITY

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