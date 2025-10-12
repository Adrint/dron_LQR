import numpy as np

# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi

g = 9.81              # [m/s^2]
RO_0 = 1.225          # [kg/m^3]

# ============================================================================
# QUADROCOPTER PARAMETERS
# ============================================================================

# Drone mass [kg]
MASS = 25.0

# Reference wing area [m^2]
S = 1.0

# Drag coefficient
CD_0 = 0.30

# ============================================================================
# MOTOR ARM LENGTH
# ============================================================================

# Distance from center to each motor [m]
# For X-configuration (45 degrees):
# Actual moment arm = MOTOR_ARM_LENGTH / sqrt(2)
MOTOR_ARM_LENGTH = 1.0  # Total diagonal distance (0.707 m per axis)

# ============================================================================
# MOMENTS OF INERTIA
# ============================================================================

# For a symmetric quadrocopter with motors at distance 'd' from center:
# Ix ≈ Iy ≈ I_motor * 4 + I_frame
# Iz ≈ I_motor * 4 * 2 (motors have more effect on yaw)

# Simplified calculation:
d_arm = MOTOR_ARM_LENGTH / np.sqrt(2)

# Assume each motor+prop has rotational inertia ~ 0.1 kg*m^2
I_motor = 0.1

# Frame inertia (estimate for square frame)
I_frame = 2.0

# Combined moments of inertia
IX = I_frame + 4 * I_motor * (d_arm ** 2)  # Roll
IY = I_frame + 4 * I_motor * (d_arm ** 2)  # Pitch
IZ = I_frame + 4 * I_motor * (2 * d_arm ** 2)  # Yaw (motors contribute more)

print(f"DEBUG: IX = {IX:.4f}, IY = {IY:.4f}, IZ = {IZ:.4f} kg·m^2")

# ============================================================================
# AERODYNAMIC DAMPING
# ============================================================================

CM_Q = -0.01  # Moment damping coefficient

# ============================================================================
# MOTOR/ACTUATOR DYNAMICS
# ============================================================================

TAU = 0.05  # Motor response time constant [s]

# ============================================================================
# ENVIRONMENTAL CONDITIONS
# ============================================================================

az_turbulence = 0.0
ax_wind = 0.0
az_wind = 0.0

c_turb = 1000.0
X_turb_1 = 1500.0
X_turb_2 = 2000.0

# ============================================================================
# INITIAL CONDITIONS & TRAJECTORY
# ============================================================================

z0 = 2.0              # Initial altitude [m]
h_flight = 1.0        # Flight altitude above terrain [m]
Vel = 1.0             # Reference velocity [m/s]

# ============================================================================
# PHYSICAL LIMITS
# ============================================================================

T_max = 300.0  # Max thrust per motor [N]
T_min = 0.0    # Min thrust [N]

# ============================================================================
# QUADROCOPTER CONFIGURATION
# ============================================================================

# X-Configuration (45 degrees):
# Motor 1 (front-left):   position = (-d/sqrt(2), -d/sqrt(2))
# Motor 2 (front-right):  position = (-d/sqrt(2), +d/sqrt(2))
# Motor 3 (rear-left):    position = (+d/sqrt(2), -d/sqrt(2))
# Motor 4 (rear-right):   position = (+d/sqrt(2), +d/sqrt(2))

# Control mixing (how thrusts combine to create moments):
# Roll (phi):   M_roll = (T3 + T4 - T1 - T2) * arm
# Pitch (theta): M_pitch = (T2 + T4 - T1 - T3) * arm
# Yaw (psi):    M_yaw = (T1 + T3 - T2 - T4) * k (k is scaling factor)

YAW_SCALE = 0.1  # Scaling factor for yaw moment

# ============================================================================
# SANITY CHECKS
# ============================================================================

if __name__ == "__main__":
    print("=== QUADROCOPTER PARAMETERS ===")
    print(f"Mass: {MASS} kg")
    print(f"Motor arm length: {MOTOR_ARM_LENGTH} m")
    print(f"Effective moment arm per axis: {d_arm:.4f} m")
    print(f"IX (Roll):  {IX:.4f} kg·m^2")
    print(f"IY (Pitch): {IY:.4f} kg·m^2")
    print(f"IZ (Yaw):   {IZ:.4f} kg·m^2")
    print(f"Weight: {MASS * g:.2f} N")
    print(f"Thrust per motor for hover: {MASS * g / 4:.2f} N")
    print(f"Max total thrust: {T_max * 4:.2f} N")
    print(f"Thrust-to-weight ratio: {T_max * 4 / (MASS * g):.2f}")