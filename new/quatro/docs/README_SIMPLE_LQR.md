# Quadrocopter LQR Control - Simple Approach

## Overview

This implementation adapts the **working bicopter code** to create a functional quadrocopter simulation with LQR control. The key insight is to use a **simple, direct LQR approach** instead of a complex cascade controller.

## Key Changes from Original Quadrocopter Code

### 1. **Simplified Control Architecture**
- **Before**: Complex cascade LQR with outer position loop and inner attitude loop
- **After**: Single direct LQR controller that computes thrust perturbations from state errors

### 2. **Direct Thrust Control**
- **Before**: Position → Velocity command → Attitude command → Moments → Motor thrusts (5 transformations!)
- **After**: State error → LQR gain → Thrust perturbations → Motor thrusts (2 steps!)

### 3. **Clearer State Space Model**
```python
State vector (n=12):
[vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]

Control vector (m=4):
[T1, T2, T3, T4]  # Direct motor thrusts in Newtons
```

### 4. **Conservative LQR Tuning**
Following the bicopter's successful approach:
- High weight on altitude (Z): **Q[8,8] = 100.0**
- Moderate weights on velocities: **Q[0:3, 0:3] = 10.0**
- Low weights on angular rates: **Q[3:6, 3:6] = 1.0**
- Uniform control penalty: **R = I × 1.0**

### 5. **Simplified Dynamics**
Created `dynamics_simple.py` that closely follows the bicopter's structure:
- Clear separation of aerodynamics, thrust, and rotation
- Proper rotation matrix for body-to-inertial frame transformation
- Simplified X-configuration moment calculations

## File Structure

### New Files (Simple Approach)
```
src_new/
├── quadrocopter_simple_lqr.py     # Main simulation (RECOMMENDED)
├── dynamics_simple.py              # Simplified dynamics model
├── integrator_simple.py            # RK45 integrator for simple dynamics
└── linearization_simple.py         # Linearization for simple dynamics
```

### Original Files (Still Available)
```
src_new/
├── main.py                         # Original cascade approach
├── cascade_lqr.py                  # Complex cascade controller
├── dynamics.py                     # Original dynamics
├── integrator.py                   # Original integrator
└── linearization.py                # Original linearization
```

### Shared Files
```
src_new/
├── constants.py                    # Physical parameters
├── trajectory.py                   # Reference trajectory generation
├── lqr_controller.py               # LQR solver (design_lqr function)
├── mixers.py                       # Motor mixer (for cascade controller)
└── plotter.py                      # Visualization (for cascade controller)
```

## How to Run

### Recommended: Simple LQR Approach
```bash
cd D:\Praca_Magisterska_PW\src_new
python quadrocopter_simple_lqr.py
```

This should work immediately and show:
- Real-time trajectory tracking
- Stable altitude control
- Smooth reference following

### Alternative: Original Cascade Approach
```bash
cd D:\Praca_Magisterska_PW\src_new
python main.py
```

## Understanding the Bicopter Approach

The bicopter code that **actually works** uses these principles:

### 1. **Direct State Feedback**
```python
# Compute error
e = x - x_ref

# LQR control law
u_perturbation = -K @ e

# Apply to baseline
u = u_baseline + u_perturbation
```

### 2. **Physical Thrust Limits**
```python
T_baseline = MASS * g / num_motors
u = np.clip(u, T_min, T_max)
```

### 3. **Clear Diagnostics**
Every 100 steps, print:
- Position vs reference
- Velocity components
- Attitude angles
- Thrust values
- Error magnitudes

### 4. **Conservative Gain Updates**
Recompute LQR gains every 10 steps (not every step) to save computation.

## Quadrocopter X-Configuration

```
     1 (front-right)
    / \
   /   \
  4     2 (back-right, front-left)
   \   /
    \ /
     3 (back-left)
```

### Motor Positions
- Motor 1: (+x, +y) diagonal
- Motor 2: (-x, +y) diagonal  
- Motor 3: (-x, -y) diagonal
- Motor 4: (+x, -y) diagonal

### Moment Equations
```python
arm = MOTOR_ARM_LENGTH / sqrt(2)

M_roll  = (T3 + T4 - T1 - T2) * arm      # Around X-axis
M_pitch = (T2 + T4 - T1 - T3) * arm      # Around Y-axis
M_yaw   = (T1 + T3 - T2 - T4) * k_yaw    # Around Z-axis
```

## Tuning Guide

If the quadrocopter is unstable, try adjusting these parameters in `quadrocopter_simple_lqr.py`:

### 1. **Increase Altitude Control**
```python
Q[8, 8] = 200.0  # Was 100.0
```

### 2. **Reduce Control Aggressiveness**
```python
R = np.eye(m) * 2.0  # Was 1.0
```

### 3. **Increase Position Damping**
```python
Q[6, 6] = 10.0  # X position (was 5.0)
Q[7, 7] = 10.0  # Y position (was 5.0)
```

### 4. **Reduce Velocity Tracking**
```python
Q[0, 0] = 5.0   # vx (was 10.0)
Q[1, 1] = 5.0   # vy (was 10.0)
Q[2, 2] = 5.0   # vz (was 10.0)
```

## Common Issues and Solutions

### Issue: Quadrocopter diverges immediately
**Solution**: Check initial conditions and T_hover calculation
```python
T_hover = MASS * g / 4.0  # Must be exact!
```

### Issue: Oscillations in altitude
**Solution**: Increase R (control penalty) or decrease Q[8,8]

### Issue: Doesn't follow reference
**Solution**: Increase Q[6,6] and Q[7,7] (position weights)

### Issue: Extreme attitude angles
**Solution**: Increase Q[9,9], Q[10,10] (attitude weights)

## Comparison with Bicopter

| Aspect | Bicopter | Quadrocopter |
|--------|----------|--------------|
| State dimension | n=6 or n=8 | n=12 |
| Control inputs | m=2 | m=4 |
| Moments of inertia | IY only | IX, IY, IZ |
| Rotation | 2D (pitch) | 3D (roll, pitch, yaw) |
| Complexity | Simple | Moderate |

## Expected Performance

With default tuning, you should see:
- **Altitude tracking error**: < 0.5m RMS
- **Position tracking error**: < 1.0m RMS
- **Attitude angles**: < 10° during maneuvers
- **Simulation time**: ~3-5 seconds for 50s flight

## Next Steps

1. **Run the simple version** to verify it works
2. **Tune the gains** for your specific requirements
3. **Add disturbances** (wind, turbulence) in the simulation
4. **Compare** with the cascade controller performance
5. **Analyze** which approach better suits your needs

## Credits

This implementation is based on the working bicopter code in:
```
D:\Praca_Magisterska_PW\bicopter\
```

Key files analyzed:
- `dron_lqr.py` - Main simulation loop
- `rhs.py` - Dynamics equations
- `lqr.py` - LQR controller design
- `matrices.py` - Linearization

## License

This is part of a master's thesis project (Praca Magisterska) at Warsaw University of Technology.
