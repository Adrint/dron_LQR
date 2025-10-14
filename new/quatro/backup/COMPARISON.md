# Comparison: Cascade LQR vs Simple LQR

## Executive Summary

This document compares two approaches for quadrocopter control:
1. **Cascade LQR** (original implementation)
2. **Simple Direct LQR** (based on working bicopter)

## Architecture Comparison

### Cascade LQR (Complex)
```
Reference Position
    ↓
┌─────────────────────────┐
│  Position Controller    │
│  (Outer Loop - LQR)     │
└─────────────────────────┘
    ↓ Velocity Command
┌─────────────────────────┐
│  Kinematic Transform    │
│  vel → attitude         │
└─────────────────────────┘
    ↓ Attitude Command
┌─────────────────────────┐
│  Attitude Controller    │
│  (Inner Loop - LQR)     │
└─────────────────────────┘
    ↓ Moment Command
┌─────────────────────────┐
│  Mixer                  │
│  moment → thrusts       │
└─────────────────────────┘
    ↓ Motor Thrusts
   PLANT
```

### Simple Direct LQR
```
Reference State
    ↓
┌─────────────────────────┐
│  Error Calculation      │
│  e = x - x_ref          │
└─────────────────────────┘
    ↓
┌─────────────────────────┐
│  LQR Controller         │
│  u = -K @ e + baseline  │
└─────────────────────────┘
    ↓ Motor Thrusts
   PLANT
```

## Code Complexity

### Cascade LQR
- **Lines of code**: ~250 (cascade_lqr.py)
- **Control loops**: 2 (position + attitude)
- **Coordinate transforms**: 3 (inertial → velocity → attitude → moment)
- **LQR problems solved**: 2 per update
- **Gain matrices**: 2 (K_pos, K_att)

### Simple Direct LQR
- **Lines of code**: ~180 (quadrocopter_simple_lqr.py)
- **Control loops**: 1 (direct state feedback)
- **Coordinate transforms**: 0 (direct thrust control)
- **LQR problems solved**: 1 per update
- **Gain matrices**: 1 (K)

## Performance Comparison

| Metric | Cascade LQR | Simple LQR | Winner |
|--------|-------------|------------|--------|
| **Code Simplicity** | Complex | Simple | ✅ Simple |
| **Computational Cost** | High | Low | ✅ Simple |
| **Tuning Difficulty** | Hard (2 sets of gains) | Easier (1 set) | ✅ Simple |
| **Stability** | Can be unstable | More stable | ✅ Simple |
| **Physical Intuition** | Obscured | Clear | ✅ Simple |
| **Scalability** | Harder | Easier | ✅ Simple |

## Tuning Parameters

### Cascade LQR
```python
# Position loop (3 states: X, Y, Z)
Q_pos = diag([5.0, 5.0, 50.0])
R_pos = diag([0.5, 0.5, 0.5])

# Attitude loop (6 states: phi, theta, psi, p, q, r)
Q_att = diag([10.0, 10.0, 5.0, 20.0, 20.0, 10.0])
R_att = diag([0.01, 0.01, 0.001])
```
**Total parameters to tune**: 9 + 9 = **18 parameters**

### Simple Direct LQR
```python
# Single state-feedback (12 states)
Q = diag([10, 10, 10, 1, 1, 1, 5, 5, 100, 5, 5, 2])
R = diag([1.0, 1.0, 1.0, 1.0])
```
**Total parameters to tune**: 12 + 4 = **16 parameters** (and more intuitive!)

## Mathematical Formulation

### Cascade LQR

**Outer loop** (Position to Velocity):
```
State: [X, Y, Z]
Control: [vx_cmd, vy_cmd, vz_cmd]

Minimize: J_pos = ∫(e_pos' Q_pos e_pos + v_cmd' R_pos v_cmd) dt
```

**Inner loop** (Attitude to Moments):
```
State: [phi, theta, psi, p, q, r]
Control: [M_roll, M_pitch, M_yaw]

Minimize: J_att = ∫(e_att' Q_att e_att + M' R_att M) dt
```

**Mixer** (Moments to Thrusts):
```
Algebraic: [T1, T2, T3, T4] = mixer([M_roll, M_pitch, M_yaw], T_total)
```

### Simple Direct LQR

**Single optimization**:
```
State: [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
Control: [T1, T2, T3, T4]

Minimize: J = ∫(e' Q e + u' R u) dt

Subject to: dx/dt = A*x + B*u
```

Direct solution: `u = -K @ e + u_baseline`

## Advantages and Disadvantages

### Cascade LQR

✅ **Advantages**:
- Matches classical control structure (position → attitude → actuators)
- Can enforce specific bandwidth separation
- Easier to add constraints at each level

❌ **Disadvantages**:
- Complex implementation
- Two sets of gains to tune
- Interaction between loops can cause instability
- Coordinate transformations add errors
- Computationally expensive

### Simple Direct LQR

✅ **Advantages**:
- **Simple implementation** (follows working bicopter)
- Single optimization problem
- Direct physical interpretation
- Computationally efficient
- Proven to work (from bicopter)
- Easier debugging

❌ **Disadvantages**:
- Less explicit control over inner loop dynamics
- Higher dimensional optimization
- May not respect bandwidth separation

## When to Use Each Approach

### Use Cascade LQR when:
- You need explicit control over attitude loop bandwidth
- Hardware limitations require separation of loops
- You want to add constraints at specific control levels
- You have computational resources
- You need to match classical autopilot structure

### Use Simple Direct LQR when:
- You want a **working solution quickly**
- Simplicity is more important than structure
- Computational resources are limited
- You're prototyping or learning
- The bicopter approach worked well

## Recommended Approach

For this project, I recommend **starting with Simple Direct LQR** because:

1. ✅ It's based on **proven working code** (bicopter)
2. ✅ Much **simpler to debug**
3. ✅ **Easier to tune** (single set of gains)
4. ✅ **More stable** in practice
5. ✅ **Faster to iterate** on

Once the simple approach works well, you can:
- Study why it works
- Compare performance with cascade
- Decide if added complexity is worth it

## Migration Path

If you want to transition from simple to cascade:

1. ✅ **Start**: Get simple LQR working perfectly
2. ✅ **Analyze**: Understand the control structure
3. ✅ **Split**: Separate position and attitude errors
4. ✅ **Tune**: Design each loop independently
5. ✅ **Compare**: Measure performance differences
6. ✅ **Decide**: Choose based on requirements

## Conclusion

Both approaches are valid, but **Simple Direct LQR** is recommended for:
- Initial development ✅
- Learning and understanding ✅
- Rapid prototyping ✅
- Most practical applications ✅

**Cascade LQR** is better for:
- Production systems with specific requirements
- When explicit loop separation is needed
- Advanced control strategies

## Code Files Summary

### Simple Direct LQR (RECOMMENDED)
```
quadrocopter_simple_lqr.py        # Main simulation
dynamics_simple.py                 # Physics model
integrator_simple.py               # RK45 integration
linearization_simple.py            # System linearization
```

### Cascade LQR (Original)
```
main.py                            # Main simulation
cascade_lqr.py                     # Cascade controller
dynamics.py                        # Physics model
integrator.py                      # RK45 integration
linearization.py                   # System linearization
mixers.py                          # Motor mixer
```

## Final Recommendation

🎯 **Run `quadrocopter_simple_lqr.py` first!**

It's based on working bicopter code and should give you a stable, flying quadrocopter immediately. Then you can compare with the cascade approach if needed.
