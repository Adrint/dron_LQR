# Quick Reference - Quadrocopter Simple LQR

## 🚀 Quick Start
```bash
cd D:\Praca_Magisterska_PW\src_new
python quadrocopter_simple_lqr.py
```

## 📊 State Vector (n=12)
```
x = [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
     ↑velocity  ↑rates  ↑position  ↑angles
```

## 🎮 Control Vector (m=4)
```
u = [T1, T2, T3, T4]  # Motor thrusts [N]
```

## ⚙️ Key Parameters

### Physical Constants (constants.py)
```python
MASS = 25.0 kg
g = 9.81 m/s²
MOTOR_ARM_LENGTH = 1.0 m
T_max = 300.0 N per motor
T_min = 0.0 N per motor
```

### LQR Gains (quadrocopter_simple_lqr.py)
```python
Q = diag([10, 10, 10,    # vx, vy, vz
          1, 1, 1,        # p, q, r
          5, 5, 100,      # X, Y, Z  ← Z is most important!
          5, 5, 2])       # phi, theta, psi

R = diag([1, 1, 1, 1])    # Motor thrust penalties
```

## 🔧 Quick Tuning

### More stable (less aggressive)
```python
R = np.eye(4) * 2.0  # Increase from 1.0
```

### Better altitude tracking
```python
Q[8, 8] = 150.0  # Increase from 100.0
```

### Better position tracking
```python
Q[6, 6] = 10.0  # X: Increase from 5.0
Q[7, 7] = 10.0  # Y: Increase from 5.0
```

### Smaller attitude angles
```python
Q[9, 9] = 10.0   # phi: Increase from 5.0
Q[10, 10] = 10.0  # theta: Increase from 5.0
```

## 🎯 Motor Configuration (X-config)

```
    1 (FR)      2 (FL)
      \    ↑    /
       \   │   /
        \  │  /
    ────⊕──┼──⊕────
        /  │  \
       /   │   \
      /    ↓    \
    4 (BR)      3 (BL)
```

## 📈 Expected Performance
- **Altitude error**: < 0.5m
- **Position error**: < 1.0m
- **Max angles**: < 10°
- **Hover thrust**: ~61N per motor

## ⚠️ Common Issues

### Diverges immediately?
→ Check `T_hover = MASS * g / 4.0`

### High oscillations?
→ Increase `R` (control penalty)

### Doesn't track?
→ Increase `Q[6,6]`, `Q[7,7]`, `Q[8,8]`

### Unstable angles?
→ Increase `Q[9,9]`, `Q[10,10]`

## 📁 Files

### ⭐ Main (Use This!)
- `quadrocopter_simple_lqr.py`

### 🔨 Core Modules
- `dynamics_simple.py`
- `integrator_simple.py`
- `linearization_simple.py`

### 📖 Documentation
- `IMPLEMENTATION_SUMMARY.md`
- `README_SIMPLE_LQR.md`
- `COMPARISON.md`

## 🎓 Control Law

```python
# 1. Calculate error
e = x - x_ref

# 2. LQR feedback
u_pert = -K @ e

# 3. Add baseline
u = T_baseline + u_pert

# 4. Clip to limits
u = np.clip(u, T_min, T_max)
```

## 🔄 Update Frequency

```python
# Gain recomputation: Every 10 steps
# Diagnostics print: Every 100 steps  
# Visualization: Every 50 steps
```

## 📊 Diagnostics Output

```
t=  1.000s
Position: X=  1.00 Y=  0.00 Z=  3.00 (ref: Z=  3.00)
Velocity: vx=  1.000 vy=  0.000 vz=  0.000
Attitude: phi=  0.50° theta=  2.30° psi=  0.00°
Thrusts: T1= 61.2N T2= 61.5N T3= 61.0N T4= 61.3N (sum=245.0N)
Errors: ΔZ= 0.000m Δvz= 0.000m/s
```

## 🧮 Hover Equilibrium

```python
T_hover = MASS * g / 4.0
        = 25.0 * 9.81 / 4.0
        = 61.3125 N per motor
Total   = 245.25 N (equals weight!)
```

## 🌐 Moments (X-configuration)

```python
arm = L / √2 = 0.707 m

M_roll  = (T3 + T4 - T1 - T2) × arm
M_pitch = (T2 + T4 - T1 - T3) × arm
M_yaw   = (T1 + T3 - T2 - T4) × k_yaw
```

## 💡 Tips

1. **Start simple** - Use default gains first
2. **Check diagnostics** - Print every 100 steps
3. **Visualize** - Watch plots update
4. **Tune conservatively** - Small changes at a time
5. **Compare with bicopter** - Learn from working code

## 📚 Learn More

- Full guide: `README_SIMPLE_LQR.md`
- Comparison: `COMPARISON.md`
- Summary: `IMPLEMENTATION_SUMMARY.md`
- Bicopter: `../bicopter/dron_lqr.py`

---
**🎯 This is based on working bicopter code - it should work out of the box!**
