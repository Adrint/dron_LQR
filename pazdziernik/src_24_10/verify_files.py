"""
Verification script - run this to check if all files are correct
"""
import numpy as np

print("="*70)
print("CHECKING DRONE SIMULATION FILES")
print("="*70)

# Test 1: Import constants
try:
    from constants import MASS, g, IX, IY, CM_P, CM_Q, MOTOR_ARM_LENGTH_X, MOTOR_ARM_LENGTH_Y
    print("✓ constants.py - OK")
    print(f"  MASS={MASS}kg, IX={IX:.2f}, IY={IY:.2f}")
except Exception as e:
    print(f"✗ constants.py - ERROR: {e}")

# Test 2: Import trajectory
try:
    from trajectory import generate_reference_profile
    X, Y, Z_terr, Z_ref, alpha, beta = generate_reference_profile(2.0, 0.01, 50)
    print("✓ trajectory.py - OK")
    print(f"  Generated {len(X)} points, Y range: {Y.min():.2f} to {Y.max():.2f}m")
except Exception as e:
    print(f"✗ trajectory.py - ERROR: {e}")

# Test 3: Import rhs
try:
    from rhs import aa_rhs
    
    # Test hover equilibrium (n=14 state vector)
    x_hover = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0,
                       MASS*g/4, MASS*g/4, MASS*g/4, MASS*g/4])
    u_hover = np.array([MASS*g/4, MASS*g/4, MASS*g/4, MASS*g/4])
    
    dx = aa_rhs(x_hover, 0.0, u_hover)
    
    # Check if derivatives are near zero
    if abs(dx[0]) < 1e-6 and abs(dx[1]) < 1e-6 and abs(dx[2]) < 1e-6:
        print("✓ rhs.py - OK (hover equilibrium achieved)")
        print(f"  dvx/dt={dx[0]:.2e}, dvy/dt={dx[1]:.2e}, dvz/dt={dx[2]:.2e}")
    else:
        print("✗ rhs.py - Hover equilibrium failed")
        print(f"  dvx/dt={dx[0]:.2e}, dvy/dt={dx[1]:.2e}, dvz/dt={dx[2]:.2e}")
except Exception as e:
    print(f"✗ rhs.py - ERROR: {e}")

# Test 4: Import rk45
try:
    from rk45 import aa_rk45
    print("✓ rk45.py - OK")
except Exception as e:
    print(f"✗ rk45.py - ERROR: {e}")

# Test 5: Import matrices
try:
    from matrices import aa_matrices_AB
    print("✓ matrices.py - OK")
except Exception as e:
    print(f"✗ matrices.py - ERROR: {e}")

# Test 6: Import lqr
try:
    from lqr import lqr_m
    print("✓ lqr.py - OK")
except Exception as e:
    print(f"✗ lqr.py - ERROR: {e}")

print("\n" + "="*70)
print("VERIFICATION COMPLETE")
print("="*70)
print("\nIf all tests passed, run: python dron_lqr_with_orientation.py")
