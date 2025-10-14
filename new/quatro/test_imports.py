"""
Quick test script to verify imports work correctly
"""

import sys
import os

print("=" * 80)
print("TESTING IMPORTS")
print("=" * 80)

try:
    print("\n1. Testing common modules...")
    from common.constants import MASS, g, T_max
    print(f"   ✓ constants.py: MASS={MASS}kg, g={g}m/s², T_max={T_max}N")
    
    from common.integrator import rk45_simple
    print(f"   ✓ integrator.py: rk45_simple imported")
    
    from common.trajectory import generate_reference_profile, generate_3d_trajectory
    print(f"   ✓ trajectory.py: functions imported")
    
    from common.linearization import linearize_system_simple
    print(f"   ✓ linearization.py: linearize_system_simple imported")
    
    from common.lqr_controller import design_lqr
    print(f"   ✓ lqr_controller.py: design_lqr imported")
    
    print("\n2. Testing trajectory generation...")
    import numpy as np
    X_ref, Z_terr, Z_ref, alpha = generate_reference_profile(2.0, 0.01, 10)
    print(f"   ✓ 2D trajectory: {len(X_ref)} points generated")
    
    t, X, Y, Z, Z_t, Vx, Vy, Vz = generate_3d_trajectory(2.0, 0.01, 5.0)
    print(f"   ✓ 3D trajectory: {len(t)} points generated")
    
    print("\n3. Testing dynamics...")
    from common.dynamics_simple import quadrocopter_dynamics_simple
    x = np.zeros(12)
    x[8] = 2.0  # Z = 2m
    u = np.array([MASS*g/4, MASS*g/4, MASS*g/4, MASS*g/4])
    dx = quadrocopter_dynamics_simple(x, 0.0, u)
    print(f"   ✓ Dynamics: dx computed, shape={dx.shape}")
    
    print("\n" + "=" * 80)
    print("✅ ALL TESTS PASSED!")
    print("=" * 80)
    print("\nYou can now run simulations:")
    print("  cd simulations_3D")
    print("  python dron_PID.py")
    print("\nOr use the launcher:")
    print("  python run_simulation.py")
    print()
    
except Exception as e:
    print(f"\n❌ ERROR: {e}")
    print("\nPlease check:")
    print("  1. Are you in the src_new/ directory?")
    print("  2. Are all required packages installed?")
    print("     pip install numpy scipy matplotlib")
    import traceback
    traceback.print_exc()
