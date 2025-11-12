import numpy as np
from rhs import aa_rhs
from matrices import aa_matrices_AB
from constants import MASS, g, z0

"""
CONTROLLABILITY ANALYSIS FOR QUADROCOPTER

The issue: Not all states can be controlled!

Analysis of the system:
- 12 states: [vx, vy, vz, p, q, r, X, Y, Z, phi, theta, psi]
- 4 controls: [T1, T2, T3, T4]

Problems:
1. X and Y positions cannot be directly controlled (no lateral forces in hover)
2. The coupling between forces and moment arms limits controllability
3. Hover equilibrium loses controllability rank
"""


def analyze_controllability(n, name):
    print(f"\n{'=' * 80}")
    print(f"CONTROLLABILITY ANALYSIS: {name} (n={n})")
    print(f"{'=' * 80}")

    T_hover = MASS * g / 4.0
    m = 4

    if n == 12:
        x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
    else:  # n == 16
        x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0,
                      T_hover, T_hover, T_hover, T_hover])

    u = np.array([T_hover, T_hover, T_hover, T_hover])
    t = 0.0

    # Get linearized system
    A, B = aa_matrices_AB("rhs", x, t, u, n, m)

    print(f"\nA matrix shape: {A.shape}")
    print(f"B matrix shape: {B.shape}")

    # Build controllability matrix
    C_matrices = [B]
    for i in range(1, n):
        C_matrices.append(np.linalg.matrix_power(A, i) @ B)

    C = np.hstack(C_matrices)
    rank_C = np.linalg.matrix_rank(C)

    print(f"\nControllability matrix rank: {rank_C}/{n}")
    print(f"System is {'CONTROLLABLE' if rank_C == n else 'NOT FULLY CONTROLLABLE'}")

    # Analyze which states are affected
    print(f"\n--- CONTROL INFLUENCE ANALYSIS ---")
    state_names = ['vx', 'vy', 'vz', 'p', 'q', 'r', 'X', 'Y', 'Z', 'phi', 'theta', 'psi']
    if n == 16:
        state_names += ['T1', 'T2', 'T3', 'T4']

    # Check B matrix influence
    print(f"\nDirect B matrix influence (dx/du):")
    for i in range(n):
        influence = np.linalg.norm(B[i, :])
        print(f"  State {i:2d} ({state_names[i]:8s}): {influence:.6e}")

    # Identify uncontrollable modes (eigenvectors of A not in range of B)
    print(f"\n--- EIGENVALUE ANALYSIS ---")
    eigvals, eigvecs = np.linalg.eig(A)
    print(f"Eigenvalues of A:")
    for i, ev in enumerate(eigvals):
        print(f"  λ_{i}: {ev.real:12.6f} + {ev.imag:12.6f}j")

    # Check which states are uncontrollable
    print(f"\n--- UNCONTROLLABLE STATES ---")
    uncontrollable_indices = []
    for i in range(n):
        if np.linalg.norm(C[i, :]) < 1e-10:
            uncontrollable_indices.append(i)

    if uncontrollable_indices:
        print("Uncontrollable state indices:")
        for idx in uncontrollable_indices:
            print(f"  State {idx}: {state_names[idx]}")
    else:
        print("No completely uncontrollable states found.")

    # Rank analysis per state
    print(f"\n--- RANK CONTRIBUTION PER STATE ---")
    for i in range(n):
        # Remove row i and check rank drop
        C_without_i = np.delete(C, i, axis=0)
        rank_without = np.linalg.matrix_rank(C_without_i)
        is_critical = "CRITICAL" if rank_without < rank_C else ""
        print(f"  State {i:2d} ({state_names[i]:8s}): rank={rank_without:2d} {is_critical}")

    return rank_C == n


# Analyze both systems
n12_controllable = analyze_controllability(12, "Simple model")
n16_controllable = analyze_controllability(16, "Model with motor dynamics")

print(f"\n{'=' * 80}")
print("RECOMMENDATIONS")
print(f"{'=' * 80}")

if not n12_controllable:
    print("\n⚠️  n=12 model is NOT controllable")
    print("   Reason: Position states (X, Y) cannot be controlled at hover")
    print("   Solution: Use trajectory tracking with error on controllable states only")

if not n16_controllable:
    print("\n⚠️  n=16 model is NOT controllable")
    print("   Reason: Same as n=12, plus possible motor dynamics issues")
    print("   Solution: Use cascade control or reduced feedback")

print("\n" + "=" * 80)
print("SOLUTION: Use LQR on CONTROLLABLE STATES ONLY")
print("=" * 80)
print("\nOption 1: Control only attitude and velocity (6 states)")
print("  Use: [vx, vy, vz, p, q, r] as feedback")
print("  Position follows from velocity integration")

print("\nOption 2: Use cascade control")
print("  Inner loop: Motor control (fast)")
print("  Outer loop: Attitude control (slow)")

print("\nOption 3: Pre-compute gain matrix K offline")
print("  Use pseudoinverse instead of standard LQR")
print("  K = (R^-1 B^T Q) pinv(A)")