import numpy as np
import math

def generate_reference_profile(Vel, dt, X_max=50):
    """
    Generate 3D reference trajectory profile including terrain and flight altitude.
    
    NED Coordinate System:
    - X: Forward (North)
    - Y: Right (East)
    - Z: Down (positive downward)
    - Ground at Z = 0
    - Terrain/flight altitudes: Z < 0 (negative values)

    Parameters:
    -----------
    Vel : float
        Drone velocity (m/s)
    dt : float
        Time step (s)
    X_max : float
        Maximum X range for generating trajectory

    Returns:
    --------
    X_ref : ndarray
        Horizontal reference positions (forward)
    Y_terr : ndarray
        Terrain lateral positions at each X (NED: Y positive = right)
    Z_terr : ndarray
        Terrain heights at each X (NED: negative values = above ground)
    Y_ref : ndarray
        Reference lateral positions for flight
    Z_ref : ndarray
        Reference flight altitudes above terrain (NED: negative values)
    alpha : ndarray
        Pitch angles - slope angles in X-Z plane (radians)
    beta : ndarray
        Roll angles - slope angles in X-Y plane (radians)
    """
    dx = Vel * dt
    X_ref = np.arange(0, X_max + dx, dx)

    Z_terr = []
    Y_terr = []

    # Generate terrain profile with both vertical (Z) and lateral (Y) variations
    for X in X_ref:
        if X <= 10.0:
            # Flat section at low altitude, centered
            Z = -1.0  # 1m above ground (NED: negative = altitude)
            Y = 0.0   # Centered (no lateral deviation)
            
        elif X < 20.0:
            # Climbing and moving right
            progress = (X - 10.0) / 10.0  # 0 to 1
            Z = -1.0 - progress * 5.0     # -1m → -6m (climbing)
            Y = progress * 5.0            # 0m → 5m (moving right)
            
        elif X <= 30.0:
            # High altitude, offset to the right
            Z = -6.0  # 6m above ground
            Y = 5.0   # 5m to the right
            
        elif X < 40.0:
            # Descending and moving back to center
            progress = (X - 30.0) / 10.0  # 0 to 1
            Z = -6.0 + progress * 5.0     # -6m → -1m (descending)
            Y = 5.0 - progress * 5.0      # 5m → 0m (moving left/center)
            
        else:
            # Final flat section at low altitude, centered
            Z = -1.0
            Y = 0.0
            
        Z_terr.append(Z)
        Y_terr.append(Y)

    Z_terr = np.array(Z_terr)
    Y_terr = np.array(Y_terr)
    
    # Calculate slope angles
    # Alpha: pitch angle (slope in X-Z plane)
    dZ_dX = np.gradient(Z_terr, dx)
    alpha = np.arctan(dZ_dX)
    
    # Beta: roll angle needed for lateral trajectory (slope in X-Y plane)
    dY_dX = np.gradient(Y_terr, dx)
    beta = np.arctan(dY_dX)

    # Reference trajectory: fly 2m above terrain
    # In NED: subtracting 2 means going more negative (higher altitude)
    Z_ref = Z_terr - 2.0  # Fly 2m above terrain
    Y_ref = Y_terr.copy()  # Follow terrain lateral position

    return X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta


def generate_reference_profile_2d(Vel, dt, X_max=50):
    """
    Generate 2D reference trajectory profile (backward compatibility).
    
    For 2D simulations (X-Z plane only, no Y motion).
    
    Returns:
    --------
    X_ref : ndarray
        Horizontal reference positions
    Z_terr : ndarray
        Terrain heights at each X (NED: negative values)
    Z_ref : ndarray
        Reference flight altitudes above terrain (NED: negative values)
    alpha : ndarray
        Slope angles of the terrain (in radians)
    """
    X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta = generate_reference_profile(Vel, dt, X_max)
    return X_ref, Z_terr, Z_ref, alpha


# ============================================================================
# TEST & VISUALIZATION
# ============================================================================

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    print("=== TRAJECTORY GENERATOR TEST ===\n")
    
    # Generate test trajectory
    Vel = 2.0  # m/s
    dt = 0.01  # s
    X_max = 50.0  # m
    
    X_ref, Y_terr, Z_terr, Y_ref, Z_ref, alpha, beta = generate_reference_profile(Vel, dt, X_max)
    
    print(f"Velocity: {Vel} m/s")
    print(f"Time step: {dt} s")
    print(f"X range: 0 to {X_max} m")
    print(f"Number of points: {len(X_ref)}")
    print(f"\nAltitude range (Z_terr): {Z_terr.min():.2f} to {Z_terr.max():.2f} m")
    print(f"Altitude range (Z_ref):  {Z_ref.min():.2f} to {Z_ref.max():.2f} m")
    print(f"Lateral range (Y_terr):  {Y_terr.min():.2f} to {Y_terr.max():.2f} m")
    print(f"Pitch angle range:       {np.degrees(alpha.min()):.2f}° to {np.degrees(alpha.max()):.2f}°")
    print(f"Roll angle range:        {np.degrees(beta.min()):.2f}° to {np.degrees(beta.max()):.2f}°")
    
    # Visualization
    fig = plt.figure(figsize=(16, 10))
    
    # Plot 1: X-Z plane (side view - pitch)
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(X_ref, Z_terr, 'g-', linewidth=2, label='Terrain')
    ax1.plot(X_ref, Z_ref, 'r--', linewidth=2, label='Reference flight path')
    ax1.axhline(y=0, color='brown', linestyle='--', linewidth=1, label='Ground', alpha=0.7)
    ax1.fill_between(X_ref, Z_terr, 0, alpha=0.3, color='green')
    ax1.set_xlabel('X [m] (Forward)')
    ax1.set_ylabel('Z [m] (NED: negative = up)')
    ax1.set_title('Side View: X-Z Plane (Pitch Control)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.invert_yaxis()
    
    # Plot 2: X-Y plane (top view - roll/lateral)
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(X_ref, Y_terr, 'g-', linewidth=2, label='Terrain lateral position')
    ax2.plot(X_ref, Y_ref, 'r--', linewidth=2, label='Reference lateral position')
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax2.set_xlabel('X [m] (Forward)')
    ax2.set_ylabel('Y [m] (Right)')
    ax2.set_title('Top View: X-Y Plane (Roll Control)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: 3D trajectory
    ax3 = plt.subplot(2, 2, 3, projection='3d')
    ax3.plot(X_ref, Y_terr, Z_terr, 'g-', linewidth=2, label='Terrain')
    ax3.plot(X_ref, Y_ref, Z_ref, 'r--', linewidth=2, label='Reference path')
    ax3.set_xlabel('X [m] (Forward)')
    ax3.set_ylabel('Y [m] (Right)')
    ax3.set_zlabel('Z [m] (Down)')
    ax3.set_title('3D Trajectory')
    ax3.legend()
    ax3.invert_zaxis()
    
    # Plot 4: Angles
    ax4 = plt.subplot(2, 2, 4)
    ax4.plot(X_ref, np.degrees(alpha), 'b-', linewidth=2, label='Alpha (pitch angle)')
    ax4.plot(X_ref, np.degrees(beta), 'r-', linewidth=2, label='Beta (roll angle)')
    ax4.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax4.set_xlabel('X [m] (Forward)')
    ax4.set_ylabel('Angle [degrees]')
    ax4.set_title('Required Angles for Trajectory Following')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    print("\n=== TEST COMPLETED ===")
