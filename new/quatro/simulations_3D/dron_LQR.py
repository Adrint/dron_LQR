"""
Quadrocopter 3D Simulation - LQR Controller
Clean and organized implementation with proper 3D visualization
"""

import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from common.integrator import rk45_simple
from common.linearization import linearize_system_simple
from common.lqr_controller import design_lqr
from common.trajectory import generate_3d_trajectory
from common.constants import (z0, Vel, MASS, g, T_max, T_min)


def main():
    print("=" * 80)
    print("QUADROCOPTER 3D SIMULATION - LQR CONTROLLER")
    print("=" * 80)
    
    # Simulation parameters
    dt = 0.01
    total_time = 50.0
    n = 12  # State dimension
    m = 4   # Control inputs
    
    # Initial state
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
    T_hover = MASS * g / 4.0
    u = np.array([T_hover, T_hover, T_hover, T_hover])
    K = None
    
    # Generate trajectory
    print("\nGenerating 3D trajectory...")
    t_traj, X_traj, Y_traj, Z_traj, Z_terr, Vx_traj, Vy_traj, Vz_traj = generate_3d_trajectory(Vel, dt, total_time)
    print(f"Generated {len(t_traj)} trajectory points")
    
    tp, yp, up = [], [], []
    
    # Setup plots
    plt.ion()
    fig = plt.figure(figsize=(18, 10))
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax2 = fig.add_subplot(2, 3, 2)
    ax3 = fig.add_subplot(2, 3, 3)
    ax4 = fig.add_subplot(2, 3, 4)
    ax5 = fig.add_subplot(2, 3, 5)
    ax6 = fig.add_subplot(2, 3, 6)
    plt.tight_layout()
    plt.pause(0.1)
    
    print("\nStarting simulation...")
    
    for i in range(len(t_traj) - 1):
        t = t_traj[i]
        
        x_ref = X_traj[i]
        y_ref = Y_traj[i]
        z_ref = Z_traj[i]
        vx_ref = Vx_traj[i]
        vy_ref = Vy_traj[i]
        vz_ref = Vz_traj[i]
        
        tp.append(t)
        yp.append(x.copy())
        up.append(u.copy())
        
        # LQR tuning
        R = np.eye(m) * 15.0
        Q = np.eye(n)
        Q[0, 0] = 3.0
        Q[1, 1] = 3.0
        Q[2, 2] = 5.0
        Q[3, 3] = 0.1
        Q[4, 4] = 0.1
        Q[5, 5] = 0.1
        Q[6, 6] = 2.0
        Q[7, 7] = 2.0
        Q[8, 8] = 50.0
        Q[9, 9] = 25.0
        Q[10, 10] = 25.0
        Q[11, 11] = 5.0
        
        # Errors
        e = np.zeros(n)
        e[0] = x[0] - vx_ref
        e[1] = x[1] - vy_ref
        e[2] = x[2] - vz_ref
        e[3:6] = x[3:6]
        e[6] = x[6] - x_ref
        e[7] = x[7] - y_ref
        e[8] = x[8] - z_ref
        e[9:12] = x[9:12]
        
        if i % 1000 == 0:
            e_pos = np.sqrt(e[6]**2 + e[7]**2 + e[8]**2)
            print(f"t={t:6.2f}s | X={x[6]:6.2f} Y={x[7]:6.2f} Z={x[8]:6.2f} | Error={e_pos:5.3f}m")
        
        # Compute LQR
        if i % 10 == 0:
            try:
                A, B = linearize_system_simple(x, t, u, n, m)
                K_new, P = design_lqr(A, B, Q, R)
                if K_new is not None and not np.any(np.isnan(K_new)):
                    K = K_new
            except:
                pass
        
        # Control
        if K is not None:
            u_pert = -K @ e
            u_pert = np.clip(u_pert, -60.0, 60.0)
        else:
            u_pert = np.zeros(m)
        
        T_baseline = MASS * g / 4.0
        u = np.clip(T_baseline + u_pert, T_min, T_max)
        
        # Integrate
        x = rk45_simple(x, t, dt, u, 0.0, 0.0, 0.0)
        
        # Update plots
        if i % 100 == 0:
            yp_array = np.array(yp)
            up_array = np.array(up)
            
            ax1.clear()
            ax1.plot(X_traj, Y_traj, Z_traj, 'r--', linewidth=2, alpha=0.5)
            ax1.plot(yp_array[:, 6], yp_array[:, 7], yp_array[:, 8], 'b-', linewidth=2)
            ax1.scatter([x[6]], [x[7]], [x[8]], c='blue', s=100)
            ax1.set_title(f'3D Trajectory - LQR (t={t:.1f}s)')
            ax1.view_init(elev=25, azim=45)
            
            ax2.clear()
            ax2.fill_between(X_traj, 0, Z_terr, color='brown', alpha=0.3)
            ax2.plot(X_traj, Z_traj, 'r--', linewidth=2)
            ax2.plot(yp_array[:, 6], yp_array[:, 8], 'b-', linewidth=2)
            ax2.set_title('Z(X)')
            ax2.grid(True)
            
            ax3.clear()
            ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            ax3.plot(X_traj, Y_traj, 'r--', linewidth=2)
            ax3.plot(yp_array[:, 6], yp_array[:, 7], 'b-', linewidth=2)
            ax3.set_title('Y(X)')
            ax3.grid(True)
            
            ax4.clear()
            ax4.plot(tp, yp_array[:, 6], 'r-', label='X')
            ax4.plot(tp, yp_array[:, 7], 'g-', label='Y')
            ax4.plot(tp, yp_array[:, 8], 'b-', label='Z')
            ax4.legend()
            ax4.grid(True)
            
            ax5.clear()
            ax5.plot(tp, np.degrees(yp_array[:, 9]), 'r-', label='φ')
            ax5.plot(tp, np.degrees(yp_array[:, 10]), 'g-', label='θ')
            ax5.plot(tp, np.degrees(yp_array[:, 11]), 'b-', label='ψ')
            ax5.legend()
            ax5.grid(True)
            
            ax6.clear()
            ax6.plot(tp, up_array[:, 0], label='T1')
            ax6.plot(tp, up_array[:, 1], label='T2')
            ax6.plot(tp, up_array[:, 2], label='T3')
            ax6.plot(tp, up_array[:, 3], label='T4')
            ax6.legend()
            ax6.grid(True)
            
            plt.draw()
            plt.pause(0.001)
    
    print("\n" + "=" * 80)
    print("SIMULATION COMPLETED")
    print("=" * 80)
    
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
