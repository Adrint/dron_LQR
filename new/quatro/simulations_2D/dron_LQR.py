"""
Quadrocopter 2D Simulation - LQR Controller
Stabilized LQR for 2D terrain-following
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import math
import matplotlib.pyplot as plt

from common.integrator import rk45_simple
from common.linearization import linearize_system_simple
from common.lqr_controller import design_lqr
from common.trajectory import generate_reference_profile
from common.constants import (z0, Vel, MASS, g, T_max, T_min)


def main():
    print("=" * 80)
    print("QUADROCOPTER 2D - LQR CONTROLLER")
    print("=" * 80)
    
    dt = 0.01
    t = 0.0
    n = 12
    m = 4
    
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
    T_hover = MASS * g / 4.0
    u = np.array([T_hover, T_hover, T_hover, T_hover])
    K = None
    
    X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)
    
    tp, yp = [], []
    x_ref = 0.0
    y_ref = 0.0
    
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    print("\nRunning simulation...")
    
    for i in range(100000):
        if t >= 50.0:
            break
        
        X = x[6]
        idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
        z_ref = Z_ref_all[idx_ref]
        alfa = alpha_all[idx_ref]
        
        Vx_ref = Vel * np.cos(alfa)
        Vz_ref = Vel * np.sin(alfa)
        x_ref += Vx_ref * dt
        
        tp.append(t)
        yp.append(x.copy())
        
        # LQR tuning
        R = np.eye(m) * 10.0
        Q = np.eye(n)
        Q[0, 0] = 2.0
        Q[1, 1] = 2.0
        Q[2, 2] = 5.0
        Q[3, 3] = 0.1
        Q[4, 4] = 0.1
        Q[5, 5] = 0.1
        Q[6, 6] = 1.0
        Q[7, 7] = 1.0
        Q[8, 8] = 50.0
        Q[9, 9] = 20.0
        Q[10, 10] = 20.0
        Q[11, 11] = 5.0
        
        # Error
        e = np.zeros(n)
        e[0] = x[0] - Vx_ref
        e[1] = x[1] - 0.0
        e[2] = x[2] - Vz_ref
        e[3:6] = x[3:6]
        e[6] = x[6] - x_ref
        e[7] = x[7] - y_ref
        e[8] = x[8] - z_ref
        e[9:12] = x[9:12]
        
        # LQR (every 10 steps)
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
            u_pert = np.clip(u_pert, -50.0, 50.0)
        else:
            u_pert = np.zeros(m)
        
        T_baseline = MASS * g / 4.0
        u = np.clip(T_baseline + u_pert, T_min, T_max)
        
        if i % 500 == 0:
            print(f"t={t:6.2f}s | X={x[6]:7.2f} Z={x[8]:7.2f} (ref: {z_ref:7.2f})")
        
        x = rk45_simple(x, t, dt, u, 0.0, 0.0, 0.0)
        
        if i % 50 == 0:
            yp_array = np.array(yp)
            
            ax1.clear()
            ax1.fill_between(X_ref_all, 0, Z_terr_all, color='brown', alpha=0.3, label='Terrain')
            ax1.plot(X_ref_all, Z_ref_all, 'r--', label='Reference', linewidth=2)
            ax1.plot(yp_array[:, 6], yp_array[:, 8], 'b-', label='Actual', linewidth=2)
            ax1.scatter([x[6]], [x[8]], c='blue', s=150, marker='o')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Z [m]')
            ax1.set_xlim([0, 50])
            ax1.set_ylim([0, 12])
            ax1.grid(True, alpha=0.3)
            ax1.legend()
            ax1.set_title(f'2D Trajectory - LQR (t={t:.1f}s)')
            
            ax2.clear()
            ax2.plot(tp, yp_array[:, 8], 'b-', label='Altitude', linewidth=2)
            ax2.plot(tp, [Z_ref_all[np.argmin(np.abs(X_ref_all - yp_array[j, 6]))] for j in range(len(yp_array))],
                     'r--', label='Reference', linewidth=2)
            ax2.set_xlabel('Time [s]')
            ax2.set_ylabel('Z [m]')
            ax2.set_xlim([0, 50])
            ax2.grid(True, alpha=0.3)
            ax2.legend()
            ax2.set_title('Altitude vs Time')
            
            plt.draw()
            plt.pause(0.001)
        
        t = t + dt
    
    print("\n" + "=" * 80)
    print("Simulation completed!")
    print("=" * 80)
    
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
