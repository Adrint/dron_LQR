"""
Quadrocopter 2D Simulation - PID Controller  
Clean 2D terrain-following implementation
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import math
import matplotlib.pyplot as plt

from common.integrator import rk45_simple
from common.trajectory import generate_reference_profile
from common.constants import (z0, Vel, MASS, g, T_max, T_min, MOTOR_ARM_LENGTH)


class Quadcopter2DController:
    def __init__(self):
        self.kp_z = 20.0
        self.kd_z = 10.0
        self.kp_xy = 2.0
        self.kd_xy = 3.0
        self.kp_att = 5.0
        self.kd_att = 2.0
        
    def compute_thrust(self, state, ref_pos):
        vx, vy, vz = state[0:3]
        p, q, r = state[3:6]
        X, Y, Z = state[6:9]
        phi, theta, psi = state[9:12]
        
        x_ref, y_ref, z_ref = ref_pos
        
        e_z = z_ref - Z
        e_vz = 0.0 - vz
        T_total = MASS * g + self.kp_z * e_z + self.kd_z * e_vz
        T_total = np.clip(T_total, MASS * g * 0.3, MASS * g * 2.5)
        
        e_x = x_ref - X
        theta_des = np.clip(self.kp_xy * e_x - self.kd_xy * vx, -0.3, 0.3)
        phi_des = 0.0
        
        e_phi = phi_des - phi
        e_theta = theta_des - theta
        
        M_roll = self.kp_att * e_phi - self.kd_att * p
        M_pitch = self.kp_att * e_theta - self.kd_att * q
        M_yaw = 0.0
        
        M_roll = np.clip(M_roll, -10.0, 10.0)
        M_pitch = np.clip(M_pitch, -10.0, 10.0)
        
        arm = MOTOR_ARM_LENGTH / np.sqrt(2)
        T_avg = T_total / 4.0
        
        dT_roll = M_roll / (2 * arm)
        dT_pitch = M_pitch / (2 * arm)
        
        T1 = T_avg - dT_roll - dT_pitch
        T2 = T_avg - dT_roll + dT_pitch
        T3 = T_avg + dT_roll - dT_pitch
        T4 = T_avg + dT_roll + dT_pitch
        
        thrusts = np.array([T1, T2, T3, T4])
        thrusts = np.clip(thrusts, T_min, T_max)
        
        return thrusts


def main():
    print("=" * 80)
    print("QUADROCOPTER 2D - PID CONTROLLER")
    print("=" * 80)
    
    dt = 0.01
    t = 0.0
    
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
    T_hover = MASS * g / 4.0
    u = np.array([T_hover, T_hover, T_hover, T_hover])
    
    X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)
    
    controller = Quadcopter2DController()
    tp, yp = [], []
    
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    x_ref = 0.0
    y_ref = 0.0
    
    print("\nRunning simulation...")
    
    for i in range(100000):
        if t >= 50.0:
            break
        
        X = x[6]
        idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
        z_ref = Z_ref_all[idx_ref]
        alfa = alpha_all[idx_ref]
        
        Vx_ref = Vel * np.cos(alfa)
        x_ref += Vx_ref * dt
        
        tp.append(t)
        yp.append(x.copy())
        
        ref_pos = np.array([x_ref, y_ref, z_ref])
        u = controller.compute_thrust(x, ref_pos)
        
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
            ax1.set_title(f'2D Trajectory (t={t:.1f}s)')
            
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
