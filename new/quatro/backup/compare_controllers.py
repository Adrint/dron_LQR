"""
Porównanie kontrolerów PID i LQR dla trajektorii 3D
Uruchamia obie symulacje i pokazuje wyniki obok siebie
"""

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from integrator import rk45_simple
from linearization import linearize_system_simple
from lqr_controller import design_lqr
from trajectory import generate_3d_trajectory
from constants import (z0, Vel, MASS, g, T_max, T_min, MOTOR_ARM_LENGTH, IX, IY, IZ)

print("=" * 80)
print("PORÓWNANIE KONTROLERÓW: PID vs LQR")
print("=" * 80)

# Parametry symulacji
dt = 0.01
total_time = 30.0  # Krótszy czas dla porównania
n = 12
m = 4

# Generuj trajektorię
print("\nGenerowanie trajektorii 3D...")
t_traj, X_traj, Y_traj, Z_traj, Vx_traj, Vy_traj, Vz_traj = generate_3d_trajectory(
    Vel, dt, total_time
)
print(f"Wygenerowano {len(t_traj)} punktów")

# ============================================================================
# SYMULACJA 1: PID
# ============================================================================

print("\n" + "=" * 80)
print("SYMULACJA 1: Kontroler PID")
print("=" * 80)

# Kontroler PID
class Quad3DController:
    def __init__(self):
        self.kp_z = 25.0
        self.kd_z = 12.0
        self.kp_x = 3.0
        self.kd_x = 4.0
        self.kp_y = 3.0
        self.kd_y = 4.0
        self.kp_att = 6.0
        self.kd_att = 3.0
        self.kp_yaw = 3.0
        self.kd_yaw = 1.5
        
    def compute_thrust(self, state, ref_pos, ref_vel):
        vx, vy, vz = state[0:3]
        p, q, r = state[3:6]
        X, Y, Z = state[6:9]
        phi, theta, psi = state[9:12]
        
        x_ref, y_ref, z_ref = ref_pos
        vx_ref, vy_ref, vz_ref = ref_vel
        
        e_z = z_ref - Z
        e_vz = vz_ref - vz
        T_total = MASS * g + self.kp_z * e_z + self.kd_z * e_vz
        T_total = np.clip(T_total, MASS * g * 0.3, MASS * g * 2.5)
        
        e_x = x_ref - X
        e_y = y_ref - Y
        e_vx = vx_ref - vx
        e_vy = vy_ref - vy
        
        theta_des = np.clip(self.kp_x * e_x + self.kd_x * e_vx, -0.4, 0.4)
        phi_des = np.clip(self.kp_y * e_y + self.kd_y * e_vy, -0.4, 0.4)
        
        if abs(vx) > 0.5 or abs(vy) > 0.5:
            psi_des = np.arctan2(vy, vx)
        else:
            psi_des = 0.0
        
        e_phi = phi_des - phi
        e_theta = theta_des - theta
        e_psi = np.arctan2(np.sin(psi_des - psi), np.cos(psi_des - psi))
        
        M_roll = self.kp_att * e_phi - self.kd_att * p
        M_pitch = self.kp_att * e_theta - self.kd_att * q
        M_yaw = self.kp_yaw * e_psi - self.kd_yaw * r
        
        M_roll = np.clip(M_roll, -15.0, 15.0)
        M_pitch = np.clip(M_pitch, -15.0, 15.0)
        M_yaw = np.clip(M_yaw, -3.0, 3.0)
        
        arm = MOTOR_ARM_LENGTH / np.sqrt(2)
        k_yaw = 0.1
        T_avg = T_total / 4.0
        
        dT_roll = M_roll / (2 * arm)
        dT_pitch = M_pitch / (2 * arm)
        dT_yaw = M_yaw / (2 * k_yaw)
        
        T1 = T_avg - dT_roll - dT_pitch + dT_yaw
        T2 = T_avg - dT_roll + dT_pitch - dT_yaw
        T3 = T_avg + dT_roll - dT_pitch - dT_yaw
        T4 = T_avg + dT_roll + dT_pitch + dT_yaw
        
        thrusts = np.array([T1, T2, T3, T4])
        thrusts = np.clip(thrusts, T_min, T_max)
        
        return thrusts

# Stan początkowy PID
x_pid = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
T_hover = MASS * g / 4.0
u_pid = np.array([T_hover, T_hover, T_hover, T_hover])

controller_pid = Quad3DController()

# Storage PID
tp_pid = []
yp_pid = []
up_pid = []

# Symuluj PID
print("Uruchamianie symulacji PID...")
for i in range(len(t_traj) - 1):
    t = t_traj[i]
    
    x_ref = X_traj[i]
    y_ref = Y_traj[i]
    z_ref = Z_traj[i]
    vx_ref = Vx_traj[i]
    vy_ref = Vy_traj[i]
    vz_ref = Vz_traj[i]
    
    tp_pid.append(t)
    yp_pid.append(x_pid.copy())
    up_pid.append(u_pid.copy())
    
    ref_pos = np.array([x_ref, y_ref, z_ref])
    ref_vel = np.array([vx_ref, vy_ref, vz_ref])
    u_pid = controller_pid.compute_thrust(x_pid, ref_pos, ref_vel)
    
    x_pid = rk45_simple(x_pid, t, dt, u_pid, 0.0, 0.0, 0.0)
    
    if i % 500 == 0:
        print(f"  PID: t={t:.1f}s, X={x_pid[6]:.1f}, Y={x_pid[7]:.1f}, Z={x_pid[8]:.1f}")

yp_pid = np.array(yp_pid)
up_pid = np.array(up_pid)
print("✓ Symulacja PID zakończona")

# ============================================================================
# SYMULACJA 2: LQR
# ============================================================================

print("\n" + "=" * 80)
print("SYMULACJA 2: Kontroler LQR")
print("=" * 80)

# Stan początkowy LQR
x_lqr = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])
u_lqr = np.array([T_hover, T_hover, T_hover, T_hover])
K_lqr = None

# Storage LQR
tp_lqr = []
yp_lqr = []
up_lqr = []

# Symuluj LQR
print("Uruchamianie symulacji LQR...")
for i in range(len(t_traj) - 1):
    t = t_traj[i]
    
    x_ref = X_traj[i]
    y_ref = Y_traj[i]
    z_ref = Z_traj[i]
    vx_ref = Vx_traj[i]
    vy_ref = Vy_traj[i]
    vz_ref = Vz_traj[i]
    
    tp_lqr.append(t)
    yp_lqr.append(x_lqr.copy())
    up_lqr.append(u_lqr.copy())
    
    # Macierze LQR
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
    
    # Błędy
    e = np.zeros(n)
    e[0] = x_lqr[0] - vx_ref
    e[1] = x_lqr[1] - vy_ref
    e[2] = x_lqr[2] - vz_ref
    e[3:6] = x_lqr[3:6]
    e[6] = x_lqr[6] - x_ref
    e[7] = x_lqr[7] - y_ref
    e[8] = x_lqr[8] - z_ref
    e[9:12] = x_lqr[9:12]
    
    # LQR co 10 kroków
    if i % 10 == 0:
        try:
            A, B = linearize_system_simple(x_lqr, t, u_lqr, n, m)
            K_new, P = design_lqr(A, B, Q, R)
            if K_new is not None and not np.any(np.isnan(K_new)):
                K_lqr = K_new
        except:
            pass
    
    if K_lqr is not None:
        u_pert = -K_lqr @ e
        u_pert = np.clip(u_pert, -60.0, 60.0)
    else:
        u_pert = np.zeros(m)
    
    T_baseline = MASS * g / 4.0
    u_lqr = np.clip(T_baseline + u_pert, T_min, T_max)
    
    x_lqr = rk45_simple(x_lqr, t, dt, u_lqr, 0.0, 0.0, 0.0)
    
    if i % 500 == 0:
        print(f"  LQR: t={t:.1f}s, X={x_lqr[6]:.1f}, Y={x_lqr[7]:.1f}, Z={x_lqr[8]:.1f}")

yp_lqr = np.array(yp_lqr)
up_lqr = np.array(up_lqr)
print("✓ Symulacja LQR zakończona")

# ============================================================================
# ANALIZA I PORÓWNANIE
# ============================================================================

print("\n" + "=" * 80)
print("ANALIZA WYNIKÓW")
print("=" * 80)

# Błędy śledzenia
e_x_pid = X_traj[:len(tp_pid)] - yp_pid[:, 6]
e_y_pid = Y_traj[:len(tp_pid)] - yp_pid[:, 7]
e_z_pid = Z_traj[:len(tp_pid)] - yp_pid[:, 8]
e_pos_pid = np.sqrt(e_x_pid**2 + e_y_pid**2 + e_z_pid**2)

e_x_lqr = X_traj[:len(tp_lqr)] - yp_lqr[:, 6]
e_y_lqr = Y_traj[:len(tp_lqr)] - yp_lqr[:, 7]
e_z_lqr = Z_traj[:len(tp_lqr)] - yp_lqr[:, 8]
e_pos_lqr = np.sqrt(e_x_lqr**2 + e_y_lqr**2 + e_z_lqr**2)

print("\n📊 BŁĘDY POZYCJI:")
print(f"{'Metryka':<25} {'PID':>12} {'LQR':>12} {'Lepszy':>12}")
print("-" * 65)
print(f"{'Średni błąd X [m]':<25} {np.mean(np.abs(e_x_pid)):>12.3f} {np.mean(np.abs(e_x_lqr)):>12.3f} {'PID' if np.mean(np.abs(e_x_pid)) < np.mean(np.abs(e_x_lqr)) else 'LQR':>12}")
print(f"{'Średni błąd Y [m]':<25} {np.mean(np.abs(e_y_pid)):>12.3f} {np.mean(np.abs(e_y_lqr)):>12.3f} {'PID' if np.mean(np.abs(e_y_pid)) < np.mean(np.abs(e_y_lqr)) else 'LQR':>12}")
print(f"{'Średni błąd Z [m]':<25} {np.mean(np.abs(e_z_pid)):>12.3f} {np.mean(np.abs(e_z_lqr)):>12.3f} {'PID' if np.mean(np.abs(e_z_pid)) < np.mean(np.abs(e_z_lqr)) else 'LQR':>12}")
print(f"{'Średni błąd 3D [m]':<25} {np.mean(e_pos_pid):>12.3f} {np.mean(e_pos_lqr):>12.3f} {'PID' if np.mean(e_pos_pid) < np.mean(e_pos_lqr) else 'LQR':>12}")
print(f"{'Max błąd 3D [m]':<25} {np.max(e_pos_pid):>12.3f} {np.max(e_pos_lqr):>12.3f} {'PID' if np.max(e_pos_pid) < np.max(e_pos_lqr) else 'LQR':>12}")

print("\n📐 KĄTY:")
print(f"{'Metryka':<25} {'PID':>12} {'LQR':>12}")
print("-" * 50)
print(f"{'Max roll [°]':<25} {math.degrees(np.max(np.abs(yp_pid[:, 9]))):>12.1f} {math.degrees(np.max(np.abs(yp_lqr[:, 9]))):>12.1f}")
print(f"{'Max pitch [°]':<25} {math.degrees(np.max(np.abs(yp_pid[:, 10]))):>12.1f} {math.degrees(np.max(np.abs(yp_lqr[:, 10]))):>12.1f}")

print("\n⚡ STEROWANIE:")
total_thrust_pid = np.sum(up_pid, axis=1)
total_thrust_lqr = np.sum(up_lqr, axis=1)
print(f"{'Metryka':<25} {'PID':>12} {'LQR':>12}")
print("-" * 50)
print(f"{'Średni ciąg [N]':<25} {np.mean(total_thrust_pid):>12.1f} {np.mean(total_thrust_lqr):>12.1f}")
print(f"{'Max ciąg [N]':<25} {np.max(total_thrust_pid):>12.1f} {np.max(total_thrust_lqr):>12.1f}")
print(f"{'Min ciąg [N]':<25} {np.min(total_thrust_pid):>12.1f} {np.min(total_thrust_lqr):>12.1f}")

# Zmienność sterowania (smooth vs aggressive)
thrust_var_pid = np.std(np.diff(up_pid, axis=0))
thrust_var_lqr = np.std(np.diff(up_lqr, axis=0))
print(f"{'Zmienność sterowania':<25} {thrust_var_pid:>12.2f} {thrust_var_lqr:>12.2f}")

# ============================================================================
# WIZUALIZACJA PORÓWNAWCZA
# ============================================================================

print("\n" + "=" * 80)
print("TWORZENIE WYKRESÓW PORÓWNAWCZYCH...")
print("=" * 80)

fig = plt.figure(figsize=(20, 12))
fig.suptitle('Porównanie kontrolerów: PID vs LQR', fontsize=16, fontweight='bold')

# 1. Trajektorie 3D obok siebie
ax1 = fig.add_subplot(2, 4, 1, projection='3d')
ax1.plot(X_traj, Y_traj, Z_traj, 'k--', label='Referencja', linewidth=2, alpha=0.5)
ax1.plot(yp_pid[:, 6], yp_pid[:, 7], yp_pid[:, 8], 'b-', label='PID', linewidth=2)
ax1.scatter([yp_pid[-1, 6]], [yp_pid[-1, 7]], [yp_pid[-1, 8]], c='blue', s=100, marker='o')
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_zlabel('Z [m]')
ax1.set_title('PID - Trajektoria 3D', fontweight='bold')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.view_init(elev=20, azim=45)

ax2 = fig.add_subplot(2, 4, 2, projection='3d')
ax2.plot(X_traj, Y_traj, Z_traj, 'k--', label='Referencja', linewidth=2, alpha=0.5)
ax2.plot(yp_lqr[:, 6], yp_lqr[:, 7], yp_lqr[:, 8], 'r-', label='LQR', linewidth=2)
ax2.scatter([yp_lqr[-1, 6]], [yp_lqr[-1, 7]], [yp_lqr[-1, 8]], c='red', s=100, marker='o')
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Y [m]')
ax2.set_zlabel('Z [m]')
ax2.set_title('LQR - Trajektoria 3D', fontweight='bold')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.view_init(elev=20, azim=45)

# 2. Błędy pozycji XYZ
ax3 = fig.add_subplot(2, 4, 3)
ax3.plot(tp_pid, e_pos_pid, 'b-', label='PID', linewidth=2)
ax3.plot(tp_lqr, e_pos_lqr, 'r-', label='LQR', linewidth=2)
ax3.set_xlabel('Czas [s]')
ax3.set_ylabel('Błąd 3D [m]')
ax3.set_title('Błąd pozycji 3D', fontweight='bold')
ax3.legend()
ax3.grid(True, alpha=0.3)

# 3. Błąd wysokości
ax4 = fig.add_subplot(2, 4, 4)
ax4.plot(tp_pid, np.abs(e_z_pid), 'b-', label='PID', linewidth=2)
ax4.plot(tp_lqr, np.abs(e_z_lqr), 'r-', label='LQR', linewidth=2)
ax4.set_xlabel('Czas [s]')
ax4.set_ylabel('|Błąd Z| [m]')
ax4.set_title('Błąd wysokości', fontweight='bold')
ax4.legend()
ax4.grid(True, alpha=0.3)

# 4. Kąty roll
ax5 = fig.add_subplot(2, 4, 5)
ax5.plot(tp_pid, np.degrees(yp_pid[:, 9]), 'b-', label='PID', linewidth=2)
ax5.plot(tp_lqr, np.degrees(yp_lqr[:, 9]), 'r-', label='LQR', linewidth=2)
ax5.set_xlabel('Czas [s]')
ax5.set_ylabel('Roll φ [°]')
ax5.set_title('Kąt przechylenia (roll)', fontweight='bold')
ax5.legend()
ax5.grid(True, alpha=0.3)

# 5. Kąty pitch
ax6 = fig.add_subplot(2, 4, 6)
ax6.plot(tp_pid, np.degrees(yp_pid[:, 10]), 'b-', label='PID', linewidth=2)
ax6.plot(tp_lqr, np.degrees(yp_lqr[:, 10]), 'r-', label='LQR', linewidth=2)
ax6.set_xlabel('Czas [s]')
ax6.set_ylabel('Pitch θ [°]')
ax6.set_title('Kąt pochylenia (pitch)', fontweight='bold')
ax6.legend()
ax6.grid(True, alpha=0.3)

# 6. Całkowity ciąg
ax7 = fig.add_subplot(2, 4, 7)
ax7.plot(tp_pid, total_thrust_pid, 'b-', label='PID', linewidth=2)
ax7.plot(tp_lqr, total_thrust_lqr, 'r-', label='LQR', linewidth=2)
ax7.axhline(y=MASS*g, color='k', linestyle='--', alpha=0.5, label='Hover')
ax7.set_xlabel('Czas [s]')
ax7.set_ylabel('Całkowity ciąg [N]')
ax7.set_title('Całkowity ciąg silników', fontweight='bold')
ax7.legend()
ax7.grid(True, alpha=0.3)

# 7. Widok z góry (XY)
ax8 = fig.add_subplot(2, 4, 8)
ax8.plot(X_traj, Y_traj, 'k--', label='Referencja', linewidth=2, alpha=0.5)
ax8.plot(yp_pid[:, 6], yp_pid[:, 7], 'b-', label='PID', linewidth=2)
ax8.plot(yp_lqr[:, 6], yp_lqr[:, 7], 'r-', label='LQR', linewidth=2)
ax8.set_xlabel('X [m]')
ax8.set_ylabel('Y [m]')
ax8.set_title('Widok z góry (XY)', fontweight='bold')
ax8.legend()
ax8.grid(True, alpha=0.3)
ax8.axis('equal')

plt.tight_layout()

print("✓ Wykresy gotowe!")
print("\nZamknij okno aby zakończyć.")
print("=" * 80)

plt.show()
