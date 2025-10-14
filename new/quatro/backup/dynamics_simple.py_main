import numpy as np
from constants import CD_0, RO_0, MASS, g, S, CM_Q, IX, IY, IZ, MOTOR_ARM_LENGTH

def quadrocopter_dynamics_simple(x, t, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    dx_dt = np.zeros(12)
    vx, vy, vz = x[0:3]
    p, q, r = x[3:6]
    X, Y, Z = x[6:9]
    phi, theta, psi = x[9:12]
    
    T1, T2, T3, T4 = u
    T_total = T1 + T2 + T3 + T4
    
    V = np.sqrt(vx**2 + vy**2 + vz**2)
    ro = max(RO_0 * (1.0 - abs(Z) / 44300.0) ** 4.256, 0.0)
    
    if V > 0.01:
        Q_dyn = 0.5 * ro * V ** 2
        D_mag = Q_dyn * S * CD_0
        Dx, Dy, Dz = D_mag * vx / V, D_mag * vy / V, D_mag * vz / V
    else:
        Dx, Dy, Dz = 0.0, 0.0, 0.0
    
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_theta, s_theta = np.cos(theta), np.sin(theta)
    c_psi, s_psi = np.cos(psi), np.sin(psi)
    
    R11 = c_psi*c_theta
    R12 = c_psi*s_theta*s_phi - s_psi*c_phi
    R13 = c_psi*s_theta*c_phi + s_psi*s_phi
    R21 = s_psi*c_theta
    R22 = s_psi*s_theta*s_phi + c_psi*c_phi
    R23 = s_psi*s_theta*c_phi - c_psi*s_phi
    R31 = -s_theta
    R32 = c_theta*s_phi
    R33 = c_theta*c_phi
    
    Fx = R13 * T_total
    Fy = R23 * T_total
    Fz = R33 * T_total
    
    dx_dt[0] = (Fx - Dx) / MASS
    dx_dt[1] = (Fy - Dy) / MASS
    dx_dt[2] = (Fz - MASS*g - Dz) / MASS + az_turbulence
    
    arm = MOTOR_ARM_LENGTH / np.sqrt(2)
    M_roll = (T3 + T4 - T1 - T2) * arm
    M_pitch = (T2 + T4 - T1 - T3) * arm
    M_yaw = (T1 + T3 - T2 - T4) * 0.1
    
    dx_dt[3] = (M_roll + CM_Q * p) / IX
    dx_dt[4] = (M_pitch + CM_Q * q) / IY
    dx_dt[5] = (M_yaw + CM_Q * r) / IZ
    
    dx_dt[6], dx_dt[7], dx_dt[8] = vx, vy, vz
    
    if abs(np.cos(theta)) > 0.01:
        dx_dt[9] = p + q*s_phi*np.tan(theta) + r*c_phi*np.tan(theta)
        dx_dt[10] = q*c_phi - r*s_phi
        dx_dt[11] = (q*s_phi + r*c_phi) / np.cos(theta)
    else:
        dx_dt[9], dx_dt[10], dx_dt[11] = p, q*c_phi - r*s_phi, 0.0
    
    return dx_dt
