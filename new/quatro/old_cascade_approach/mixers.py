import numpy as np
from constants import T_max, T_min  # ZMIENIONE


def mixer_motors(moment_cmd, thrust_cmd, arm):
    """
    Convert moments + total thrust to motor thrusts

    Quadrocopter X-configuration:
    T_total = T1 + T2 + T3 + T4
    M_roll = (T3 + T4 - T1 - T2) * arm
    M_pitch = (T2 + T4 - T1 - T3) * arm
    M_yaw = (T1 + T3 - T2 - T4) * k_yaw
    """
    M_roll, M_pitch, M_yaw = moment_cmd
    k_yaw = 0.1

    T_avg = thrust_cmd / 4.0
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