import numpy as np
from src_new.old_cascade_approach.dynamics import quadrocopter_dynamics


def rk45(x, t, dt, u, az_turbulence=0.0, ax_wind=0.0, az_wind=0.0):
    """Runge-Kutta 4-5 integration"""
    y = np.zeros(len(x))

    y0 = quadrocopter_dynamics(x, t, u, az_turbulence, ax_wind, az_wind)

    t1 = t + dt * 0.25
    vec = x + dt * 0.25 * y0
    y1 = quadrocopter_dynamics(vec, t1, u, az_turbulence, ax_wind, az_wind)

    t2 = t + dt * (3.0 / 8.0)
    vec = x + dt * ((3.0 / 32.0) * y0 + (9.0 / 32.0) * y1)
    y2 = quadrocopter_dynamics(vec, t2, u, az_turbulence, ax_wind, az_wind)

    t3 = t + dt * (12.0 / 13.0)
    vec = x + dt * ((1932.0 / 2197.0) * y0 + (-7200.0 / 2197.0) * y1 + (7296.0 / 2197.0) * y2)
    y3 = quadrocopter_dynamics(vec, t3, u, az_turbulence, ax_wind, az_wind)

    t4 = t + dt
    vec = x + dt * ((439.0 / 216.0) * y0 + (-8.0) * y1 + (3680.0 / 513.0) * y2 + (-845.0 / 4104.0) * y3)
    y4 = quadrocopter_dynamics(vec, t4, u, az_turbulence, ax_wind, az_wind)

    t5 = t + dt * (1.0 / 2.0)
    vec = x + dt * (
                -(8.0 / 27.0) * y0 + 2.0 * y1 + (-3544.0 / 2565.0) * y2 + (1859.0 / 4104.0) * y3 + (-11.0 / 40.0) * y4)
    y5 = quadrocopter_dynamics(vec, t5, u, az_turbulence, ax_wind, az_wind)

    y = x + dt * ((16.0 / 135.0) * y0 + (6656.0 / 12825.0) * y2 + (28561.0 / 56430.0) * y3 + (-9.0 / 50.0) * y4 + (
                2.0 / 55.0) * y5)

    return y