import numpy as np
from integrator import rk45


class QuadrocopterSimulator:
    """Main simulator class"""

    def __init__(self, dt=0.01):
        self.dt = dt
        self.t = 0.0
        self.state_history = []
        self.control_history = []
        self.time_history = []

    def step(self, state, control, disturbances=(0.0, 0.0, 0.0)):
        """Single simulation step"""
        az_turb, ax_wind, az_wind = disturbances

        state_new = rk45(state, self.t, self.dt, control, az_turb, ax_wind, az_wind)

        self.time_history.append(self.t)
        self.state_history.append(state.copy())
        self.control_history.append(control.copy())

        self.t += self.dt

        return state_new