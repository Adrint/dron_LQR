import numpy as np
import math
from constants import (z0, Vel, MASS, g, IX, IY, IZ, MOTOR_ARM_LENGTH)
from cascade_lqr import CascadeLQRController
from trajectory import generate_reference_profile
from simulator import QuadrocopterSimulator
from plotter import Plotter

# Initialize
dt = 0.01
max_time = 50.0

# Initial state
x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, z0, 0.0, 0.0, 0.0])

# Drone parameters
drone_params = {
    'mass': MASS,
    'g': g,
    'ix': IX,
    'iy': IY,
    'iz': IZ,
    'arm': MOTOR_ARM_LENGTH / np.sqrt(2)
}

# Initialize controller and simulator
controller = CascadeLQRController(drone_params)
simulator = QuadrocopterSimulator(dt)
plotter = Plotter()

# Reference trajectory
X_ref_all, Z_terr_all, Z_ref_all, alpha_all = generate_reference_profile(Vel, dt, 50)

x_ref = 0.0
yp = []

print("=" * 80)
print("QUADROCOPTER SIMULATION - CASCADE LQR CONTROL")
print("=" * 80)

# Main loop
for step in range(int(max_time / dt)):
    if simulator.t >= max_time:
        break

    X = x[6]
    idx_ref = np.argmin(np.abs(X_ref_all - x_ref))
    z_terr = Z_terr_all[idx_ref]
    z_ref = Z_ref_all[idx_ref]
    alfa = alpha_all[idx_ref]

    Vx_ref = Vel * np.cos(alfa)
    x_ref += Vx_ref * dt

    ref_pos = np.array([x_ref, 0.0, z_ref])

    # Control
    u = controller.update(x, ref_pos, dt)

    # Simulation step
    x = simulator.step(x, u)
    yp.append(x.copy())

    # Diagnostics
    if step % 100 == 0:
        vx, vy, vz = x[0:3]
        Z_pos = x[8]
        z_error = Z_pos - z_ref

        print(f"t={simulator.t:7.3f}s | Z={Z_pos:6.2f}m Z_ref={z_ref:6.2f}m ΔZ={z_error:6.3f}m | "
              f"v={np.linalg.norm([vx, vy, vz]):5.2f}m/s | T_sum={u.sum():7.1f}N")

    # Visualization
    if step % 50 == 0:
        yp_array = np.array(yp)
        plotter.update(X_ref_all, Z_terr_all, Z_ref_all, yp_array, x)

print("\n" + "=" * 80)
print("Simulation completed!")
print("=" * 80)