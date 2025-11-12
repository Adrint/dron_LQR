import numpy as np
import matplotlib.pyplot as plt


class Plotter:
    """Visualization class"""

    def __init__(self, figsize=(14, 6)):
        self.fig = None
        self.ax1 = None
        self.ax2 = None
        self.setup_plot(figsize)

    def setup_plot(self, figsize):
        """Setup matplotlib"""
        plt.ion()
        self.fig = plt.figure(1, figsize=figsize)
        self.ax1 = self.fig.add_subplot(121, projection='3d')
        self.ax2 = self.fig.add_subplot(122)

    def update(self, x_ref_all, z_terr_all, z_ref_all, yp_array, x_current):
        """Update visualization"""
        self.ax1.clear()
        self.ax1.plot(x_ref_all, np.zeros_like(x_ref_all), z_ref_all, 'r-',
                      label='Reference', linewidth=2, alpha=0.6)
        self.ax1.plot(x_ref_all, np.zeros_like(x_ref_all), z_terr_all, 'g--',
                      label='Terrain', linewidth=2, alpha=0.6)
        self.ax1.plot(yp_array[:, 6], yp_array[:, 7], yp_array[:, 8], 'b-',
                      label='Flight path', linewidth=1.5)
        self.ax1.scatter([x_current[6]], [x_current[7]], [x_current[8]],
                         c='blue', s=150, marker='o', label='Drone')

        self.ax1.set_xlabel('X [m]')
        self.ax1.set_ylabel('Y [m]')
        self.ax1.set_zlabel('Z [m]')
        self.ax1.set_xlim([0, 50])
        self.ax1.set_ylim([-5, 5])
        self.ax1.set_zlim([0, 20])
        self.ax1.legend()

        self.ax2.clear()
        self.ax2.plot(x_ref_all, z_ref_all, 'r-', label='Reference', linewidth=2, alpha=0.7)
        self.ax2.plot(x_ref_all, z_terr_all, 'g--', label='Terrain', linewidth=2, alpha=0.7)
        if len(yp_array) > 0:
            self.ax2.plot(yp_array[:, 6], yp_array[:, 8], 'b-', label='Flight', linewidth=1.5)
            self.ax2.scatter([x_current[6]], [x_current[8]], c='blue', s=150, marker='o')

        self.ax2.set_xlabel('X [m]')
        self.ax2.set_ylabel('Z [m]')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend()

        plt.draw()
        plt.pause(0.001)