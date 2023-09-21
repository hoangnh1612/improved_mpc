import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Plotting:
    def __init__(self, name, xlim=[-6, 6], ylim=[-6, 6], zlim=[0, 5], is_grid=True):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')  # Use add_subplot to create a 3D axes
        self.ax.set_title(name)
        self.ax.grid(is_grid)
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_zlim(zlim)
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')
        self.ax.set_zlabel('z [m]')

    def plot_path(self, path, label=None):
        path = np.array(path)
        self.ax.plot(path[:, 0], path[:, 1], -path[:, 2], label=label)  # Add the 'label' parameter here

    def show(self):
        # Add a legend with the labels you want
        self.ax.legend()
        plt.show()
