#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

def visualization():
    # Load the trajectory data from CSV
    trajectory = np.loadtxt("trajectory.csv", delimiter=',')

    # Create a plot
    _, ax = plt.subplots(1)
    ax.set_aspect('equal')  # Keep aspect ratio square

    # Plot the trajectory
    ax.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2, label='Robot Trajectory')

    # Plot the desired square path for reference
    square_x = [4, 4, 0, 0, 4]
    square_y = [0, 4, 4, 0, 0]
    ax.plot(square_x, square_y, 'r--', label='Desired Path')

    # Set plot limits
    plt.xlim(-1, 5)
    plt.ylim(-1, 5)

    # Add labels, title, and legend
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Turtlebot Trajectory Visualization")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    visualization()

