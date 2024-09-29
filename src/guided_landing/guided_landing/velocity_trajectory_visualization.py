import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_csv_file(file_path):
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header
        data = np.array(list(reader), dtype=float)
    return data

def plot_3d_trajectory(data, title):
    times = data[:, 0]
    fb_vel = data[:, 1]
    ud_vel = data[:, 2]
    y_vel = data[:, 3]

    # Calculate positions by integrating velocities
    dt = np.diff(times, prepend=0)
    x_pos = np.cumsum(fb_vel * np.cos(np.cumsum(y_vel * dt)) * dt)
    y_pos = np.cumsum(fb_vel * np.sin(np.cumsum(y_vel * dt)) * dt)
    z_pos = np.cumsum(ud_vel * dt)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x_pos, y_pos, z_pos)

    ax.set_xlabel('X distance (m)')
    ax.set_ylabel('Y distance (m)')
    ax.set_zlabel('Z distance (m)')
    ax.set_title(f'3D Trajectory - {title}')

    plt.show()

def main():
    directory = 'trajectory_data'
    
    if not os.path.exists(directory):
        print(f"Directory '{directory}' does not exist.")
        return

    for filename in os.listdir(directory):
        if filename.endswith('.csv'):
            file_path = os.path.join(directory, filename)
            data = read_csv_file(file_path)
            plot_3d_trajectory(data, filename[:-4])  # Use filename without .csv as title

if __name__ == '__main__':
    main()
