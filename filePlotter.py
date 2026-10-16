# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader
import math
import numpy as np

def plot_errors(filename):
    
    headers, values = FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]

    # Special stuff for laserscan
    if "laser" in filename:
        ranges, angle_increment, stamp = values[0]

        x_coords = []
        y_coords = []
        for i in range(len(ranges)):
            if not np.isfinite(ranges[i]): continue
            x_coords.append(ranges[i] * math.cos(angle_increment*i))
            y_coords.append(ranges[i] * math.sin(angle_increment*i))

        plt.plot(x_coords, y_coords, label="ranges")
        plt.legend()
        plt.title(f"Laserscan of {filename} at t=0")
        plt.xlabel("X-axis [m]")
        plt.ylabel("Y-axis [m]")
    # Odomoetry and IMU
    else:
        if "odom" in filename:
            # x- vs -y plot
            plt.figure(1)

            plt.plot([lin[0] for lin in values], [lin[1] for lin in values], label= "position")

            plt.legend()
            plt.title(f"Top View for {filename}")
            plt.xlabel("X-axis [m]")
            plt.ylabel("Y-axis [m]")
            plt.grid()

        y1_label, y2_label, plt_title = ["", "", ""]
        if "odom" in filename:
            y1_label = "Linear Displacement [m]"
            y2_label = "Angular Displacement [rad]"
            plt_title = f"Odometry Displacements of {filename}"
        elif "imu" in filename:
            y1_label = "Linear Acceleration [m/s^2]"
            y2_label = "Angular Velocity [rad/s]"
            plt_title = f"IMU Kinematics of {filename}"
        
        # x, y, th- vs -t plot
        _, ax1 = plt.subplots()

        # Gather the timestamps (nanoseconds)
        for val in values:
            time_list.append(val[-1] - first_stamp)

        lines = [0, 0, 0]
        # Set up y-axis for linear displacement (x and y)
        ax1.set_xlabel("Time [ns]")
        ax1.set_ylabel(y1_label)
        # Plot x vs t
        lines[0] = ax1.plot(time_list, [lin[0] for lin in values], label= headers[0]+ " linear")
        # Plot y vs t
        lines[1] = ax1.plot(time_list, [lin[1] for lin in values], label= headers[1]+ " linear")
        ax1.grid(True)

        # For layering reasons, the grid and line for th need to be on seperate axes
        # Set up the y-axis and grid for angular displacement (th)
        ax2 = ax1.twinx(delta_zorder=-1) # Need the grid to be below x and y lines
        ax2.set_ylabel(y2_label, color="g")
        # Plot an invisible copy of th vs t
        ax2.plot(time_list, [lin[2] for lin in values], visible=False)
        #lines[2] = ax2.plot(time_list, [lin[2] for lin in values], label= headers[2]+ " linear", color="g")
        ax2.tick_params(axis='y', labelcolor="g")
        ax2.grid(True, axis="y", color="g", linestyle="-.", linewidth="0.5")

        # Set up the actual th vs t
        ax3 = ax1.twinx()
        # Plot th vs t
        lines[2] = ax3.plot(time_list, [lin[2] for lin in values], label= headers[2]+ " linear", color="g")
        ax3.grid() # Remove the grid

        # Set up the legend for both axes
        l_lines = lines[0] + lines[1] + lines[2]
        labels = [line.get_label() for line in l_lines]
        ax3.legend(l_lines, labels, loc=0)

        plt.title(plt_title)
    
    plt.grid()
    plt.show()
    
import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)