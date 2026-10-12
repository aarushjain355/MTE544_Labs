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

        plt.plot(x_coords, y_coords)
        plt.title(f"Laserscan Cartesian Data for {filename}")
        plt.xlabel("X-axis")
        plt.ylabel("Y-label")
    
    else :
        for val in values:
            time_list.append(val[-1] - first_stamp)

        for i in range(0, len(headers) - 1):
            plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

        plt.legend()
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
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