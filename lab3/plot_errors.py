# # import matplotlib.pyplot as plt
# # from utilities import FileReader




# # def plot_errors(filename):
    
# #     headers, values=FileReader(filename).read_file()
    
# #     time_list=[]
    
# #     first_stamp=values[0][-1]
    
# #     for val in values:
# #         time_list.append(val[-1] - first_stamp)

    
    
# #     fig, axes = plt.subplots(2,1, figsize=(14,6))


# #     axes[0].plot([lin[len(headers) - 5] for lin in values], [lin[len(headers) - 4] for lin in values])
# #     axes[0].set_title("state space")
# #     axes[0].grid()

    
# #     axes[1].set_title("each individual state")
# #     for i in range(0, len(headers) - 1):
# #         axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

# #     axes[1].legend()
# #     axes[1].grid()

# #     output_filename = filename.replace('.csv', 'q=0.25_r=0.5_spiral.png')
# #     plt.savefig(output_filename, dpi=300, bbox_inches='tight')
# #     print(f"Plot saved successfully as {output_filename}")
# #     plt.close()
# #     #plt.show()
    
    





# # import argparse

# # if __name__=="__main__":

# #     parser = argparse.ArgumentParser(description='Process some files.')
# #     parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
# #     args = parser.parse_args()
    
# #     print("plotting the files", args.files)

# #     filenames=args.files
# #     for filename in filenames:
# #         plot_errors(filename)


# import matplotlib.pyplot as plt
# from utilities import FileReader

# def plot_errors(filename):
#     headers, values = FileReader(filename).read_file()
    
#     time_list = []
#     first_stamp = values[0][-1]
    
#     for val in values:
#         time_list.append(val[-1] - first_stamp)

#     # Create 3 stacked subplots instead of 2
#     fig, axes = plt.subplots(3, 1, figsize=(14, 10))

#     # 1. State Space Plot (X vs Y)
#     # kf_x is at len(headers) - 5, kf_y is at len(headers) - 4
#     axes[0].plot([lin[len(headers) - 7] for lin in values], [lin[len(headers) - 6] for lin in values])
#     axes[0].set_title("State Space (KF X vs Y)")
#     axes[0].grid()

#     # 2. All Individual States Over Time
#     axes[1].set_title("All Individual States")
#     for i in range(0, len(headers) - 1):
#         axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i])
#     axes[1].legend(loc='upper right', bbox_to_anchor=(1.15, 1))
#     axes[1].grid()

#     # 3. Dedicated Velocity Comparison Plot (kf_v, kf_w, odom_v, odom_w)
#     # Indices based on your header list:
#     # index 4: kf_v, index 5: kf_w, index 8: odom_v, index 9: odom_w
#     axes[2].set_title("Velocity Comparison: Kalman Filter vs. Odometry Measurements")
#     axes[2].plot(time_list, [lin[4] for lin in values], label="kf_v (Filter Linear)")
#     axes[2].plot(time_list, [lin[5] for lin in values], label="kf_w (Filter Angular)")
#     axes[2].plot(time_list, [lin[8] for lin in values], label="odom_v (Raw Linear)", linestyle='--')
#     axes[2].plot(time_list, [lin[9] for lin in values], label="odom_w (Raw Angular)", linestyle='--')
#     axes[2].set_xlabel("Time (seconds)")
#     axes[2].set_ylabel("Velocity / Angular Rate")
#     axes[2].legend()
#     axes[2].grid()

#     plt.tight_layout()
#     output_filename = filename.replace('.csv', '_q=0.25_r=0.5_velocities_kidnapping.png')
#     plt.savefig(output_filename, dpi=300, bbox_inches='tight')
#     print(f"Plot saved successfully as {output_filename}")
#     plt.close()

# import argparse

# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description='Process some files.')
#     parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
#     args = parser.parse_args()
#     print("plotting the files", args.files)

#     for filename in args.files:
#         plot_errors(filename)

import matplotlib.pyplot as plt
from utilities import FileReader

def plot_errors(filename):
    headers, values = FileReader(filename).read_file()
    
    time_list = []
    first_stamp = values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    # Create 3 stacked subplots
    fig, axes = plt.subplots(3, 1, figsize=(14, 11))

    # 1. State Space Plot: Compare Kalman Filter Trajectory vs. Raw Odometry Trajectory
    # kf_x (index 6), kf_y (index 7)
    # odom_x (index 8), odom_y (index 9)
    axes[0].plot([lin[6] for lin in values], [lin[7] for lin in values], label="KF Trajectory (kf_x vs kf_y)", color='blue')
    axes[0].plot([lin[8] for lin in values], [lin[9] for lin in values], label="Raw Odom Trajectory (odom_x vs odom_y)", color='orange', linestyle='--')
    axes[0].set_title("State Space Trajectory Comparison: Kalman Filter vs. Raw Odometry")
    axes[0].set_xlabel("X Position (m)")
    axes[0].set_ylabel("Y Position (m)")
    axes[0].legend()
    axes[0].grid()

    # 2. All Individual States Over Time
    axes[1].set_title("All Individual States Over Time")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i])
    axes[1].legend(loc='upper right', bbox_to_anchor=(1.18, 1))
    axes[1].grid()

    # 3. Dedicated Velocity Comparison Plot (kf_v, kf_w, odom_v, odom_w)
    # Updated indices based on your new header order:
    # index 4: kf_v, index 5: kf_w, index 10: odom_v, index 11: odom_w
    axes[2].set_title("Velocity Comparison: Kalman Filter vs. Odometry Measurements")
    axes[2].plot(time_list, [lin[4] for lin in values], label="kf_v (Filter Linear)")
    axes[2].plot(time_list, [lin[5] for lin in values], label="kf_w (Filter Angular)")
    axes[2].plot(time_list, [lin[10] for lin in values], label="odom_v (Raw Linear)", linestyle='--')
    axes[2].plot(time_list, [lin[11] for lin in values], label="odom_w (Raw Angular)", linestyle='--')
    axes[2].set_xlabel("Time (seconds)")
    axes[2].set_ylabel("Velocity / Angular Rate")
    axes[2].legend()
    axes[2].grid()

    plt.tight_layout()
    output_filename = filename.replace('.csv', '_q=0.25_r=0.5_trajectory_comparison.png')
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f"Plot saved successfully as {output_filename}")
    plt.close()

import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    print("plotting the files", args.files)

    for filename in args.files:
        plot_errors(filename)