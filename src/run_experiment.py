from ros_dispatch import parse_traj_file, follow_trajectory, start_ros_node, end_ros_node 
import random
import tkinter as tk
import datetime
import rospy
import tf
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
import json


marker_array = []
campos = [-0.019, 0.420, 1.112]
transform = tf.transformations.euler_matrix(-2.161, 0.046, -3.110)[:3, :3]

def callback(data):
    joints = [2, 3, 5, 6, 7, 8, 12, 13, 14, 15, 26]
    markers = data.markers
    curr_frame = []
    for index in joints:
        old_coords = np.array([markers[index].pose.position.x, markers[index].pose.position.y, markers[index].pose.position.z])
        new_coords = np.matmul(transform, old_coords.T)
        curr_frame.append(new_coords[0] + campos[0])#1.4)#
        curr_frame.append(new_coords[1] + campos[1])#0.5)#
        curr_frame.append(new_coords[2] + campos[2])#-0.5)#
    marker_array.append(curr_frame)

map_type_to_traj = {
    "WHITE_COMOTO": "traj_white.txt",
    "WHITE_NOMINAL": "traj_nominal_white.txt",
    "GREEN_COMOTO": "traj_green.txt",
    "GREEN_NOMINAL": "traj_nominal_green.txt",
    "PINK_COMOTO": "traj_pink.txt",
    "PINK_NOMINAL": "traj_nominal_pink.txt",
    "BLACK_COMOTO": "traj_black.txt",
    "BLACK_NOMINAL": "traj_nominal_black.txt",

}

types = list(map_type_to_traj.keys())

print(types)

random.shuffle(types)

start_ros_node()

# window = tk.Tk()

# label = tk.Label(text="Click the button to run the experiment")

# next_button = tk.Button(text="Start Experiment")

# close_button = tk.Button(text="End Experiment")

# label.pack()

# next_button.pack()

# close_button.pack()

rospy.Subscriber("/front/body_tracking_data", MarkerArray, callback)
rate = rospy.Rate(30)

for i in range(len(types)):
    traj_file_name = map_type_to_traj[types[i]]

    type_name = types[i]

    traj, dt, t_0 = parse_traj_file(traj_file_name)

    print(traj_file_name)

    if type_name.startswith("WHITE"):
        print("Reach for White Calculator")
    
    if type_name.startswith("GREEN"):
        print("Reach for Green Calculator")
    

    if type_name.startswith("PINK"):
        print("Reach for Pink Calculator")
    
    if type_name.startswith("BLACK"):
        print("Reach for Black Calculator")

    raw_input("Go to start?")
    old_time = datetime.datetime.now()
    print(old_time)

    follow_trajectory([traj[0]], 4.0,  4.0)

    raw_input("Execute?")
    marker_array = []
    follow_trajectory(traj, dt, 1.0)
    new_time = datetime.datetime.now()
    print(new_time)
    print(old_time-new_time)

    with open("human_marker_"+type_name+"_.txt", "w") as txt_file:
        for line in marker_array:
            for point in line:
                txt_file.write(str(point) + ",") # works with any number of elements in a line
            txt_file.write("\n")
    marker_array = []


end_ros_node()