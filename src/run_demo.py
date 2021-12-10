from ros_dispatch import parse_traj_file, follow_trajectory, start_ros_node, end_ros_node 
import random
import tkinter as tk
import datetime
import rospy
import tf
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
import json
import tkinter as tk
from pygame import mixer
import time

mixer.init()

def play_music():
    time.sleep(3)
    mixer.music.load("beep.mp3")
    mixer.music.play()


map_type_to_traj = {
    "WHITE_1_A_D_62":"traj_1_A_D_62.txt",
    "PINK_2_A_B":"traj_2_A_B.txt",
    "PINK_2_D_E":"traj_2_D_E.txt",
    "PINK_2_E_B":"traj_2_E_B.txt",
    "PINK_2_E_C":"traj_2_E_C.txt",
    "BLACK_3_A_C":"traj_3_A_C_52.txt",
    "BLACK_3_C_B":"traj_3_C_B.txt",
    "BLACK_3_D_B":"traj_3_D_B.txt",
}

start_traj = ["WHITE_1_A_D_62", "PINK_2_E_C"]
end_traj = ["PINK_2_D_E", "BLACK_3_A_C"]
middle_traj = ["PINK_2_A_B","BLACK_3_D_B", "BLACK_3_C_B", "PINK_2_E_B"]

methods =  ["COMOTO", "NOMINAL"]
calculator_problems = ["2498 + 1984", "4298 - 1233", "3224 + 6304", "1213 / 239", "9280 - 3244", "2389 x 1438", "3429 - 1222", "9234 + 1221"]

random.shuffle(methods)
random.shuffle(calculator_problems)
random.shuffle(start_traj)
random.shuffle(end_traj)
random.shuffle(middle_traj)

start_ros_node()

window = tk.Tk()
width= window.winfo_screenwidth() 
height= window.winfo_screenheight()
window.geometry("%dx%d" % (width, height))
window.configure(bg='#8f918e')
frame = tk.Frame(window, bg="#8f918e", width = width, height = height)
my_string_var = tk.StringVar() 
my_string_var.set("Please read the instructions before starting the experiment")
text_label = tk.Label(frame, bg="#8f918e", fg = "black", textvariable=my_string_var, font=("Arial", 40))
second_string_var = tk.StringVar()
second_string_var.set("")
calculate_label = tk.Label(frame, bg="#8f918e", fg = "black", textvariable=second_string_var, font=("Arial", 40))
frame.pack()
text_label.place(x = 300, y = 500)
calculate_label.place(x = 600, y = 600)

counter = 0

types = [start_traj[0], end_traj[0], middle_traj[0], middle_traj[1]]

for method in methods:

    for i in range(len(types)):
        traj_file_name = map_type_to_traj[types[i]]

        if method == "NOMINAL":
            traj_file_name = traj_file_name[:5] + "nominal_" + traj_file_name[5:]

        type_name = types[i]

        traj, dt, t_0 = parse_traj_file(traj_file_name)

        print(traj_file_name)

        raw_input("Display Start Instructions?")

        my_string_var.set("Be careful as the arm moves to the start position")
        second_string_var.set("")
        
        frame.configure(bg='#8f918e')
        text_label.configure(bg = "#8f918e", fg = "black")
        calculate_label.configure(bg = "#8f918e", fg = "black")


        raw_input("Go to start?")

        follow_trajectory([traj[0]], 4.0,  4.0)

        raw_input("Display move instructions?")

        if type_name.startswith("WHITE"):
            frame.configure(bg='white')
            my_string_var.set("Reach for the WHITE calculator (Number 1) after the beep")
            text_label.configure(bg = "white")
            calculate_label.configure(bg = "white")
        

        if type_name.startswith("PINK"):
            frame.configure(bg='pink')
            my_string_var.set("Reach for the PINK calculator (Number 2) after the beep")
            text_label.configure(bg = "pink", fg = "black")
            calculate_label.configure(bg = "pink", fg = "black")
        
        if type_name.startswith("BLACK"):
            frame.configure(bg='black')
            my_string_var.set("Reach for the BLACK calculator (Number 3) after the beep")
            text_label.configure(bg = "black", fg = "white")
            calculate_label.configure(bg = "black", fg = "white")
        
        second_string_var.set("Calculate: " + calculator_problems[counter])

        counter += 1

        if counter > 7:
            counter = 0

        raw_input("Start countdown?")

        play_music()

        time.sleep(0.5)

        follow_trajectory(traj, dt, 1.0)


end_ros_node()