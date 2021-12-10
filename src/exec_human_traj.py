from human_traj_display.srv import ExecuteHumanTraj
from geometry_msgs.msg import Point

import csv
import sys
import rospy

rospy.init_node("human_traj_dispatch")

def parse_human_traj_file(fname):
    tsteps = []
    with open(fname, 'r') as f:
        reader = csv.reader(f)
        header = reader.next() #
        for row in reader:
            frow = [float(x) for x in row]
            pointarr = [Point(frow[i], frow[i+1], frow[i+2]) for i in range(0, len(frow), 3)]
            tsteps.append(pointarr)
    return tsteps

def pointsub(p1, p2):
    return Point(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z)

def in_frame(arr, a, fr):
    arr[a] = [pointsub(arr[a][i], arr[fr][i]) for i in range(len(arr[a]))]

def transform_jointarr(arr):
    # shoulder, elbow, wrist, palm, neck, head, torso
    # 7: shoulder, elbow...
    # need to transform tree from leaves to root
    in_frame(arr, 3, 2) # R palm to wrist frame
    in_frame(arr, 2, 1) # R wrist to elbow frame
    in_frame(arr, 1, 0) # R elbow to shoulder frame

    in_frame(arr, 10, 9) # L palm to wrist
    in_frame(arr, 9, 8) # L wrist to elbow
    in_frame(arr, 8, 7) # L elbow to shoulder
    in_frame(arr, 7, 4) # L shoulder to neck

    in_frame(arr, 5, 4) # head to neck 
    in_frame(arr, 6, 4) # torso to neck 
    in_frame(arr, 4, 0) # neck to R shoulder

def visualize_human_trajectory(fname, human_timestep_size):
    arr_by_timestep = parse_human_traj_file(fname)
    n_human_timesteps = len(arr_by_timestep)

    n_human_joints = len(arr_by_timestep[0])
    arr_T = [[x[i] for x in arr_by_timestep] for i in range(n_human_joints)]

    transform_jointarr(arr_T)

    rospy.wait_for_service("execute_human_traj")
    try:
        execute_human_traj_service = rospy.ServiceProxy("execute_human_traj", ExecuteHumanTraj) 
        resp = execute_human_traj_service(arr_T[0], arr_T[1], arr_T[2], arr_T[3], arr_T[4], arr_T[5], arr_T[6], arr_T[7], arr_T[8], arr_T[9], arr_T[10], human_timestep_size, n_human_timesteps)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

dt = float(sys.argv[2])
print(dt)
visualize_human_trajectory(sys.argv[1], dt)
rospy.signal_shutdown("Sending human trajectory complete")