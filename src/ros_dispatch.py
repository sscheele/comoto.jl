import actionlib
import rospy
import sys

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


client = None
joint_names = None

def follow_trajectory(points, dt, t_0):
    global client
    global joint_names
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    for t, point in enumerate(points):
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[-1].positions = point
        trajectory.points[-1].velocities = [0.0 for _ in point]
        trajectory.points[-1].accelerations = [0.0 for _ in point]
        trajectory.points[-1].time_from_start = rospy.Duration((t*dt)+t_0)
    follow_goal = FollowJointTrajectoryGoal()
    follow_goal.trajectory = trajectory
    print("sent, waiting...")
    client.send_goal(follow_goal)
    client.wait_for_result()
    print("Done...")

def parse_traj_line(line):
    tmp = line.split(", ")
    tmp = [float(s) for s in tmp]
    return tmp

def parse_traj_file(fname):
    stuff = None
    with open(fname, 'r') as f:
        stuff = f.readlines()
    traj = stuff[:-1]
    info = parse_traj_line(stuff[-1])
    traj = map(parse_traj_line, traj)
    dt, t_0 = info[0], info[1]
    return traj, dt, t_0

def start_ros_node():
    global client
    global joint_names
    rospy.init_node("comoto_dispatch")
    joint_names = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", 
        "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"]
    client = actionlib.SimpleActionClient("/jaco_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server()


def end_ros_node():
    rospy.signal_shutdown("Sending trajectory complete")
    
if __name__ == "__main__":
    rospy.init_node("comoto_dispatch")
    traj, dt, t_0 = parse_traj_file(sys.argv[1])
    robot_name = sys.argv[2]
    print(robot_name)
    if robot_name == "JACO":
        joint_names = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", 
            "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"]
        client = actionlib.SimpleActionClient("/jaco_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    else:
        client = actionlib.SimpleActionClient("iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", 
            "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]
    client.wait_for_server()
    follow_trajectory(traj, dt, t_0)
    rospy.signal_shutdown("Sending trajectory complete")