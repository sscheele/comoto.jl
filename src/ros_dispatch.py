import actionlib
import rospy
import sys

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from exec_human_traj import vis_traj_pts

joint_names = ["iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_4", 
    "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"]

rospy.init_node("comoto_dispatch")

client = actionlib.SimpleActionClient("iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
client.wait_for_server()

def follow_trajectory(points, dt, t_0):
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

def line2float(line):
    tmp = line.split(", ")
    tmp = [float(s) for s in tmp]
    return tmp

def parse_traj_file(fname):
    stuff = None
    with open(fname, 'r') as f:
        stuff = f.readlines()
    traj = stuff[:-1]
    info = line2float(stuff[-1])
    traj = map(line2float, traj)
    dt, t_0 = info[0], info[1]
    return traj, dt, t_0

if __name__ == "__main__":
    traj, dt, t_0 = parse_traj_file(sys.argv[1])
    follow_trajectory(traj, dt, t_0)
    rospy.signal_shutdown("Sending trajectory complete")