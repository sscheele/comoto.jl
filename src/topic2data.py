#!/usr/bin/env python

# For simple data rostopic echo -b file.bag -p /topic > data.csv
# http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

import os
import sys
import csv
import rospy
import rospkg
import argparse

import cv2
from cv_bridge import CvBridge
import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image

def save2csv(tag, skeleton_trajectory, dump_folder):
    with open('{}/{}_skeletal.csv'.format(dump_folder, tag), 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter='\t')
        for t in skeleton_trajectory:
            sklen = len(t)
            if sklen >0:
                writer.writerow([(sklen*'{}, ').format(*t.ravel())])
            else:
                writer.writerow(['NONE'])


class Rostopic2Data:

    def __init__(self, image_topic, skeleton_topic, user_id, mode, dump_folder=''):
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')

        self.bridge = CvBridge()
        tag = 'User-{}_mode-{}'.format(user_id, mode)
        self.video_writer = cv2.VideoWriter('{}/{}_video.avi'.format(dump_folder, tag), 
                                            fourcc=fourcc, fps=5, frameSize=(640,480))
        self.skeleton_trajectory = []

        self.image_subscriber = Subscriber(image_topic, Image)
        self.skeleton_subscriber = Subscriber(skeleton_topic, MarkerArray)

        sync = ApproximateTimeSynchronizer([self.skeleton_subscriber, self.image_subscriber], queue_size=10, slop=10000000, allow_headerless=True)
        sync.registerCallback(self.callback)

        try:
            print('Collecting data')
            rospy.spin()
        except KeyboardInterrupt:
            pass

        self.skeleton_trajectory = np.array(self.skeleton_trajectory)
        save2csv(tag, self.skeleton_trajectory, dump_folder)
        self.video_writer.release()

        print("Shutting down")

    def callback(self, skeleton_msg, rgb_msg):
        skeleton_joints = np.array([np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]) for marker in skeleton_msg.markers  if marker.pose.position.x < 1])
        #if len(skeleton_joints) == 32:
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        rgb_img = cv2.resize(rgb_img, (640, 480))

        self.skeleton_trajectory.append(skeleton_joints)
        self.video_writer.write(rgb_img)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-user_id')
    parser.add_argument('-mode')
    args = parser.parse_args(sys.argv[1:5])

    rospy.init_node("topic2data", anonymous=True)
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('comanip_htn')

    logdir_path = os.path.join(package_path, 'expt_logs/user_{}'.format(args.user_id))
    if not os.path.exists(logdir_path):
        os.mkdir(logdir_path)
        print('Created directory for User {}'.format(args.user_id))

    image_topic = '/front/rgb/image_raw'
    skeleton_topic = '/front/body_tracking_data'

    Rostopic2Data(image_topic=image_topic, skeleton_topic=skeleton_topic, user_id=args.user_id, 
                  mode=args.mode, dump_folder=logdir_path)

if __name__ == '__main__':
    main()