#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, PoseStamped
import cPickle
from collections import defaultdict
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState, Image, CompressedImage
import time
import os
import datetime
import pyrealsense2 as rs
import rospkg

RECORDING = False

ALL_DATA = defaultdict(list)


def stop_recording(*_, **__):
    global RECORDING
    global ALL_DATA
    RECORDING = False

    # only do this if you want to see camera data within RVIZ rather than the RS viewer
    # pipeline.stop()                 # stop recording RS_camera

    # Create file path
    formatted_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name = 'grasper_{}.pickle'.format(formatted_ts)
    path = os.path.join(root, file_name)

    # Save file
    with open(path, 'wb') as fh:
        cPickle.dump(ALL_DATA, fh)
        rospy.loginfo('Saved file to {}'.format(path))

    # Resets the dictionary - maybe you don't want this?
    ALL_DATA = defaultdict(list)

    return []


def start_recording( *_, **__):
    global RECORDING
    RECORDING = True

    # only do this if you want to see camera data within RVIZ rather than the RS viewer
    # pipeline.start(config)              # start recording RS camera feed
    return []


def add_subscriber(topic, msg_type):
    rospy.Subscriber(topic, msg_type, create_subscriber_handler(topic), queue_size=1)


def create_subscriber_handler(name):
    def handler(msg):
        global RECORDING
        global ALL_DATA

        if RECORDING:
            ALL_DATA[name].append(msg)

    return handler


if __name__ == '__main__':
    rospy.init_node('record_data')

    rospack = rospkg.RosPack()
    root = os.path.join(rospack.get_path('apple_grasper'), 'data')

    a = rospy.Service('start_recording', Empty, start_recording)
    b = rospy.Service('stop_recording', Empty, stop_recording)

    #Configure the RealSense Camera recording data
    # Configure depth and color streams
    # pipeline = rs.pipeline()  # for recording rs-camera data
    # config = rs.config()
    # config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    #
    # # Create file path
    # formatted_ts = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    # file_name = 'catkin_ws/src/simple-grasper/Data rs_{}.bag'.format(formatted_ts)
    # root = os.path.expanduser('~')
    # path = os.path.join(root, file_name)
    #
    # config.enable_record_to_file(path)

    # Add your interesting topics here
    # TODO: You may consider throttling these topics (i.e. creating a throttled version of the node and subscribing to the throttled node)
    add_subscriber('/wrench', WrenchStamped)                 # 3 linear force and 3 torques
    add_subscriber('/camera/color/image_raw/compressed', CompressedImage)
    add_subscriber('/manipulator_pose', PoseStamped)
    # add_subscriber('rpy_data', Vector3Stamped)

    rospy.spin()
