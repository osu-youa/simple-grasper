#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
import cPickle
from collections import defaultdict
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
import time
import os
import datetime

RECORDING = False

ALL_DATA = defaultdict(list)

def stop_recording(*_, **__):
    global RECORDING
    global ALL_DATA
    RECORDING = False

    # Create file path
    formatted_ts = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    file_name = 'grasper_{}.pickle'.format(formatted_ts)
    root = os.path.expanduser('~')
    path = os.path.join(root, file_name)

    # Save file
    with open(path, 'wb') as fh:
        cPickle.dump(ALL_DATA, fh)
        rospy.loginfo('Saved file to {}'.format(path))

    # Resets the dictionary - maybe you don't want this?
    ALL_DATA = defaultdict(list)

    return []

def start_recording(*_, **__):
    global RECORDING
    RECORDING = True
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
    rospy.Service('start_recording', Empty, start_recording)
    rospy.Service('stop_recording', Empty, stop_recording)

    # Add your interesting topics here
    # TODO: You may consider throttling these topics (i.e. creating a throttled version of the node and subscribing to the throttled node)
    add_subscriber('wrench', WrenchStamped)
    add_subscriber('joint_states', JointState)

    rospy.spin()
