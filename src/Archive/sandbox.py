#!/usr/bin/env python

import pyrealsense2 as rs
import os
import datetime
import rospy

if __name__ == '__main__':
    rospy.init_node('sandbox')
    #Configure the RealSense Camera recording data
    # Configure depth and color streams
    pipeline = rs.pipeline()  # for recording rs-camera data
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Create file path
    formatted_ts = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    file_name = 'rs_{}.bag'.format(formatted_ts)
    root = os.path.expanduser('~')
    path = os.path.join(root, file_name)

    config.enable_record_to_file('object_detection.bag')          # 'object_detection.bag'

    pipeline.start(config)

    rospy.sleep(180)

    pipeline.stop()

