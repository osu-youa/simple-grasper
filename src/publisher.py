#!/usr/bin/env python
# This script controls and executes my publishing. Will make a node and will be
# consistently publishing out throughout the whole time

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from tf2_ros import TransformBroadcaster, TransformStamped, Buffer, TransformListener
from tf import transformations
import message_filters
from geometry_msgs.msg import Vector3Stamped
from math import radians
import pickle


class MyPublisher:
    def __init__(self):
        # Just set up the empty arrays to publish
        # publish: the stored data collected by path tracker, tare_tf, actual_tf, estimated_tf, raw_IMU_data
        self.index = 0                          # used to index the published data

        self.tf_buffer = Buffer()
        tf_listener = TransformListener(self.tf_buffer)
        self.base_link = rospy.get_param('base_frame')  # this will always be our parent frame of reference

        self.tare_tf = TransformStamped()           # intial (base->ee)+(ee->IMU)
        self.tare_tf.header.frame_id = self.base_link
        self.tare_tf.child_frame_id = "tare_tf"
        self.tare_tf.transform.rotation.w = 1

        self.Apple_nom_tf = TransformStamped()                  # static transform between base and apple found via the   (base->ee)+(ee->IMU)
        self.Apple_nom_tf.header.frame_id = self.base_link
        self.Apple_nom_tf.child_frame_id = "apple_nom_tf"
        self.Apple_nom_tf.transform.rotation.w = 1

        # self.actual_tf = TransformStamped()  # running time (base->ee)+(ee->IMU)
        # self.actual_tf.header.frame_id = self.base_link
        # self.actual_tf.child_frame_id = "actual_tf"
        #
        # self.estimated_tf = TransformStamped()  # What the arduino sends back as it's guess to its location
        # self.estimated_tf.header.frame_id = "IMU_tare"
        # self.estimated_tf.child_frame_id = "estimated_tf"
        # self.estimated_tf.transform.rotation.x = 0
        # self.estimated_tf.transform.rotation.y = 0
        # self.estimated_tf.transform.rotation.z = 0
        # self.estimated_tf.transform.rotation.w = 1

    def publish_tf(self):
        """  publish all that shiiiiiiiiittttt  """
        i = 0
        tf_list = [self.tare_tf, self.Apple_nom_tf]
        for transform in tf_list:           # iterate through the list of transforms that we need to boadcast
            # print(transform)
            br = TransformBroadcaster()
            transform.child_frame_id = child_id[i]
            br.sendTransform(transform)                     # broadcast it out
            broadcast[child_id[i]].publish(transform)
            i += 1

    def update_tared(self, request):
        """ This will update the tare_tf to what the point actually is at the tared positon once
        :return: Nada """
        print("Updating the tared transfrom")
        stamp = rospy.Time.now()
        self.tf_buffer.can_transform(self.base_link, "test_frame", stamp, rospy.Duration(0.5))
        self.tare_tf = self.tf_buffer.lookup_transform(self.base_link, "test_frame", stamp)
        return TriggerResponse(success = True, message = "merhhhh")

    def set_apple_nom(self, request):
        """ This will update the tare_tf to what the point actually is at the tared positon once
        :return: Nada """
        print("Updating the apple_nom")
        stamp = rospy.Time.now()
        self.tf_buffer.can_transform('base_link', "ee_link", stamp, rospy.Duration(0.5))
        self.Apple_nom_tf = self.tf_buffer.lookup_transform('base_link', "ee_link", stamp)
        return TriggerResponse(success = True, message = "merhhhh")

    def update_estimate(self, rpy_msg):  #, act_msg):
        """  gathers RPY from the IMU and calculates the expected orientation
        :param rpy_msg data comes in in degrees; need to translate into radians
        to get the correct orientation the rotations must be applied in the correct order which for this
        configuration is yaw, pitch, and then roll
        """
        self.estimated_tf.header.stamp = rospy.Time.now()

        # for the location we will place it right at the same point the IMU is at since we are not watching the
        # location but orientation
        self.estimated_tf.transform.translation.x = 0   #act_msg.transform.translation.x
        self.estimated_tf.transform.translation.y = 0   # act_msg.transform.translation.y
        self.estimated_tf.transform.translation.z = 0   # act_msg.transform.translation.z

        roll = radians(rpy_msg.vector.x)            # angle between sensor y-axis and Earth ground plane, y-axis up is
                                                        # positive roll
        pitch = radians(rpy_msg.vector.y)           # angle between sensor x-axis and Earth ground plane, toward the
                                                        # Earth is positive, up toward the sky is negative
        yaw = radians(rpy_msg.vector.z)             # Yaw is the angle between Sensor x-axis and Earth magnetic North;
                                                        # looking down on the sensor positive yaw is counterclockwise
        q = transformations.quaternion_from_euler(roll, pitch, yaw)     # orient = [x, y, z, w]ros
        self.estimated_tf.transform.rotation.x = q[0]
        self.estimated_tf.transform.rotation.y = q[1]
        self.estimated_tf.transform.rotation.z = q[2]
        self.estimated_tf.transform.rotation.w = q[3]


if __name__ == '__main__':
    rospy.init_node('publisher')
    my_pub = MyPublisher()
    rate = rospy.Rate(10)

    child_id = rospy.get_param('tf2pub').strip().split(' ')

    # Services
    tare_service = rospy.Service('/update_tare', Trigger, my_pub.update_tared)
    nominal_service = rospy.Service('/set_apple_nom', Trigger, my_pub.set_apple_nom)

    broadcast = {frame: rospy.Publisher('my_{}'.format(frame), TransformStamped, queue_size=1) for frame in child_id}
    # estimated_sub = message_filters.Subscriber('/rpy_data', Vector3Stamped)
    # actual_sub = message_filters.Subscriber('/IMU_custom_tf', static_transform_publisher)
    rospy.sleep(2.0)
    topic = 0

    while not rospy.is_shutdown():
        # while not topic:                # wait till the IMU has been intialized/ wait for the rpy_data topic to exist
        #     topic = rospy.wait_for_message('/rpy_data', Vector3Stamped)
        #     rospy.sleep(0.5)
        # estimated_sub.registerCallback(my_pub.update_estimate)
        my_pub.publish_tf()
    rospy.spin()
