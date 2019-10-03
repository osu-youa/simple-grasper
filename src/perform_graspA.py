#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from industrial_msgs.msg import RobotStatus
from geometry_msgs.msg import Pose, Point, Quaternion, TwistStamped, TransformStamped, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_slerp, euler_from_matrix, \
    quaternion_from_matrix, euler_matrix, quaternion_matrix
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Vector3
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from math import radians, ceil
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import sys
from std_srvs.srv import Empty
from rosserial_arduino.srv import Test
from std_srvs.srv import Trigger, TriggerRequest
import UR5Motion_functions as UR5

ERROR = 0

# Config - These define the motion to be completed. You can modify this, but if you want multiple options, you may
# want to modify the code to take it through a sys.argv
# (e.g. running it like "rosrun apple_grasper perform_grasp.py 0.05 0.20
retreat_dist = 0.05                 # How far back does the arm move? + means backwards
rotation = radians(20)              # How much should the end effector rotate? Currently assumes around Z axis
vel = 0.04                          # What's the speed the arm should move?
interp_N = 20                       # How many points should be interpolated for the trajectory?
sum_abs_radian_warn = radians(10)   # If any total movement exceeds this amount, issue a warning

# Frames
# THESE FRAMES SHOULD NOT BE CHANGED because the kinematics solver assumes these references
ur_base_link = 'base'
ur_end_link = 'tool0'

# THESE FRAMES CAN BE CHANGED as they are not bound to the UR5 program code
tool_frame = 'test_frame'                   # What is the frame you're trying to move?
                                            # IMPORTANT! THIS FRAME MUST BE STATIC w.r.t. ur_end_link
movement_frame = 'base_link'                # What frame is the movement/rotation vector defined in? base_link
movement_vector = [-retreat_dist, 0, 0]     # What's the linear movement you want in the movement frame?
movement_rpy = [0, 0, rotation]             # What's the rotation you want in the movement frame, centered around tool_frame's origin?

f_exit = False                              # Flag to signify when the user is done experimenting with different poses
#
# def update_error(msg):
#     global ERROR
#     ERROR = msg.in_error.val
#
#
# def get_tf(target, source, stamp = rospy.Time(0)):
#     tf_buffer.can_transform(target, source, stamp, rospy.Duration(0.5))
#     tf = tf_buffer.lookup_transform(target, source, stamp)
#     return tf
#
#
# def point_to_array(pt):
#     # Also works on vectors!
#     return np.array([pt.x, pt.y, pt.z])
#
#
# def quat_to_array(quat):
#     return np.array([quat.x, quat.y, quat.z, quat.w])
#
#
# def tare_force():
#     urscript_pub.publish('zero_ftsensor()\n')
#
#
# def issue_stop():
#     cmd = "stopl(50.0)\n"
#     urscript_pub.publish(cmd)
#
#
# def ros_pose_slerp(start, end, n):
#
#     frame = None
#     if isinstance(start, PoseStamped):
#         frame = start.header.frame_id
#         start = start.pose
#         end = end.pose
#
#     start_pos = point_to_array(start.position)
#     end_pos = point_to_array(end.position)
#
#     positions = np.linspace(start_pos, end_pos, n, endpoint=True)
#     quats = [quaternion_slerp(quat_to_array(start.orientation), quat_to_array(end.orientation), i) for i in np.linspace(0, 1, n, endpoint=True)]
#
#     poses = [Pose(Point(*pos), Quaternion(*quat)) for pos, quat in zip(positions, quats)]
#     if frame is not None:
#         header = Header()
#         header.frame_id = frame
#         poses_stamped = [PoseStamped(header, pose) for pose in poses]
#         return poses_stamped
#     return poses
#
#
# def ros_quat_to_euler(q):
#     return np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))
#
#
# def construct_joint_point(position, t):
#     point = JointTrajectoryPoint()
#     point.positions = position
#     if not isinstance(t, rospy.Duration):
#         t = rospy.Duration(t)
#     point.velocities = [0.0] * len(position)
#     point.accelerations = [0.0] * len(position)
#     point.time_from_start = t
#     return point
#
#
# def convert_mat_to_pose(mat):
#     quat = Quaternion(*quaternion_from_matrix(mat))
#     pos = Point(mat[0, 3], mat[1, 3], mat[2, 3])
#     return Pose(pos, quat)
#
#
# def convert_tf_to_pose(tf):
#     pose = PoseStamped()
#     pose.pose = Pose(Point(*point_to_array(tf.transform.translation)), tf.transform.rotation)
#     pose.header.frame_id = tf.header.frame_id
#     return pose
#
#
# def convert_pose_to_tf(pose_stamped, frame_id):
#     # frame_id is the ID of the frame which is located at the pose in the given frame
#     tf = TransformStamped()
#     tf.header.frame_id = frame_id
#     tf.child_frame_id = pose_stamped.header.frame_id
#     tf.transform.translation = Vector3(*point_to_array(pose_stamped.pose.position))
#     tf.transform.rotation = pose_stamped.pose.orientation
#     return tf
#
#
# def convert_poses_to_trajectory(poses, initial_joint_state, linear_vel, leeway = 2.0, warn_threshold = 0):
#
#     last_joints = initial_joint_state.position
#     last_pos = np.array(urkin.fwd_kin(last_joints)[:3,3]).T[0]
#
#     traj = JointTrajectory()
#     traj.header.stamp = rospy.Time.now()
#     traj.joint_names = initial_joint_state.name
#     traj.points.append(construct_joint_point(last_joints, 0.0))
#
#     # Kind of a hack, offset correction due to fwd kins not being exact
#
#     believed_pos = point_to_array(urkin.fwd_kin(last_joints, o_unit='p').position)
#     correction = believed_pos - point_to_array(poses[0].position)
#
#     joints = [last_joints]
#
#     t = 0.0
#     for pose in poses:
#         pose.position = Point(*point_to_array(pose.position) + correction)
#         new_pos = point_to_array(pose.position)
#         new_joints = urkin.inv_kin(pose, last_joints)
#         dt = max(np.linalg.norm(new_pos - last_pos) / linear_vel * leeway, 0.01)
#         t += dt
#         traj.points.append(construct_joint_point(new_joints, t))
#
#         last_joints = new_joints
#         last_pos = new_pos
#
#         joints.append(new_joints)
#
#     # if warn_threshold > 0:            # LMD found that this blocked many if not all positions, thus negated the check
#     #     joints = np.array(joints)
#     #     sum_abs_diff = np.abs(joints[:-1] - joints[1:]).sum(axis=1)
#     #     if np.any(sum_abs_diff > warn_threshold):
#     #         rospy.logwarn('Detected a large joint movement! Either near singularity or need to increase path resolution')
#     #         sys.exit(1)
#
#     goal = FollowJointTrajectoryGoal()
#     goal.trajectory = traj
#
#     return goal

#
# ####################
# # Move_arm functions
# ####################
# def generate_ur_pose(pose):
#     quat = pose.orientation
#     rx, ry, rz = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
#     x, y, z = pose.position.x, pose.position.y, pose.position.z
#     format_str = 'p[{}]'.format(','.join(['{:.3f}'] * 6))
#     return format_str.format(x, y, z, rx, ry, rz)
#
# def generate_move_command(pose, a, v, t=0):
#     ur_pose = generate_ur_pose(pose)
#     format_str = 'movej({}, a={:.3f}, v={:.3f}, t={:.3f}, r=0)'
#     return format_str.format(ur_pose, a, v, t)
#
# def wait_for_motion_to_complete():
#
#     # Kind of a hack, couldn't figure out a quick way to get the UR5 move status
#
#     rate = rospy.Rate(10)
#     epsilon = 0.001
#
#     rospy.sleep(0.5)
#
#     while True:
#         twist = rospy.wait_for_message('tool_velocity', TwistStamped)
#         lin = twist.twist.linear
#         ang = twist.twist.angular
#
#         lin_array = np.array([lin.x, lin.y, lin.z])
#         ang_array = np.array([ang.x, ang.y, ang.z])
#
#         if np.linalg.norm(lin_array) < epsilon and np.linalg.norm(ang_array) < epsilon:
#             break
#         rate.sleep()





####################
# Publishers
####################

def tare_force():
    urscript_pub.publish('zero_ftsensor()\n')

# def issue_linear_velocity_command(array):
#     array_str = format_array_decimal(array)
#     rospy.loginfo_throttle(0.025, array_str)
#     cmd = "speedl({},-5.0,0.05)\n".format(array_str)
#     urscript_pub.publish(cmd)

# def run_force_mode(pose, selection_vector, wrench, limits):
#     """
#     Refer to the URScript manual for the definitions of each of the arguments.
#     :param pose: A ROS pose defining a frame in which the forces are defined.
#     :param selection_vector: A Boolean array which specifies compliant axes.
#     :param wrench: A vector for the wrench to be applied for each axis.
#     :param limits: For compliant axes, a velocity limit. For non-compliant axes, an absolute position limit deviation.
#     :return:
#     """
#
#     ur_pose = generate_ur_pose(pose)
#     selection_vector = format_array(np.array(selection_vector).astype(bool).astype(int))
#     wrench = format_array_decimal(wrench)
#     limits = format_array_decimal(limits)
#
#     cmd = 'force_mode({}, {}, {}, 2, {})\nsync()\n'.format(ur_pose, selection_vector, wrench, limits)
#     urscript_pub.publish(cmd)

def issue_stop():
    cmd = "stopl(50.0)\n"
    urscript_pub.publish(cmd)


def Move2Tare():
    # move to the tare position

    tare_pose = Pose(Point(0.45, 0.3, 0.6), Quaternion(0, 0, 0, 1))
    cmd = UR5.generate_move_command(tare_pose, 1.0, 0.25)
    urscript_pub.publish(cmd)
    UR5.wait_for_motion_to_complete()

##################################
# Hand Functions
##################################
def Hand2NeutralPos():
    # Request the hand to go to the neutral open position
    rospy.wait_for_service('/open_hand')
    tare_service = rospy.ServiceProxy('/open_hand', Trigger)        # Create the connection to the service.
    trig_obj = TriggerRequest()                                     # Create an object of the type TriggerRequest.
    while not response:
        response = tare_service(trig_obj)                           # Now send the request through the connection


def CloseHand():
    # Request the hand to go to close down and stay there maintining torque intil told to open
    rospy.wait_for_service('/close_hand')
    tare_service = rospy.ServiceProxy('/close_hand', Trigger)   # Create the connection to the service.
    trig_obj = TriggerRequest()                                 # Create an object of the type TriggerRequest.
    while not response:
        response = tare_service(trig_obj)                       # Now send the request through the connection


def InitIMU():
    # send signal to Arduino to initialize the IMU at this point
    rospy.wait_for_service('init_IMU')
    init_imu_srv = rospy.ServiceProxy('init_IMU', Test)
    resp = 0
    while not resp:                 # wait till the IMU responds back that it has initialized
        resp = init_imu_srv()
        rospy.sleep(0.25)
    print(" ")
    print("IMU has been initialized")
    print(" ")



def Go2Pose(goal):
    # move the arm to the desired goal pose
    # Get the current location of the end effector
    stamp = rospy.Time.now()
    tool_to_ur_base_tf = UR5.get_tf(ur_base_link, tool_frame, stamp)
    movement_to_ur_base_tf = UR5.get_tf(ur_base_link, movement_frame, stamp)
    static_tool_to_ur_ee_pose = UR5.convert_tf_to_pose(UR5.get_tf(ur_end_link, tool_frame, stamp))
    tool_to_rotation_tf = UR5.get_tf(movement_frame, tool_frame, stamp)

    # Define a pose in the movement frame whose origin is at the current tool frame's origin (in the movement frame)
    # plus any desired movement, and the rotation is the desired rotation

    tool_origin = UR5.point_to_array(tool_to_rotation_tf.transform.translation)

    # TODO: This desired movement should be fine tuned to whatever application you need
    # Right now it just assumes a negative z movement, but in reality it can be whatever you need
    desired_movement = PoseStamped()
    desired_movement.header.frame_id = movement_frame
    desired_movement.pose.position = Point(*tool_origin + np.array(movement_vector))
    # Kinda hacky, sorry
    desired_rotation_mat = euler_matrix(*movement_rpy)      # THIS IS WHERE YOUR DESIRED ROTATION GOES
    o = tool_to_rotation_tf.transform.rotation
    composite_rotation = desired_rotation_mat.dot(quaternion_matrix([o.x, o.y, o.z, o.w]))
    desired_movement.pose.orientation = Quaternion(*quaternion_from_matrix(composite_rotation))

    current_tool_pose = UR5.convert_tf_to_pose(tool_to_ur_base_tf)              # base_link to tool frame(test_static frame)
    final_tool_pose = do_transform_pose(desired_movement, movement_to_ur_base_tf)   # base_link to movment_frame(base_link)

    # Each pose in the interpolated poses actually represents an orientation of the frame relative to the offset
    # Therefore you are transforming a static offset in the tool frame with a variable transform
    interp_poses = UR5.ros_pose_slerp(current_tool_pose, final_tool_pose, interp_N)
    interpolated_tool_poses = [do_transform_pose(static_tool_to_ur_ee_pose, UR5.convert_pose_to_tf(pose,
                                        static_tool_to_ur_ee_pose.header.frame_id)).pose for pose in interp_poses]

    # Convert the poses into a trajectory which can be received by the action client
    joint_start = rospy.wait_for_message('/joint_states', JointState)
    goal = UR5.convert_poses_to_trajectory(interpolated_tool_poses, joint_start, vel, warn_threshold=sum_abs_radian_warn)

    # Pre-grasp preparation
    UR5.tare_force()
    rospy.sleep(1.0)

    start_recording()

    # You probably want your grasping function here
    grasp = lambda: None
    grasp()

    # Send the trajectory to the server
    try:
        traj_client.send_goal_and_wait(goal, goal.trajectory.points[-1].time_from_start * 2)
    finally:
        UR5.issue_stop()


def RetreatFromPose():
    # match the path taken out to the goal pose and go back to the tare spot

    stop_recording()


if __name__ == '__main__':

    # Initialization stuff - subscribers, publishers, service proxies, action clients
    rospy.init_node('move_arm')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    rospy.Subscriber('/ur_driver/robot_status', RobotStatus, UR5.update_error, queue_size=1)
    urscript_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
    traj_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)

    if not traj_client.wait_for_server(rospy.Duration(5.0)):
        msg = 'Can\'t connect to action server for trajectory uploading!'
        rospy.logwarn(msg)
        rospy.signal_shutdown(msg)
        sys.exit(1)

    start_recording = rospy.ServiceProxy('start_recording', Empty, persistent=True)
    stop_recording = rospy.ServiceProxy('stop_recording', Empty, persistent=True)

    # Move to the Tare position
    Move2Tare()
    rospy.sleep(5)

    # signal for the service to update the Tare_tf
    rospy.wait_for_service('/update_tare')
    tare_service = rospy.ServiceProxy('/update_tare', Trigger)  # Create the connection to the service.
    trig_obj = TriggerRequest()  # Create an object of the type TriggerRequest.
    while not response:
        response = tare_service(trig_obj)  # Now send the request through the connection
    rospy.sleep(5)

    # Free drive to the apple location
    # TODO: May require sending a stop_freedrive() command to the arm after setting to False - modify freedrive_node.py
    rospy.set_param('freedrive', True)
    raw_input('Please freedrive the arm to the desired grasping position, then hit Enter')

    # Gather Pose and save this at the parent goal pose
    rospy.wait_for_service('/set_apple_nom')
    tare_service = rospy.ServiceProxy('/set_apple_nom', Trigger)
    trig_obj = TriggerRequest()
    while not response:
        response = tare_service(trig_obj)
    rospy.sleep(5)

    # Free drive a bit back from the apple and then return to the tare spot
    raw_input('Please move the arm back a bit from the apple, then hit Enter to return home')
    rospy.set_param('freedrive', False)
    rospy.sleep(1.0)
    Move2Tare()
    rospy.sleep(5)

    # Send service request to open up the hand to neutral position
    Hand2NeutralPos()

    # Send service request to initialize the IMU
    InitIMU()

    # # receive user input as to what variant of motion is desired and run that full action
    # # "OG" will run the hand to the original pose
    # # Exit "XX" will run the "XX" variant and will exit this UI loop
    # # "0.5 right" will shift the pose over to the right by 0.5
    # loop = True
    # while loop:
    #     if UI.contains("Exit"):
    #         f_exit = True
    #         loop = False
    #     if UI == "OG":
    #         # desired = OG pose
    #         # Go2Pose(desired)          # Alex this is where most of your code is set-up, I just moved it into a repeatable function
    #         # CloseHand()               # send service request to close the hand
    #         rospy.sleep(5)              # give time for the hand to close and wait to get stabalized data
    #         if not f_exit:
    #             # OpenHand()                # now open the hand so that the arm can retreat without effecting the apple
    #             i = 47                      # just filler code to make python happy
    #         # RetreatFromPose()


    # # Get the current location of the end effector
    # stamp = rospy.Time.now()
    # tool_to_ur_base_tf = get_tf(ur_base_link, tool_frame, stamp)
    # movement_to_ur_base_tf = get_tf(ur_base_link, movement_frame, stamp)
    # static_tool_to_ur_ee_pose = convert_tf_to_pose(get_tf(ur_end_link, tool_frame, stamp))
    # tool_to_rotation_tf = get_tf(movement_frame, tool_frame, stamp)
    #
    # # Define a pose in the movement frame whose origin is at the current tool frame's origin (in the movement frame)
    # # plus any desired movement, and the rotation is the desired rotation
    #
    # tool_origin = point_to_array(tool_to_rotation_tf.transform.translation)
    #
    # # TODO: This desired movement should be fine tuned to whatever application you need
    # # Right now it just assumes a negative z movement, but in reality it can be whatever you need
    # desired_movement = PoseStamped()
    # desired_movement.header.frame_id = movement_frame
    # desired_movement.pose.position = Point(*tool_origin + np.array(movement_vector))
    # # Kinda hacky, sorry
    # desired_rotation_mat = euler_matrix(*movement_rpy)      # THIS IS WHERE YOUR DESIRED ROTATION GOES
    # o = tool_to_rotation_tf.transform.rotation
    # composite_rotation = desired_rotation_mat.dot(quaternion_matrix([o.x, o.y, o.z, o.w]))
    # desired_movement.pose.orientation = Quaternion(*quaternion_from_matrix(composite_rotation))
    #
    # current_tool_pose = convert_tf_to_pose(tool_to_ur_base_tf)              # base_link to tool frame(test_static frame)
    # final_tool_pose = do_transform_pose(desired_movement, movement_to_ur_base_tf)   # base_link to movment_frame(base_link)
    #
    # # Each pose in the interpolated poses actually represents an orientation of the frame relative to the offset
    # # Therefore you are transforming a static offset in the tool frame with a variable transform
    # interp_poses = ros_pose_slerp(current_tool_pose, final_tool_pose, interp_N)
    # interpolated_tool_poses = [do_transform_pose(static_tool_to_ur_ee_pose, convert_pose_to_tf(pose, static_tool_to_ur_ee_pose.header.frame_id)).pose for pose in interp_poses]
    #
    # # Convert the poses into a trajectory which can be received by the action client
    # joint_start = rospy.wait_for_message('/joint_states', JointState)
    # goal = convert_poses_to_trajectory(interpolated_tool_poses, joint_start, vel, warn_threshold=sum_abs_radian_warn)
    #
    # # Pre-grasp preparation
    # tare_force()
    # rospy.sleep(1.0)
    #
    # # start_recording()
    #
    # # You probably want your grasping function here
    # grasp = lambda: None
    # grasp()
    #
    # # Send the trajectory to the server
    # try:
    #     traj_client.send_goal_and_wait(goal, goal.trajectory.points[-1].time_from_start * 2)
    # finally:
    #     issue_stop()
    #
    # # Final stuff
    # stop_recording()
