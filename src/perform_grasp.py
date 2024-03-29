#!/usr/bin/env python


import rospy
import roslib
import kinematics as urkin
# roslib.load_manifest("ur_kinematics")
# from ur_kin_py import forward, inverse
from std_msgs.msg import String
from industrial_msgs.msg import RobotStatus
from geometry_msgs.msg import Pose, Point, Quaternion, TwistStamped, TransformStamped, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_slerp, euler_from_matrix, quaternion_from_matrix, euler_matrix, quaternion_matrix
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Vector3
from collections import defaultdict
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Header
from copy import deepcopy
from math import radians, ceil
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
import sys
from std_srvs.srv import Empty

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
movement_frame = 'base_link'                # What frame is the movement/rotation vector defined in?
movement_vector = [0, -retreat_dist, 0]     # What's the linear movement you want in the movement frame?
movement_rpy = [0, 0, rotation]             # What's the rotation you want in the movement frame, centered around tool_frame's origin?

def update_error(msg):
    global ERROR
    ERROR = msg.in_error.val

# def generate_ur_pose(pose):
#     quat = pose.orientation
#     rx, ry, rz = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
#     x, y, z = pose.position.x, pose.position.y, pose.position.z
#     format_str = 'p[{}]'.format(','.join(['{:.3f}'] * 6))
#     return format_str.format(x, y, z, rx, ry, rz)

# def format_array(array):
#
#     values = ','.join(['{}'.format(x) for x in array])
#     return '[{}]'.format(values)
#
#
# def format_array_decimal(array):
#     values = ','.join(['{:.2f}'.format(x) for x in array])
#     return '[{}]'.format(values)


def get_tf(target, source, stamp = rospy.Time(0)):
    tf_buffer.can_transform(target, source, stamp, rospy.Duration(0.5))
    tf = tf_buffer.lookup_transform(target, source, stamp)
    return tf

def point_to_array(pt):
    # Also works on vectors!
    return np.array([pt.x, pt.y, pt.z])

def quat_to_array(quat):
    return np.array([quat.x, quat.y, quat.z, quat.w])

def tare_force():
    urscript_pub.publish('zero_ftsensor()\n')

# def issue_linear_velocity_command(array):
#     array_str = format_array_decimal(array)
#     rospy.loginfo_throttle(0.025, array_str)
#     cmd = "speedl({},-5.0,0.05)\n".format(array_str)
#     urscript_pub.publish(cmd)

def issue_stop():
    cmd = "stopl(50.0)\n"
    urscript_pub.publish(cmd)

# def generate_move_command(pose, a, v, t=0, linear=False):
#
#     cmd = 'movel' if linear else 'movej'
#
#     ur_pose = generate_ur_pose(pose)
#     format_str = '{}({}, a={:.3f}, v={:.3f}, t={:.3f}, r=0)'
#     return format_str.format(cmd, ur_pose, a, v, t)

def ros_pose_slerp(start, end, n):

    frame = None
    if isinstance(start, PoseStamped):
        frame = start.header.frame_id
        start = start.pose
        end = end.pose

    start_pos = point_to_array(start.position)
    end_pos = point_to_array(end.position)

    positions = np.linspace(start_pos, end_pos, n, endpoint=True)
    quats = [quaternion_slerp(quat_to_array(start.orientation), quat_to_array(end.orientation), i) for i in np.linspace(0, 1, n, endpoint=True)]

    poses = [Pose(Point(*pos), Quaternion(*quat)) for pos, quat in zip(positions, quats)]
    if frame is not None:
        header = Header()
        header.frame_id = frame
        poses_stamped = [PoseStamped(header, pose) for pose in poses]
        return poses_stamped
    return poses


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


def ros_quat_to_euler(q):
    return np.array(euler_from_quaternion([q.x, q.y, q.z, q.w]))

def construct_joint_point(position, t):
    point = JointTrajectoryPoint()
    point.positions = position
    if not isinstance(t, rospy.Duration):
        t = rospy.Duration(t)
    point.velocities = [0.0] * len(position)
    point.accelerations = [0.0] * len(position)
    point.time_from_start = t
    return point

def convert_mat_to_pose(mat):
    quat = Quaternion(*quaternion_from_matrix(mat))
    pos = Point(mat[0,3], mat[1,3], mat[2,3])
    return Pose(pos, quat)

def convert_tf_to_pose(tf):
    pose = PoseStamped()
    pose.pose = Pose(Point(*point_to_array(tf.transform.translation)), tf.transform.rotation)
    pose.header.frame_id = tf.header.frame_id

    return pose

def convert_pose_to_tf(pose_stamped, frame_id):
    # frame_id is the ID of the frame which is located at the pose in the given frame
    tf = TransformStamped()
    tf.header.frame_id = frame_id
    tf.child_frame_id = pose_stamped.header.frame_id
    tf.transform.translation = Vector3(*point_to_array(pose_stamped.pose.position))
    tf.transform.rotation = pose_stamped.pose.orientation

    return tf


def convert_poses_to_trajectory(poses, initial_joint_state, linear_vel, leeway = 2.0, warn_threshold = 0):


    last_joints = initial_joint_state.position
    last_pos = np.array(urkin.fwd_kin(last_joints)[:3,3]).T[0]


    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = initial_joint_state.name
    traj.points.append(construct_joint_point(last_joints, 0.0))

    # Kind of a hack, offset correction due to fwd kins not being exact

    believed_pos = point_to_array(urkin.fwd_kin(last_joints, o_unit='p').position)
    correction = believed_pos - point_to_array(poses[0].position)

    joints = [last_joints]

    t = 0.0
    for pose in poses:

        pose.position = Point(*point_to_array(pose.position) + correction)
        new_pos = point_to_array(pose.position)
        new_joints = urkin.inv_kin(pose, last_joints)
        dt = max(np.linalg.norm(new_pos - last_pos) / linear_vel * leeway, 0.01)
        t += dt
        traj.points.append(construct_joint_point(new_joints, t))

        last_joints = new_joints
        last_pos = new_pos

        joints.append(new_joints)

    if warn_threshold > 0:
        joints = np.array(joints)
        sum_abs_diff = np.abs(joints[:-1] - joints[1:]).sum(axis=1)
        if np.any(sum_abs_diff > warn_threshold):
            rospy.logwarn('Detected a large joint movement! Either near singularity or need to increase path resolution')
            sys.exit(1)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    return goal


if __name__ == '__main__':

    # Initialization stuff - subscribers, publishers, service proxies, action clients
    rospy.init_node('move_arm')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)
    rospy.Subscriber('/ur_driver/robot_status', RobotStatus, update_error, queue_size=1)
    urscript_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
    traj_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    if not traj_client.wait_for_server(rospy.Duration(5.0)):
        msg = 'Can\'t connect to action server for trajectory uploading!'
        rospy.logwarn(msg)
        rospy.signal_shutdown(msg)
        sys.exit(1)

    start_recording = rospy.ServiceProxy('start_recording', Empty)
    stop_recording = rospy.ServiceProxy('stop_recording', Empty)

    # # Uncomment this section to allow freedriving the arm
    # # TODO: May require sending a stop_freedrive() command to the arm after setting to False - modify freedrive_node.py
    # rospy.set_param('freedrive', True)
    # raw_input('Please freedrive the arm to the desired grasping position, then hit Enter')
    # rospy.set_param('freedrive', False)
    # rospy.sleep(1.0)

    # Get the current location of the end effector
    stamp = rospy.Time.now()

    tool_to_ur_base_tf = get_tf(ur_base_link, tool_frame, stamp)
    movement_to_ur_base_tf = get_tf(ur_base_link, movement_frame, stamp)
    static_tool_to_ur_ee_pose = convert_tf_to_pose(get_tf(ur_end_link, tool_frame, stamp))
    tool_to_rotation_tf = get_tf(movement_frame, tool_frame, stamp)

    # Define a pose in the movement frame whose origin is at the current tool frame's origin (in the movement frame)
    # plus any desired movement, and the rotation is the desired rotation

    tool_origin = point_to_array(tool_to_rotation_tf.transform.translation)

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

    current_tool_pose = convert_tf_to_pose(tool_to_ur_base_tf)
    final_tool_pose = do_transform_pose(desired_movement, movement_to_ur_base_tf)

    # Each pose in the interpolated poses actually represents an orientation of the frame relative to the offset
    # Therefore you are transforming a static offset in the tool frame with a variable transform
    interp_poses = ros_pose_slerp(current_tool_pose, final_tool_pose, interp_N)
    interpolated_tool_poses = [do_transform_pose(static_tool_to_ur_ee_pose, convert_pose_to_tf(pose, static_tool_to_ur_ee_pose.header.frame_id)).pose for pose in interp_poses]

    # Convert the poses into a trajectory which can be received by the action client
    joint_start = rospy.wait_for_message('/joint_states', JointState)
    goal = convert_poses_to_trajectory(interpolated_tool_poses, joint_start, vel, warn_threshold=sum_abs_radian_warn)

    # Pre-grasp preparation
    tare_force()
    rospy.sleep(1.0)
    start_recording()

    # You probably want your grasping function here
    grasp = lambda: None
    grasp()

    # Send the trajectory to the server
    try:
        traj_client.send_goal_and_wait(goal, goal.trajectory.points[-1].time_from_start * 2)
    finally:
        issue_stop()

    # Final stuff
    stop_recording()
