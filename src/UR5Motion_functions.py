#!/usr/bin/env python

# this is just a way to clear up the main script; this is just a folder for all of the functions that are used
# to make the UR5 move; most of these functions are written by Alex

import rospy
import kinematics as urkin
from geometry_msgs.msg import Pose, Point, Quaternion, TwistStamped, TransformStamped, Transform
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_slerp, euler_from_matrix, quaternion_from_matrix, euler_matrix, quaternion_matrix
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Vector3
from std_msgs.msg import Header
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from tf2_geometry_msgs import do_transform_pose
from sensor_msgs.msg import JointState


ERROR = 0
#
# # Config - These define the motion to be completed. You can modify this, but if you want multiple options, you may
# # want to modify the code to take it through a sys.argv
# # (e.g. running it like "rosrun apple_grasper perform_grasp.py 0.05 0.20
# retreat_dist = 0.05                 # How far back does the arm move? + means backwards
# rotation = radians(20)              # How much should the end effector rotate? Currently assumes around Z axis
# vel = 0.04                          # What's the speed the arm should move?
# interp_N = 20                       # How many points should be interpolated for the trajectory?
# sum_abs_radian_warn = radians(10)   # If any total movement exceeds this amount, issue a warning
#
# # Frames
# # THESE FRAMES SHOULD NOT BE CHANGED because the kinematics solver assumes these references
# ur_base_link = 'base'
# ur_end_link = 'tool0'
#
# # THESE FRAMES CAN BE CHANGED as they are not bound to the UR5 program code
# tool_frame = 'test_frame'                   # What is the frame you're trying to move?
#                                             # IMPORTANT! THIS FRAME MUST BE STATIC w.r.t. ur_end_link
# movement_frame = 'base_link'                # What frame is the movement/rotation vector defined in?
# movement_vector = [-retreat_dist, 0, 0]     # What's the linear movement you want in the movement frame?
# movement_rpy = [0, 0, rotation]             # What's the rotation you want in the movement frame, centered around tool_frame's origin?


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
    pos = Point(mat[0, 3], mat[1, 3], mat[2, 3])
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

    # if warn_threshold > 0:   # LMD
    #     joints = np.array(joints)
    #     sum_abs_diff = np.abs(joints[:-1] - joints[1:]).sum(axis=1)
    #     if np.any(sum_abs_diff > warn_threshold):
    #         rospy.logwarn('Detected a large joint movement! Either near singularity or need to increase path resolution')
    #         sys.exit(1)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    return goal


def freedriveMode():
    # temprary time to freedrive the system to a spot and to check if it is nnot near any singularities
    # TODO: May require sending a stop_freedrive() command to the arm after setting to False - modify freedrive_node.py
    rospy.set_param('freedrive', True)
    raw_input('Please freedrive the arm to the desired grasping position, then hit Enter')
    rospy.set_param('freedrive', False)
    rospy.sleep(1.0)


####################
# Basic Move_arm functions
####################

def generate_ur_pose(pose):
    quat = pose.orientation
    rx, ry, rz = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    x, y, z = pose.position.x, pose.position.y, pose.position.z
    format_str = 'p[{}]'.format(','.join(['{:.3f}'] * 6))
    return format_str.format(x, y, z, rx, ry, rz)

def generate_move_command(pose, a, v, t=0):
    ur_pose = generate_ur_pose(pose)
    format_str = 'movej({}, a={:.3f}, v={:.3f}, t={:.3f}, r=0)'
    return format_str.format(ur_pose, a, v, t)

def wait_for_motion_to_complete():

    # Kind of a hack, couldn't figure out a quick way to get the UR5 move status
    rate = rospy.Rate(10)
    epsilon = 0.001

    rospy.sleep(0.5)

    while True:
        twist = rospy.wait_for_message('tool_velocity', TwistStamped)
        lin = twist.twist.linear
        ang = twist.twist.angular

        lin_array = np.array([lin.x, lin.y, lin.z])
        ang_array = np.array([ang.x, ang.y, ang.z])

        if np.linalg.norm(lin_array) < epsilon and np.linalg.norm(ang_array) < epsilon:
            break
        rate.sleep()

