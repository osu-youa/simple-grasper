#!/usr/bin/env python
# -*- coding: utf-8 -*-

###########################################################
# Written by Lisa Dischinger
# Ros node to control the torque of an Yale openHand manipulator
# will also periodicly publish out dynamixel data
# Built off of Chelse Vanatter'torque control code and XX drive code
#############################################################

import os
import rospy
import time
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Trigger, TriggerResponse

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


import rospkg
rospack = rospkg.RosPack()
package_directory = rospack.get_path('apple_grasper')
os.sys.path.append(os.path.join(package_directory, 'src', 'DynamixelSDK', 'python', 'src', 'dynamixel_sdk'))
# from dynamixel_sdk import *
from port_handler import PortHandler
from protocol1_packet_handler import Protocol1PacketHandler

###########################################################
# Control Table Address
ADDR_MX_BAUD                    = 4
ADDR_MX_CW                      = 6
ADDR_MX_CCW                     = 8
ADDR_MX_MAX_TORQUE              = 14
ADDR_MX_STATUS_RETURN           = 16
ADDR_MX_TORQUE_ENABLE           = 24            # Control table address is different in Dynamixel model
ADDR_MX_MOVING_SPEED            = 32
ADDR_MX_GOAL_TORQUE             = 34            # in the control table this is known as Torque limit
ADDR_MX_GOAL_POSITION           = 30
ADDR_MX_PRESENT_POSITION        = 36
ADDR_MX_PRESENT_SPEED           = 38
ADDR_MX_PRESENT_LOAD            = 40


# Protocol version
PROTOCOL_VERSION                = 1             # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                          = 1                     # Dynamixel ID: 1
BAUDRATE                        = 57600                 # 57600
DEVICENAME                      = "/dev/ttyUSB0"        # Check which port is being used on your controller

TORQUE_ENABLE                   = 1                     # Value for enabling the torque
TORQUE_DISABLE                  = 0                     # Value for disabling the torque
TORQUE_CONTROL_MODE_ENABLE      = 1
TORQUE_CONTROL_MODE_DISABLE     = 0
DXL_MINIMUM_POSITION_VALUE      = 100                    # Dynamixel will rotate between this value and the max
DXL_MAXIMUM_POSITION_VALUE      = 4000   # the Dynamixel will not move when the position value is out of movable range.
DXL_MOVING_STATUS_THRESHOLD_FINGER = 10  # Dynamixel moving status threshold for finger
DXL_MOVING_STATUS_THRESHOLD_SPREAD = 50  # Dynamixel moving status threshold for spread

ESC_ASCII_VALUE = 0x1b

COMM_SUCCESS = 0  # Communication Success result value
COMM_TX_FAIL = -1001  # Communication Tx Failed

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
##########################################################
#flags
f_open = False                          # setting this true will open the hand, should oly be done through service request
f_close = False                         # service request sets to true when it is desired to close the the hand
f_startUp = False
f_shutdown = False

# experiment settings
debug = False                           # set to true if you want to read directly from the readings

seq = 0                                 # header sample number

position = False                       # set to true if it is desired to go to a set position rather than torque
pos_array = [2500, 1000, 0, 0]          # desired finger final positions

# for 0-1,023, CCW torque is applied; for 1,024 ~ 2,047, CW torque is applied. Setting it to 0 or 1,024 will stop.
vel_array = [200, 200, 200, 200]            # desired finger speed; will increase torque to try to maintain that speed
tor_array = [100, 100, 100, 100]            # desired finger torques, x/1023 is the percentage of max set torque to run
vel_array_back = [1324, 1324, 1324, 1324]   # Used to open the hand back up, this is the sae speed as before, just in the op. direction
torque_run_time = 1                    # run time duration to run at those torques

##########################################################


# Class to actuate Openhand finger that attached to DynamixelSDK servo (Leon,2017)
class Dynamixel_servo():
    def __init__(self, port_num, pro_ver, ID = 1, debug=True):
        global COMM_SUCCESS, COMM_TX_FAIL, ADDR_MX_PRESENT_POSITION, ADDR_MX_PRESENT_SPEED, ADDR_MX_PRESENT_LOAD,\
            ADDR_MX_GOAL_POSITION, ADDR_MX_TORQUE_ENABLE, ADDR_MX_MOVING_SPEED, ADDR_MX_GOAL_TORQUE, ADDR_MX_MAX_TORQUE,\
            ADDR_MX_CW, ADDR_MX_CCW, ADDR_MX_BAUD
        self.ID = ID
        self.debug = debug
        self.port_num = port_num
        self.pro_ver = pro_ver
        self.addr_baud = ADDR_MX_BAUD
        self.addr_torque = ADDR_MX_TORQUE_ENABLE
        self.addr_pres_pos = ADDR_MX_PRESENT_POSITION
        self.addr_pres_rpm = ADDR_MX_PRESENT_SPEED
        self.addr_pres_load = ADDR_MX_PRESENT_LOAD
        self.addr_pos_goal = ADDR_MX_GOAL_POSITION
        self.addr_vel_goal = ADDR_MX_MOVING_SPEED
        self.addr_tor_goal = ADDR_MX_GOAL_TORQUE
        self.addr_stat_return = ADDR_MX_STATUS_RETURN
        self.addr_dir = [ADDR_MX_CW, ADDR_MX_CCW]
        self.COMM_SUCCESS = COMM_SUCCESS
        self.COMM_TX_FAIL = COMM_TX_FAIL
        self.dxl_comm_result = COMM_TX_FAIL
        self.dxl_error = 0
        self.dxl_present_position = 0
        self.dxl_present_rpm = 0
        self.dxl_present_load = 0
        self.goalpos = 0

        self.reset_mode()               # make sure we are not in wheel mode for the initial motion

    # Initiate torque for servo (Leon, 2017)
    def EnableTorque(self):
        dxl_comm_result, dxl_error = PacH.write1ByteTxRx(self.port_num, self.ID, self.addr_torque, TORQUE_ENABLE)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'Enable torque')
        if not z:
            print("Dynamixel {} has been successfully connected".format(self.ID))
        return None

    def Move(self, goalpos):
        self.goalpos = int(goalpos)
        dxl_comm_result, dxl_error = PacH.write2ByteTxRx(self.port_num, self.ID, self.addr_pos_goal, self.goalpos)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'reading')
        return None

    # Print current position of finger(Leon,2017)
    def PresentPos_finger(self):
        # Print current pos with goalpos
        global DXL_MOVING_STATUS_THRESHOLD
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = PacH.read2ByteTxRx(self.port_num, self.ID, self.addr_pres_pos)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'reading position')

        if not (abs(self.goalpos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD_FINGER):
            if self.debug:
                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.ID, self.goalpos, dxl_present_position))
            return True
        return False

    # Print current position of spread (Leon, 2017)
    def PresentPos_spread(self):
        # Print current pos with goalpos
        global DXL_MOVING_STATUS_THRESHOLD
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = PacH.read4ByteTxRx(self.port_num, self.ID, self.addr_pres_pos)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'reading spread position')
        if not z and self.debug:
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.ID, self.goalpos, dxl_present_position))

        if not (abs(self.goalpos - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD_SPREAD):
            return True
        return False

    # Print current position of finger without requiring goal pos (Leon, 2017)
    def PresentPos_1(self):
        # Print current pos without goalpos
        global DXL_MOVING_STATUS_THRESHOLD
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = PacH.read2ByteTxRx(self.port_num, self.ID, self.addr_pres_pos)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'reading init_pos')
        if not z and self.debug:
            print("[ID:%03d] PresPos:%03d" % (self.ID, dxl_present_position))
        return None

    # Close servo torque (Leon, 2017)
    def DisableTorque(self):
        global TORQUE_DISABLE
        # Disable Dynamixel Torque
        self.dxl_comm_result, self.dxl_error = PacH.write1ByteTxRx(self.port_num, self.ID, self.addr_torque, TORQUE_DISABLE)
        if self.dxl_comm_result != self.COMM_SUCCESS:
            print(PacH.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print(PacH.getRxPacketError(self.dxl_error))
        return None

    def Finger_Reset(self):                 # Reset to original position
        global DXL_MINIMUM_POSITION_VALUE
        self.Move(DXL_MINIMUM_POSITION_VALUE)
        val = 0
        while not val:
            val = self.PresentPos_finger()
        return None

    def Spread_Reset(self):             # Reset spread to zero position
        self.Move(4095)
        return None

    def runTorque(self, torque, velocity):        # run torque for a set time
        global TORQUE_CONTROL_MODE_ENABLE, TORQUE_CONTROL_MODE_DISABLE

        # set to wheel mode
        dxl_comm_result, dxl_error = PacH.write2ByteTxRx(self.port_num, self.ID, self.addr_dir[0], 0)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'setting CW wheel mode')
        dxl_comm_result, dxl_error = PacH.write2ByteTxRx(self.port_num, self.ID, self.addr_dir[1], 0)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'setting CCW wheel mode')

        # set the torque allowed in this move
        dxl_comm_result, dxl_error = PacH.write2ByteTxRx(self.port_num, self.ID, self.addr_tor_goal, torque)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'setting finger torque')

        # Write goal velocity/ instead of telling it to run torque
        dxl_comm_result, dxl_error = PacH.write2ByteTxRx(self.port_num, self.ID, self.addr_vel_goal, int(velocity))
        z = self.check_for_response(dxl_comm_result, dxl_error, 'setting moving speed')

    def SetStatusRetun(self):
        # set the status return byte so that it will return for any instruction; sometimes I accidentily overwrite this

        # Try to ping the Dynamixel
        # Get Dynamixel model number
        dxl_model_number, dxl_comm_result, dxl_error = PacH.ping(self.port_num, self.ID)
        if dxl_comm_result != self.COMM_SUCCESS:
            print("%s" % PacH.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % PacH.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID, dxl_model_number))

        print("reseting the status bit")
        dxl_comm_result, dxl_error = PacH.write2ByteTxRx(self.port_num, self.ID, self.addr_stat_return, 2)
        if dxl_comm_result != self.COMM_SUCCESS:
            print(PacH.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(PacH.getRxPacketError(dxl_error))
        return None

    def read_addr(self, addr, num_bytes):
        # this reads the value at the specified address
        if num_bytes == 1:
            data, dxl_comm_result, dxl_error = PacH.read1ByteTxRx(self.port_num, self.ID, addr)
            z = self.check_for_response(dxl_comm_result, dxl_error, 'reading')
            if not z:
                print("At addr:{} we found {}".format(addr, data))

        elif num_bytes == 2:
            data, dxl_comm_result, dxl_error = PacH.read2ByteTxRx(self.port_num, self.ID, addr)
            z = self.check_for_response(dxl_comm_result, dxl_error, 'reading')
            if not z:
                print("At addr:{} we found {}".format(addr, data))

    def check_for_response(self, result, error, location):
        global debug
        if result != self.COMM_SUCCESS:
            if debug:
                print("from {}".format(location))
                print("%s" % PacH.getTxRxResult(result))
            return 1
        elif error != 0:
            if debug:
                print("from {}".format(location))
                print("%s" % PacH.getRxPacketError(error))
            return 1
        else:
            return 0

    def reset_mode(self):
        # used to reset back to the original mode, not wheel, joint or multi_rotation mode. when in wheel mode, the
            # motor does not handle going to a goal position
        dxl_comm_result, dxl_error = PacH.write2ByteTxRx(self.port_num, self.ID, self.addr_dir[0], 0)
        z = self.check_for_response(dxl_comm_result, dxl_error, "resetting CW")
        dxl_comm_result, dxl_error = PacH.write2ByteTxRx(self.port_num, self.ID, self.addr_dir[1], 4095)
        z = self.check_for_response(dxl_comm_result, dxl_error, "resetting CCW")

    def corr_baud(self):
        # correct the baud rate to 57600
        dxl_comm_result, dxl_error = PacH.write1ByteTxRx(self.port_num, self.ID, self.addr_baud, 34)
        z = self.check_for_response(dxl_comm_result, dxl_error, "resetting Baud")

    def present_pos(self):    # Print current pos
        dxl_present_position, dxl_comm_result, dxl_error = PacH.read2ByteTxRx(self.port_num, self.ID, self.addr_pres_pos)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'reading position')
        if not z:
            if self.debug:
                print("[ID:%03d] PresPos:%03d" % (self.ID, dxl_present_position))
            return dxl_present_position
        return False

    def present_rpm(self):    # Print current rpm
        dxl_present_speed, dxl_comm_result, dxl_error = PacH.read2ByteTxRx(self.port_num, self.ID, self.addr_pres_rpm)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'reading RPM')
        if not z:
            if self.debug:
                print("[ID:%03d] PresRPM:%03d" % (self.ID, dxl_present_speed))
            return dxl_present_speed
        return False

    def present_load(self):    # Print current load
        dxl_present_load, dxl_comm_result, dxl_error = PacH.read2ByteTxRx(self.port_num, self.ID, self.addr_pres_load)
        z = self.check_for_response(dxl_comm_result, dxl_error, 'reading Load')
        if not z:
            if self.debug:
                print("[ID:%03d] PresLoad:%03d" % (self.ID, dxl_present_load))
            return dxl_present_load
        return False


class MyPublisher:
    def __init__(self):
        # Just set up the empty arrays to publish
        # publish: the stored data collected by path tracker, tare_tf, actual_tf, estimated_tf, raw_IMU_data

        self.pos = Vector3Stamped()             # all dynamixel's encoder position data [D1, D2, D4]
        self.pos.header.frame_id = 'NA position'

        self.rpm = Vector3Stamped()  # all dynamixel's encoder position data [D1, D2, D4]
        self.rpm.header.frame_id = 'NA RPM'

        self.load = Vector3Stamped()  # all dynamixel's encoder position data [D1, D2, D4]
        self.load.header.frame_id = 'NA Load'

    def publish_topics(self):
        """  publish all that shiiiiiiiiittttt  """
        i = 0
        topic_list = [self.pos, self.rpm, self.load]
        for topic in topic_list:           # iterate through the list of topics that we need to boadcast
            # print("published topic {} to name {}".format(topic, labels[i]))
            # print(topic)
            broadcast[labels[i]].publish(topic)
            i += 1

    def update_pos(self, n, pos_list):
        """  gathers dynamixel encoder positional data and sets them up to be published
        :param n: sample number stamp
        :param pos_list: position data form dynamixel servo 1, 2, 4
        """
        self.pos.header.seq = n
        self.pos.header.stamp = rospy.Time.now()

        self.pos.vector.x = pos_list[0]
        self.pos.vector.y = pos_list[1]
        self.pos.vector.z = pos_list[2]

    def update_rpm(self, n, vel_list):
        """  gathers dynamixel encoder positional data and sets them up to be published
        :param n: sample number stamp
        :param vel_list: rpm data form dynamixel servo 1, 2, 4
        """
        self.rpm.header.seq = n
        self.rpm.header.stamp = rospy.Time.now()

        self.rpm.vector.x = vel_list[0]
        self.rpm.vector.y = vel_list[1]
        self.rpm.vector.z = vel_list[2]

    def update_load(self, n, t_list):
        """  gathers dynamixel encoder positional data and sets them up to be published
        :param n: sample number stamp
        :param t_list: load data form dynamixel servo 1, 2, 4
        """
        self.load.header.seq = n
        self.load.header.stamp = rospy.Time.now()

        self.load.vector.x = t_list[0]
        self.load.vector.y = t_list[1]
        self.load.vector.z = t_list[2]


def start_up():
    # runs through the basic dynamixel connections and checks

    # Open port
    if PH.openPort():
        # print("Succeeded to open the port")
        a = 47
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if PH.setBaudRate(BAUDRATE):
        # print("Succeeded to change the baudrate")
        a = 47
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()


def publish_func(event):
    # This will run every set amount of time and will publish
    global seq
    # update the current position values
    p1 = ID_1.present_pos()
    p2 = ID_2.present_pos()
    p4 = ID_4.present_pos()
    my_pub.update_pos(seq, [p1, p2, p4])

    # update the current velocity values
    v1 = ID_1.present_rpm()
    v2 = ID_2.present_rpm()
    v4 = ID_4.present_rpm()
    my_pub.update_rpm(seq, [v1, v2, v4])

    # update the current load values
    t1 = ID_1.present_load()
    t2 = ID_2.present_load()
    t4 = ID_4.present_load()
    my_pub.update_load(seq, [t1, t2, t4])

    # my_pub.update_estimate()
    my_pub.publish_topics()
    seq += 1


def open_hand(request):
    """ This sets the global flag that triggers the openeing of the hand """
    global f_open, f_close
    print("Open the hand")
    f_open = True
    f_close = False                     # we set it to false here because we want to stay closed until the open command is set
    return TriggerResponse(success = True, message = "merhhhh")


def close_hand(request):
    """ This sets the global flag that triggers the closeing and grasping of the hand """
    global f_close
    print("Close the hand")
    f_close = True
    return TriggerResponse(success=True, message="merhhhh")

def start_up_service(*_, **__):
    """ This sets the global flag that triggers the closeing and grasping of the hand """
    global f_startUp
    print("disabling torque")
    f_startUp = True
    return TriggerResponse(success=True, message="merhhhh")

def shutdown(request):
    """ This sets the global flag that triggers the closeing and grasping of the hand """
    global f_shutdown
    print("disabling torque")
    f_shutdown = True
    return TriggerResponse(success=True, message="merhhhh")


if __name__ == '__main__':

    rospy.init_node('Hand_controller')
    rate = rospy.Rate(10)
    my_pub = MyPublisher()

    # Openhand Configuration
    # finger 1 (blue) = ID 2    finger 2 (yellow) = ID 1    finger 3 (orange) = ID 4    Spread = ID 3

    # ROS publisher set-up
    labels = ['position', 'RPM', 'Load']  # the desired topic data that we want to publish
    broadcast = {label: rospy.Publisher('dyn_{}'.format(label), Vector3Stamped, queue_size=1) for label in labels}

    PH = PortHandler(DEVICENAME)                    # Initialize PortHandler Structs (Leon, 2017)
    PacH = Protocol1PacketHandler()                 # Initialize PacketHandler Structs (leon, 2017)
    rospy.on_shutdown(PH.closePort)                 # close the port when ros is shutting down

    # setup the services
    open_service = rospy.Service('/open_hand', Trigger, open_hand)
    close_service = rospy.Service('/close_hand', Trigger, close_hand)
    shutdown_service = rospy.Service('/shutdown', Trigger, shutdown)
    init_service = rospy.Service('/enable', Trigger, start_up_service)

    start_up()
    rospy.sleep(1)

    # Initiate servo ID 1,2,3,4
    ID_1 = Dynamixel_servo(PH, PROTOCOL_VERSION, 1, debug)
    ID_2 = Dynamixel_servo(PH, PROTOCOL_VERSION, 2, debug)
    ID_4 = Dynamixel_servo(PH, PROTOCOL_VERSION, 4, debug)

    # Set some servo conditions
    # ID_1.SetStatusRetun()
    # ID_1.corr_baud()

    # read address values
    # ID_1.read_addr(ADDR_MX_BAUD, 1)

    print(" ")
    print("Dynamixel Control set")
    print(" ")

    # Print current position without goal pos
    ID_1.PresentPos_1()
    ID_2.PresentPos_1()
    ID_4.PresentPos_1()

    # Initiate torque for each servo
    ID_1.EnableTorque()
    ID_2.EnableTorque()
    ID_4.EnableTorque()

    # reset the motion of the fingers
    ID_1.Finger_Reset()
    ID_2.Finger_Reset()
    ID_4.Finger_Reset()

    rospy.sleep(2)



    # Set-up Publisher ISR
    rospy.Timer(rospy.Duration(1), publish_func)            # currently functions at 1 hz

    rospy.loginfo('Hand controller loop entered!')

    while not rospy.is_shutdown():
        if f_open:              # set to true when there was a service call to open the hand
            ID_1.runTorque(tor_array[0], vel_array_back[0])
            ID_2.runTorque(tor_array[1], vel_array_back[1])
            ID_4.runTorque(tor_array[3], vel_array_back[3])
            rospy.sleep(5)
            ID_1.runTorque(tor_array[0], 0)
            ID_2.runTorque(tor_array[1], 0)
            ID_4.runTorque(tor_array[3], 0)
            f_open = False

        if f_close:             # close the hand
            ID_1.runTorque(tor_array[0], vel_array[0])
            ID_2.runTorque(tor_array[1], vel_array[1])
            ID_4.runTorque(tor_array[3], vel_array[3])

        if f_startUp:
            # End each servo torque
            ID_1.EnableTorque()
            ID_2.EnableTorque()
            ID_4.EnableTorque()
            f_startUp = False

        if f_shutdown:
            # End each servo torque
            ID_1.DisableTorque()
            ID_2.DisableTorque()
            # ID_3.DisableTorque()
            ID_4.DisableTorque()
            f_shutdown = False
