#! /usr/bin/env python3
# license removed for brevity
import rospy
import math
import threading

# import getch
import numpy as np
import sys, select, termios, tty
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from snake_control.msg import GaitParameter
from snake_control.msg import ParamtoSine
from snake_control.msg import DstTime
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('test_move', anonymous=True)

    pub_j1 = rospy.Publisher('/prrp_control/joint_1_position_controller/command', Float64, queue_size=10)
    pub_j2 = rospy.Publisher('/prrp_control/joint_2_position_controller/command', Float64, queue_size=10)
    pub_j3 = rospy.Publisher('/prrp_control/joint_3_position_controller/command', Float64, queue_size=10)
    pub_j4 = rospy.Publisher('/prrp_control/joint_4_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        pub_j1.publish(0)
        pub_j2.publish(0)
        pub_j3.publish(0)
        pub_j4.publish(0)

        # print(0)

        rate.sleep()
