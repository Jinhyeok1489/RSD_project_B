#! /usr/bin/env python3
# license removed for brevity
import rospy
import math as m
import threading

# import getch
import numpy as np
import sys, select, termios, tty
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

def DH_modified(alp, a, d, th):
    """
    * Compute DH transformation matrix using DH parameters
    INPUT:
            alp:  angles between x axis
            a:    distance between x axis
            d:    distance between z axis
            th:   angles between z axis
    OUTPUT:
            T:    transformation matrix
    """
    
    rotx = np.array([[1, 0, 0, 0],[0, m.cos(alp), -m.sin(alp), 0],
                     [0, m.sin(alp), m.cos(alp), 0], [0, 0, 0, 1]])
    transa = np.array([[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    transz = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]])
    rotz = np.array([[m.cos(th), -m.sin(th), 0, 0],[m.sin(th), m.cos(th), 0, 0],
                    [0, 0, 1, 0],[0, 0, 0, 1]])
    
    T = rotx@transa@transz@rotz
    
    return T

def fkine_prrp(z1, l1, l2, th1, th2, z2):
    """
    * Compute prrp kinematics
    INPUT:
            alp:  angles between x axis
            a:    distance between x axis
            d:    distance between z axis
            th:   angles between z axis
    OUTPUT:
            T:    transformation matrix
    """
    TB1 = DH_modified(0, 0, z1, 0)
    T12 = DH_modified(0, 0, 0, th1)
    T23 = DH_modified(0, l1, 0, th2)
    T34 = DH_modified(0, l2, 0, 0)
    T4E = DH_modified(0, 0, -z2, 0)
    
    TB2 = TB1@T12
    TB3 = TB2@T23
    TB4 = TB3@T34
    TBE = TB4@T4E
    
    T = TBE
    
    return T

def ikine_prrp(pose, l1, l2):
    """
    * Compute prrp kinematics
    INPUT:
            pose:   cartesian position (x, y, z)
            l1:     link 1 length
            l2:     link 2 length
    OUTPUT:
            vals:   joint values (z1, th1, th2, z2)
    """   
    x = pose[0]
    y = pose[1]
    z = pose[2]
    
    out = -0.02
    # Assume that end-effector is out with constant value
    z1 = z+out
    z2 = out
    
    cth2 = (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
    sth2 = np.array([m.sqrt(1-cth2**2), -m.sqrt(1-cth2**2)])
    
    
    th2s = np.zeros(2)
    th1s = np.zeros(2)
    for i in range(len(sth2)):
        th2s[i] = m.atan2(sth2[i], cth2)
        th1s[i] = m.atan2(y, x)-m.atan2(l2*sth2[i], l1+l2*cth2)
        
    
    vals = np.array([[z1, th1s[0], th2s[0], z2],
                     [z1, th1s[1], th2s[1], z2]])
    
    return vals

if __name__ == '__main__':
    rospy.init_node('test_move', anonymous=True)

    pose = [0.32, -0.18, -0.075]
    l1 = 0.2
    l2 = 0.22
    ikine_sol = ikine_prrp(pose, l1, l2)
    sol1 = ikine_sol[0, :]
    sol2 = ikine_sol[1, :]
    # for i in range(2):
    #     z1 = ikine_sol[i,0]
    #     th1 = ikine_sol[i,1]
    #     th2 = ikine_sol[i,2]
    #     z2 = ikine_sol[i,3]
        
    #     print(th1, th2)    
        
    #     sol =  fkine_prrp(z1, l1, l2, th1, th2, z2)
    #     sol_pos = sol[0:3, 3]
        
    #     print(sol_pos)

    pub = rospy.Publisher('/prrp/joint_states', JointState, latch=True, queue_size=1)

    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        aaa = JointState()
        aaa.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        aaa.position  = sol1

        pub.publish(aaa)

        # print(0)

        rate.sleep()