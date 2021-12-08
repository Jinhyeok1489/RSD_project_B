#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math as m
import time
import pandas as pd
import numpy as np
import rospy
import threading

from rsd_vision.msg import Card_pose


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

from dynamixel_sdk import *                 # Uses Dynamixel SDK library


def Enable_torque(DXL_ID):
    # Enable dynamixel by receiving dynamixel ID
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL_ID)


def Disable_torque(DXL_ID):
    # Disable dynamixel by receiving dynamixel ID
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_XM_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def Move_dyn4(DXL_ID, goal_position):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_POSITION, goal_position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def Syncwrite_param1(DXL_ID, param_goal_position):
    dxl_addparam_result = groupSyncWrite1.addParam(DXL_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addpcoxaaram failed" % DXL_ID)
        quit()

def Syncwrite_goal_position1():
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
    groupSyncWrite.clearParam() # Clear syncwrite parameter storage

def byte_array_goal(desired_value):
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(desired_value)), DXL_HIBYTE(DXL_LOWORD(desired_value)), DXL_LOBYTE(DXL_HIWORD(desired_value)), DXL_HIBYTE(DXL_HIWORD(desired_value))]
    return param_goal_position

def byte_array_goal2(desired_value):
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(desired_value)), DXL_HIBYTE(DXL_LOWORD(desired_value)), DXL_LOBYTE(DXL_HIWORD(desired_value)), DXL_HIBYTE(DXL_HIWORD(desired_value))]
    return param_goal_position

def Syncwrite_param(DXL_ID, param_goal_position):
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addpcoxaaram failed" % DXL_ID)
        quit()

def Syncwrite_goal_position():
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    groupSyncWrite.clearParam() # Clear syncwrite parameter storage


def Read_current_position(DXL_ID, desired_value):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    position_dif = abs(desired_value - dxl_present_position)
    return [dxl_present_position, position_dif]

def get_desired_value(desired_angle):
    # desired_value = int((desired_angle + 90) * (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE) / 180 + DXL_MINIMUM_POSITION_VALUE)
    desired_value = int(desired_angle * (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE) / 360 + 2048)
    # print(desired_value)
    # print(byte_array_goal(desired_value))
    return byte_array_goal(desired_value)

def get_desired_rad(desired_angle):
    # desired_value = int((desired_angle + 90) * (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE) / 180 + DXL_MINIMUM_POSITION_VALUE)
    desired_value = int(desired_angle * (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE) / (2*m.pi) + 2048)
    # print(desired_value)
    # print(byte_array_goal(desired_value))
    return byte_array_goal(desired_value)

def get_rad_offset(desired_angle, offset, min_angle, max_angle):
    """
    Compute desired angle considering offset value
    * Input:
        desired_angle:  desired joint angle
        offset:         offset angle (if not calibrated)
    * Output:
    """
    rad_value = (desired_angle + offset)

    print("Angle: ", rad_value*180/m.pi)

    if rad_value < min_angle:
        raise Exception("Angle is too small...")
    elif rad_value > max_angle:
        raise Exception("Angle is too large...")

    desired_value = int(rad_value * (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE) / (2*m.pi) + 2048)



    return byte_array_goal(desired_value)
    

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

def fkine_prrp(l1, l2, th1, th2):
    """
    * Compute rr kinematics
    INPUT:
            alp:  angles between x axis
            a:    distance between x axis
            d:    distance between z axis
            th:   angles between z axis
    OUTPUT:
            T:    transformation matrix
    """
    # TB1 = DH_modified(0, 0, z1, 0)
    TB1 = DH_modified(0, 0, 0, th1)
    T12 = DH_modified(0, l1, 0, th2)
    T2E = DH_modified(0, l2, 0, 0)
    # T4E = DH_modified(0, 0, -z2, 0)
    
    TB2 = TB1@T12
    TBE = TB2@T2E
    # TB4 = TB3@T34
    # TBE = TB4@T4E
    
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
    
    cth2 = (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
    sth2 = np.array([m.sqrt(1-cth2**2), -m.sqrt(1-cth2**2)])
    
    
    th2s = np.zeros(2)
    th1s = np.zeros(2)
    for i in range(len(sth2)):
        th2s[i] = m.atan2(sth2[i], cth2)
        th1s[i] = m.atan2(y, x)-m.atan2(l2*sth2[i], l1+l2*cth2)
        
    
    vals = np.array([[ th1s[0], th2s[0]],
                     [ th1s[1], th2s[1]]])
    
    return vals

def cubic(p0, pf , tf, t_step):
    """
    Compute cubic cartesian trajectory between starting point and end point
    * Input:
            p0:     starting point
            pf:     end point
            tf:     final time
    * Output:
            p:      linearized trajectory
    """
    pi = np.array(p0)
    pf = np.array(pf)

    L = np.linalg.norm(pi-pf)
    t = np.arange(0, tf+t_step, t_step)
    p = np.zeros((t.size, 2))

    sigma =     3*L*pow(t,2)/pow(tf, 2)-2*L*pow(t,3)/pow(tf,3)
    d_sigma =   6*L*t/pow(tf,2)-L*pow(t,2)/pow(tf,3)
    dd_sigma =  6*L/pow(tf,2)-12*L*t/pow(tf,3)

    for i in range(t.size):
        p[i] = pi + sigma[i]*(pf-pi)/L

    return p

def cubic_joint(q0, qf, v0, vf, tf, t_step):
    a0 = q0
    a1 = v0
    a2 = 3*(qf-q0)/pow(tf,2)-2*v0/tf-vf/tf
    a3 = -2*(qf-q0)/pow(tf,3)+(vf+v0)/pow(tf,2)

    t = np.arange(0, tf+t_step, t_step)
    theta = a0+a1*t+a2*pow(t,2)+a3*pow(t,3)

    return theta

def callback(data):
    global card_pose_x
    global card_pose_y
    global card_name
    # rospy.loginfo("Current card pose: %s", data.data)
    card_pose_x = data.pos_x
    card_pose_y = data.pos_y
    card_name = data.card_name

    # rospy.loginfo("CardName: %s", card_name)


    # rospy.loginfo("Card pose: (%s, %s)", card_pose_x, card_pose_y)

    # rospy.loginfo("Data data: %s", card_pose)

def readCams():
    """
    * Subscribe camera data
    """
    rospy.Subscriber('card_pose', Card_pose, callback)
    
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('test_move', anonymous=True)

    listener = threading.Thread(target = readCams)
    listener.start()



    # DYNAMIXEL Protocol Version (1.0 / 2.0)
    # https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION = 2.0

    BAUDRATE = 1000000

    # Dynamixel with protocol version 2
    DXL_R1_ID = 1
    DXL_R2_ID = 0
    DXL_P1_ID = 3
    DXL_P2_ID = 2
    DXL_MAXIMUM_POSITION_VALUE = 4095
    DXL_MINIMUM_POSITION_VALUE = 0


    # Use the actual port assigned to the U2D2.
    # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
    DEVICENAME = '/dev/ttyUSB1'

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Addreses for motor control

    # XM-series
    ADDR_XM_GOAL_POSITION = 116
    ADDR_XM_TORQUE_ENABLE = 64
    ADDR_XM_CURR_POSITION = 132
    ADDR_XM_OPERATION = 11

    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0

    # Open port 1 (Port 1: MX-64AT)
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()
    # Set port baudrate1 (Port1: MX-64AT)
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # mat_mani = np.array([DXL_R1_ID, DXL_R2_ID ,DXL_P1_ID, DXL_P2_ID])
    mat_mani = np.array([DXL_R1_ID, DXL_R2_ID, DXL_P1_ID ,DXL_P2_ID])
    for i in range(len(mat_mani)):
        Disable_torque(mat_mani[i])


    # Set operating mode to extended position control mode - motor R1
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_R1_ID, ADDR_XM_OPERATION, 4)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))  
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Operating mode changed to extended position control mode.")

    # Set operating mode to extended position control mode - motor R2
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_R2_ID, ADDR_XM_OPERATION, 4)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))  
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Operating mode changed to extended position control mode.")

    # Set operating mode to extended position control mode - motor P1
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_P1_ID, ADDR_XM_OPERATION, 4)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))  
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Operating mode changed to extended position control mode.")
    # Set operating mode to extended position control mode - motor P2
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_P2_ID, ADDR_XM_OPERATION, 4)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))  
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Operating mode changed to extended position control mode.")


    LEN_GOAL_POSITION1 = 4
    LEN_PRESENT_POSITION1 = 4
    LEN_GOAL_POSITION2 = 4
    LEN_PRESENT_POSITION2 = 4
    # Initialize GroupSyncWrite instance
    # Set as operation modes


    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_XM_GOAL_POSITION, LEN_GOAL_POSITION1)
    # Initialize GroupSyncRead instace for Present Position
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_XM_CURR_POSITION, LEN_PRESENT_POSITION1)

    print(dxl_comm_result)


    mat_mani = np.array([DXL_R1_ID, DXL_R2_ID, DXL_P1_ID, DXL_P2_ID])
    for i in range(len(mat_mani)):
        Enable_torque(mat_mani[i])

    init_time = time.time()

    l1 = 0.22 # Firt link length
    l2 = 0.20 # second link length


    pos = np.array([0.40, 0.05])

    min_arr = np.array([20, -74, -204, 40])*m.pi/180
    max_arr = np.array([250, 156, 160, 92])*m.pi/180

    offset_arr = np.array([135, 41, -22, 66])*m.pi/180


    # kine = fkine_prrp(0.22, 0.2, ang1[0], ang1[1])


    # print("Kine: ",kine[0:2, 3])

    home = np.array([0, 0])

    th1_home = 184*m.pi/180 + offset_arr[0]
    th2_home = 0*m.pi/180 + offset_arr[1]
    home_kine = fkine_prrp(0.22, 0.2, th1_home, th2_home)
    pose_home = home_kine[0:2, 3]
    pose_home[1] = -pose_home[1]
    # Global variable (from subscriber)
    card_pose_x = -1.0
    card_pose_y = -1.0
    card_name = 'None'

    print("pose_home: ",pose_home)


    # th1_2 = np.linspace(m.pi/2, 0, num = 50, endpoint = True)
    # th2_2 = np.linspace(m.pi/2, 0, num = 50, endpoint = True)

    start = np.array([0.42, 0])
    end = np.array([0.40, 0.05])

    pp = cubic(start, pose_home, 5, 0.1)

    up = cubic_joint(136, -100, 0, 0, 5, 0.1)*m.pi/180
    down  = cubic_joint(-100, 136, 0, 0, 5, 0.1)*m.pi/180

    down2 = cubic_joint(-100, 80, 0, 0, 5, 0.1)*m.pi/180
    up2 = cubic_joint(80, -100, 0, 0, 5, 0.1)*m.pi/180

    # up = np.linspace(136, -100,  num = len(pp))*m.pi/180
    # down = np.linspace(-100, 136, num = len(pp))*m.pi/180

    upper = -100*m.pi/180
    lower = 136*m.pi/180

    p2_ori = -19*m.pi/180 # 45 degree -1
    p2_up = cubic_joint(-19, 25, 0, 0, 5, 0.1)*m.pi/180
    p2_down = cubic_joint(25, -19, 0, 0, 5, 0.1)*m.pi/180

    print(len(pp))

    index = 0

    # pub = rospy.Publisher('/prrp/joint_states', JointState, latch=True, queue_size=1)

    # rate = rospy.Rate(25)

    while not rospy.is_shutdown():
        x = input('Command: ')
        x = int(x)
        # 0: if start from base 
        # 1: if start from homing
        if x == 0:
            while 1: #  While loop 1: initial to home position
            
            
                if index == len(pp):
                    break
                # print("up: ", up)
                
                rospy.loginfo("Phase 1-1 - Initial pose to home position(go up)")
                th_arr = ikine_prrp(pp[0], 0.22, 0.2)
                # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
                th1 = th_arr[0]
                th2 = th_arr[1]

                angles = np.append(th2, up2[index])
                angles = np.append(angles, p2_ori)

                for i in range(len(mat_mani)):
                    print(i)
                    temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])

                    Syncwrite_param(mat_mani[i], temp)
                Syncwrite_goal_position()   
                index = index + 1 
                time.sleep(0.05)

            index = 0
            while 1: #  While loop 2: initial to home position (Move to homing)
            
                if index == len(pp):
                    break
                # print("up: ", up)
                
                rospy.loginfo("Phase 1-2 - Initial pose to home position(move to homing)")
                th_arr = ikine_prrp(pp[index], 0.22, 0.2)
                # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
                th1 = th_arr[0]
                th2 = th_arr[1]

                angles = np.append(th2, upper)
                angles = np.append(angles, p2_ori)

                for i in range(len(mat_mani)):
                    print(i)
                    temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])

                    Syncwrite_param(mat_mani[i], temp)
                Syncwrite_goal_position()   
                index = index + 1 
                time.sleep(0.05)




            time.sleep(10.0)

        time.sleep(5.0)
        index = 0
        while not rospy.is_shutdown(): # While loop 2: Move

            print("Phase 2 -Move to card")
            while card_pose_x == -1 and card_pose_y == -1:
                print("주화입마에 ")
                pass
            

            if index == 0:
                offset = 0.02
                curr_pose = np.array([-card_pose_y, -card_pose_x + 0.02])
                rospy.loginfo("Current_pose: %s", curr_pose)
                pp2 = cubic(pose_home, curr_pose, 5, 0.1)
                rospy.loginfo("trajectory home to card: %s", pp2)
                pp2_h = cubic(curr_pose, pose_home, 5, 0.1)
                print("len pp: ", len(pp), " index: ", index)

                offset_x = -0.03

                base1 = np.array([0.31+offset_x, -0.04 - offset])
                base2 = np.array([0.31+offset_x, 0.035 - offset])
                base3 = np.array([0.31+offset_x, 0.10 - offset])
                base4 = np.array([0.40+offset_x, -0.04 - offset])

                pp_b1 = cubic(curr_pose, base1, 5, 0.1)
                pp_b2 = cubic(curr_pose, base2, 5, 0.1)
                pp_b3 = cubic(curr_pose, base3, 5, 0.1)
                pp_b4 = cubic(curr_pose, base4, 5, 0.1) 

                pp_b1b = cubic(base1, pose_home, 5, 0.1)
                pp_b2b = cubic(base2, pose_home, 5, 0.1)
                pp_b3b = cubic(base3, pose_home, 5, 0.1)
                pp_b4b = cubic(base4, pose_home, 5, 0.1)

                # if 'Dia' in card_name:
                #     target_fore = pp_b1
                #     target_back = pp_b1b
                # elif 'Heart' in card_name:
                #     target_fore = pp_b2
                #     target_back = pp_b2b
                # elif 'Spade' in card_name:
                #     target_fore = pp_b3
                #     target_back = pp_b3b
                # elif 'Club' in card_name:
                #     target_fore = pp_b4
                #     target_back = pp_b4b

                if 'K' in card_name:
                    target_fore = pp_b1
                    target_back = pp_b1b
                elif 'Q' in card_name:
                    target_fore = pp_b2
                    target_back = pp_b2b
                elif 'J' in card_name:
                    target_fore = pp_b3
                    target_back = pp_b3b
                # elif 'Club' in card_name:
                #     target_fore = pp_b4
                #     target_back = pp_b4b


            # rospy.loginfo('Current pattern: %s', target) 

            if index == len(pp):
                break

            th_arr = ikine_prrp(pp2[index], 0.22, 0.2)
            # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
            th1 = th_arr[0]
            th2 = th_arr[1]

            angles = np.append(th2, upper)
            angles = np.append(angles, p2_ori)

            for i in range(len(mat_mani)):
                temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])
                Syncwrite_param(mat_mani[i], temp)
            Syncwrite_goal_position()   
            index = index + 1 
            time.sleep(0.05)

        time.sleep(3.0)

        index = 0
        while not rospy.is_shutdown(): # While loop 3 : Down
            rospy.loginfo("Phase 3 - Go down")
            if index == len(pp):
                break
            # print("up: ", up)
            
            th_arr = ikine_prrp(pp2[-1], 0.22, 0.2)
            # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
            th1 = th_arr[0]
            th2 = th_arr[1]

            angles = np.append(th2, down[index])
            angles = np.append(angles, p2_ori)  # p2 original position

            for i in range(len(mat_mani)):
                temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])
                Syncwrite_param(mat_mani[i], temp)
            Syncwrite_goal_position()   
            index = index + 1 
            time.sleep(0.05)

        index =0
        while not rospy.is_shutdown(): # While loop 4 : Up
            rospy.loginfo("Phase 4 - Move up")
            if index == len(pp):
                break
            # print("up: ", up)
            
            th_arr = ikine_prrp(pp2[-1], 0.22, 0.2)
            # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
            th1 = th_arr[0]
            th2 = th_arr[1]

            angles = np.append(th2, up[index])
            angles = np.append(angles, p2_ori)

            for i in range(len(mat_mani)):
                temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])
                Syncwrite_param(mat_mani[i], temp)
            Syncwrite_goal_position()   
            index = index + 1 
            time.sleep(0.05)

        index =0
        while not rospy.is_shutdown(): # While loop 5 : Move
            rospy.loginfo("Phase 5 - Move card to specific position")
            if index == len(pp):
                break
            # print("up: ", up)
            th_arr = ikine_prrp(target_fore[index], 0.22, 0.2)
            # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
            th1 = th_arr[0]
            th2 = th_arr[1]

            angles = np.append(th2, upper)
            angles = np.append(angles, p2_ori)

            for i in range(len(mat_mani)):
                temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])
                Syncwrite_param(mat_mani[i], temp)
            Syncwrite_goal_position()   
            index = index + 1 
            time.sleep(0.05)

        index =0
        while not rospy.is_shutdown(): # While loop 6 : Go Down 
            rospy.loginfo("Phase 6 - Go down")
            if index == len(pp):
                break
            # print("up: ", up)
            th_arr = ikine_prrp(target_fore[-1], 0.22, 0.2)
            # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
            th1 = th_arr[0]
            th2 = th_arr[1]

            angles = np.append(th2, down2[index])
            angles = np.append(angles, p2_ori)

            for i in range(len(mat_mani)):
                temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])
                Syncwrite_param(mat_mani[i], temp)
            Syncwrite_goal_position()   
            index = index + 1 
            time.sleep(0.05)

        index =0
        while not rospy.is_shutdown(): # While loop 7 : Detach card 
            rospy.loginfo("Phase 7 - Detach card")
            if index == len(pp):
                break
            # print("up: ", up)
            th_arr = ikine_prrp(target_fore[-1], 0.22, 0.2)
            # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
            th1 = th_arr[0]
            th2 = th_arr[1]

            angles = np.append(th2, down2[-1])
            angles = np.append(angles, p2_up[index])

            for i in range(len(mat_mani)):
                temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])
                Syncwrite_param(mat_mani[i], temp)
            Syncwrite_goal_position()   
            index = index + 1 
            time.sleep(0.05)

        index =0
        while not rospy.is_shutdown(): # While loop 8 : Move up manipulator 
            rospy.loginfo("Phase 8 - Move p2 back")
            if index == len(pp):
                break
            # print("up: ", up)
            th_arr = ikine_prrp(target_fore[-1], 0.22, 0.2)
            # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
            th1 = th_arr[0]
            th2 = th_arr[1]

            angles = np.append(th2, up2[index])
            angles = np.append(angles, p2_down[index])

            for i in range(len(mat_mani)):
                temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])
                Syncwrite_param(mat_mani[i], temp)
            Syncwrite_goal_position()   
            index = index + 1 
            time.sleep(0.05)

        index =0

        while not rospy.is_shutdown(): # While loop 9 : Go homing position 
            rospy.loginfo("Phase 9 - Go homing position")
            if index == len(pp):
                break
            # print("up: ", up)
            th_arr = ikine_prrp(target_back[index], 0.22, 0.2)
            # th_arr = ikine_prrp(pp2_h[index], 0.22, 0.2)
            th1 = th_arr[0]
            th2 = th_arr[1]

            angles = np.append(th2, upper)
            angles = np.append(angles, p2_ori)

            for i in range(len(mat_mani)):
                temp = get_rad_offset(angles[i], offset_arr[i], min_arr[i], max_arr[i])
                Syncwrite_param(mat_mani[i], temp)
            Syncwrite_goal_position()   
            index = index + 1 
            time.sleep(0.05)


    # Close port
    portHandler.closePort()
