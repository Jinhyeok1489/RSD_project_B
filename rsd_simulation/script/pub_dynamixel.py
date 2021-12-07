#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math as m
import time
import pandas as pd
import numpy as np

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

    # print("Angle: ", rad_value*180/m.pi)

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
DEVICENAME = '/dev/ttyUSB0'

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
mat_mani = np.array([DXL_R1_ID, DXL_R2_ID])
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


mat_mani = np.array([DXL_R1_ID, DXL_R2_ID])
for i in range(len(mat_mani)):
    Enable_torque(mat_mani[i])

init_time = time.time()

l1 = 0.22 # Firt link length
l2 = 0.20 # second link length

pos = np.array([0.27, 0.0])

print(ikine_prrp(pos, l1, l2)[0])

angs = ikine_prrp(pos, l1, l2)
ang1 = angs[0]
ang2 = angs[1]

min_arr = np.array([20, -80])*m.pi/180
max_arr = np.array([250, 150])*m.pi/180

offset_arr = np.array([135, 35])*m.pi/180


kine = fkine_prrp(0.22, 0.2, ang1[0], ang1[1])
print("Kine: ",kine[0:2, 3])

home = np.array([0, 0])
desired_arr = ang2

th1 = np.linspace(home[0], desired_arr[0], num = 50, endpoint = True)
th2 = np.linspace(home[1], desired_arr[1], num = 50, endpoint = True)

th1_1 = np.linspace(desired_arr[0], home[0], num = 50, endpoint = True)
th2_1 = np.linspace(desired_arr[1], home[1], num = 50, endpoint = True)

th1_2 = np.linspace(m.pi/2, 0, num = 50, endpoint = True)
th2_2 = np.linspace(m.pi/2, 0, num = 50, endpoint = True)

# get_rad_offset(desired_arr[0], offset_arr[0], min_arr[0], max_arr[0])

print("th1: ", th1)

print("th2: ", th2)
# get_rad_offset(desired_arr[1], offset_arr[1], min_arr[1], max_arr[1])

index = 0
while(1):
    if index == 49:
        break

    th_arr = np.array([th1[index], th2[index]])
    # th_arr = np.array([th1_1[index], th2_1[index]])
    # th_arr = np.array([th1_1[index], th2_1[index]])
    # print("th_arr: ", th_arr)
    for i in range(len(mat_mani)):
        # temp = get_desired_rad(ang1[i])
        temp = get_rad_offset(th_arr[i], offset_arr[i], min_arr[i], max_arr[i])
        # print("th_arr[0]: ", th_arr[i])
        # print("th_arr[1]: ", th_arr[i])
        # temp = get_desired_rad(0)
        Syncwrite_param(mat_mani[i], temp)
    Syncwrite_goal_position()   
    index = index + 1 
    time.sleep(0.05)
    
# mat_mani = np.array([DXL_R1_ID, DXL_R2_ID ,DXL_P1_ID, DXL_P2_ID])
# mat_mani = np.array([DXL_R1_ID, DXL_R2_ID])
# for i in range(len(mat_mani)):
#     Disable_torque(mat_mani[i])



# Close port
portHandler.closePort()
