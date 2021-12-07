import cv2
import numpy as np
import os  # To read image and find
import matplotlib.pyplot as plt

def position_to_pixel(K_mat, pose, z):
    """
    Transform position to pixel
    * Input:
                K_mat:  camera intrinsics
                pose:   desired position (given as x, y)
                z:      fixed camera
    * Output:
                pixel:  u,v piexel information
    """
    pose2 = np.hstack((pose, z))
    temp = np.reshape(pose2, (3,1))
    temp2 = K_mat@temp
    temp3 = temp2/z  # [u, v, 1]^T
    pixel = temp3[0:2]

    print("temp111: ", temp)
    print("inv111: ", K_mat)
    print("temp222: ", temp2)
    print("temp333: ", temp3)
    print("pixel11: ", pixel)

    return pixel

def pixel_to_position(K_mat, pixel, z):
    """
    Transform pixel to position
    * Input:
                K_mat:  camera intrinsics
                pixel:  current pixel (given as u, v)
                z:      fixed camera height
    * Output:
                pose:   x,y position
    """
    pixel2 = np.hstack((pixel, 1))
    temp = np.reshape(pixel2, (3,1))

    temp2 = np.linalg.inv(K_mat)@temp*z

    pose = temp2[0:2]
    return pose

def cam_to_robot(dst_cr):
    """
    Compute transformation matrix between camera and robot
    * Input:
                dst_cr:     distance between robot and camera
    * Output:
                trans_mat:  4x4 transformation matrix
    """
    rot1 = np.array([[-1, 0, 0, 0],[0, -1, 0, 0],
                    [0, 0, 1, 0],[0, 0, 0, 1]])
    trans1 = np.array([[1, 0, 0, 0],[0, 1, 0, -dst_cr],
                        [0, 0, 1, 0],[0, 0, 0, 1]])

    trans_mat = rot1@trans1

    return trans_mat

def pose_to_mat(pose):
    """
    Compute transformation from robot
    * Input:
                pose:       desired pose
    * Output:
                trans_mat:  transformed mat
    """
    x = pose[0]
    y = pose[1]

    trans_mat = np.array([[1, 0, 0, x], [0, 1, 0, y], 
                        [0, 0, 1, 0], [0, 0, 0, 1]])

    return trans_mat

def pixel_from_robot(trans, pixel, height, dist):
    """
    Compute robot position from pixel
    * Input:
                trans:          camera intrinsics
                pixel:          pixel given as u, v
                height:         fixed height (camera)
                dist:           distance between robot base and camera
    * Output:
                pos_robot:           position relative to robot's base
    """
    pos_cam = pixel_to_position(trans, pixel, height)
    
    pos_cam2 = pose_to_mat(pos_cam)
    trans = cam_to_robot(dist)
    trans_robot = trans@pos_cam2

    pos_robot22 = trans_robot[0:2, 3]
    pos1 = pos_robot22[0][0]
    pos2 = pos_robot22[1][0]
    pos_robot = [pos1, pos2]
    # print("pos_robot: ", pos_robot)
    return pos_robot


aa = np.load('camera_intrinsics/D.npy')
trans = np.load('camera_intrinsics/K.npy')

# print(trans)

pix = position_to_pixel(trans, [-0.1, -0.1], 0.35)
print("Pixel result: ", pix)

# pos = pixel_to_position(trans, [200, 480], 0.3)

xx = [0, 1920]
yy = [0, 1080]


posx = []
posy = []

posx_robot = []
posy_robot = []

hgt = 0.35
dist = 0.32

for i in xx:
    for j in yy:
        temp = pixel_to_position(trans, [i, j], hgt)
        temp = temp.reshape((2,1))
        pr = pixel_from_robot(trans, [i, j], height = hgt, dist = dist)

        # print(i)
        # print(j)
        # print(temp)
        # print(pr)


        posx.append(temp[0,0])
        posy.append(temp[1,0])

        posx_robot.append(pr[0])
        posy_robot.append(pr[1])



plt.scatter(posx,posy, c= 'red')

plt.scatter(posx_robot, posy_robot, c='blue' )
plt.show()


# trans_mat = cam_to_robot(0.3)

# pose_robot = pixel_from_robot(trans, [0, 0], height = 0.35, dist = 0.3)

# print("bb: ", trans)
# print("position_to_pixel: ", pix)
# print("pixel_to_position: ", pos)
# print("transform: ", trans)
# print("Trans_robot: ", trans_robot)
# print("pose_robot: ", pose_robot)