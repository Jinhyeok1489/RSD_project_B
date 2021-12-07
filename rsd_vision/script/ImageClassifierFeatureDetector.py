#! /usr/bin/env python3

import cv2
import numpy as np
import os  # To read image and find
import math as m

"""
Main reference
1. https://www.youtube.com/watch?v=nnH55-zD38I&t=930s, Feature Detection and Matching + Image Classifier Project
2. https://www.youtube.com/watch?v=m-QPjO-2IkA, OpenCV-Playing-Card-Detector

"""

def findDes(images):
    desList = []
    for img in images:

        kp, des = detector.detectAndCompute(img, None)
        print("kp: ", len(kp))
        desList.append(des)
    return desList

def findID(img, desList, threshold = 0):
    # kp2, des2 = orb.detectAndCompute(img, None)

    kp2, des2 = detector.detectAndCompute(img, None)
    bf = cv2.BFMatcher()
    matchList = []
    finalVal = -1
    try:
        # print(desList)
        for des in desList:
            matches = bf.knnMatch(des, des2, k=2)
            # print("Try matching...")
            good = []
            for m, n in matches:
                if m.distance < 0.6 * n.distance:
                    good.append([m])
            # print("Matched and append... ")
            # print("Good: ", good)
            # print(matches)
            matchList.append(len(good))
        print(matchList)
    except:
        print("passed")
        pass
    # Check if matrix is empty
    if len(matchList) != 0:
        if max(matchList) > threshold:
            finalVal = matchList.index(max(matchList))
    print(finalVal)
    return finalVal

def solving_vertex(pts, K_mat):
    """
    Compute points order for projection
    *INPUT:
            pts: points
    *OUTPUT:
            points:     ordered points
            ceneter:    center of point
    """
    points = np.zeros((4,2), dtype= "uint32") #x,y쌍이 4개 쌍이기 때문
    s = pts.sum(axis = 1)
    points[0] = pts[np.argmin(s)] # left upper
    points[3] = pts[np.argmax(s)] # right below
    #원점 (0,0)은 맨 왼쪽 상단에 있으므로, y-x의 값이 가장 작으면 우상의 꼭짓점 / y-x의 값이 가장 크면 좌하의 꼭짓점
    diff = np.diff(pts, axis = 1)
    points[2] = pts[np.argmin(diff)] # right upper
    points[1] = pts[np.argmax(diff)] # left below

    pos0 = pixel_to_position(K_mat, points[0], 0.35) ### Tuning parameter z =0.35
    pos1 = pixel_to_position(K_mat, points[1], 0.35)
    pos2 = pixel_to_position(K_mat, points[2], 0.35)
    pos3 = pixel_to_position(K_mat, points[3], 0.35)

    garo = m.sqrt((pos0[0,0]-pos2[0,0])**2+(pos0[1,0]-pos2[1,0])**2)
    # garo = norm(abs(pos0-pos2))
    sero = m.sqrt((pos0[0,0]-pos1[0,0])**2+(pos0[1,0]-pos1[1,0])**2)

    if sero < garo:
        temp = np.zeros((4,2), dtype = "uint32")
        if points[0,0] > points[1,0]: # Right rotated, large angle
            temp[0] = points[2]
            temp[1] = points[0]
            temp[2] = points[3]
            temp[3] = points[1]

            print("case1")
            print("Type: ", type(temp))

            points = temp
        else: # Left rotated, large angle
            temp[0] = points[1]
            temp[1] = points[3]
            temp[2] = points[0]
            temp[3] = points[2]

            print("case2")
            print("Type: ", type(temp))


            points = temp

    else:
        if points[2,0] > points[3,0]: # Right rotated, small angle
            points = points
        else: # Left rotated, small angle
            points = points



        # points = temp        
        # temp[0] = points[1]
        # temp[1] = points[3]
        # temp[2] = points[2]
        # temp[3] = points[0]


        # points = temp

    cv2.circle(imgOriginal, points[0], 10, (0, 0, 255), thickness = 3)
    cv2.circle(imgOriginal, points[1], 10, (0, 255, 0), thickness = 3)
    cv2.circle(imgOriginal, points[2], 10, (255, 0, 0), thickness = 3)
    cv2.circle(imgOriginal, points[3], 10, (255, 255, 0), thickness = 3)


    center = (points[0]+points[1]+points[2]+points[3])/4
    center = (int(round(center[0])), int(round(center[1])))

    return points, center

def preprocess_img(img, drawContour, drawMask, K_mat):
    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    mask = np.zeros_like(img2)
    # cv2.rectangle(mask, (100, 0), (520, 340), (255, 255, 255), -1)
    # cv2.rectangle(mask, (600, 200), (1320, 680), (255, 255, 255), -1)
    hh = 0.35
    start = [-0.1, -0.1]
    end = [0.1, 0.1]
    start_px = position_to_pixel(K_mat, start, hh)
    end_px = position_to_pixel(K_mat, end, hh) 
    print(start_px[0,0])

    start_pt = (int(start_px[0,0]), int(start_px[1,0]))
    end_pt = (int(end_px[0,0]), int(end_px[1,0]))
    print(start_pt)

    cv2.rectangle(img = mask, pt1 = start_pt, pt2 = end_pt, color = (255, 255, 255), thickness =-1)

    masked = cv2.bitwise_and(img2, mask)    

    # frame_g = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)
    ret, img3 = cv2.threshold(masked, 220, 255, 0)
    if drawContour == True:
        contours, hierarchy = cv2.findContours(img3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            cv2.drawContours(img, [cnt], 0, (255, 0, 0), 3)

    if drawMask == True:
        cv2.rectangle(img = img, pt1 = start_pt, pt2 = end_pt, color = (0, 0, 255), thickness = 3)
        cv2.putText(img, 'Workspace', start_pt, cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255),5)


    # img3:     thresholded img
    # img2:     grayscale img
    # img:      original img
    return img3, img2, img

def extract_card(img, img2, K_mat):
    """
    * Input:
            img: binary_img
            img2: grayscale_img
    """
    result4 = cv2.resize(img2, dsize=(360,480), interpolation = cv2.INTER_AREA)
    # pattern_img = result[0:80, 0:60]
    center =(0,0)
    isContour = False
    try:
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print("len of contours: ", len(contours))
        # aaa = np.zeros((len(contours), 4, 2))
        # print(contours)
        ctr = contours[0]
        isContour = True

        # ctr = ctr_list[0, :, :]
        epsilon = cv2.arcLength(ctr, True)*0.02
        approx = cv2.approxPolyDP(ctr, epsilon, True)

        edge = approx.reshape(4,2)
        # print("edge: ", edge)
        pts, center = solving_vertex(edge, K_mat)
        edge_np = np.array(pts, dtype = np.float32)
        # print("edge_np: ", edge_np)
        dst_np = np.array([[0,0], [0, 480], [360, 0], [360,480]], dtype = np.float32)
        
        M = cv2.getPerspectiveTransform(edge_np, dst_np)
        # result4 = cv2.warpPerspective(img2, M = M, dsize = (360, 480))
        result4 = cv2.warpPerspective(img2, M = M, dsize = (360, 480))        
        kernel = np.ones((3,3), np.uint8)
        # result4 = cv2.erode(result, kernel, iterations = 1)
        # result2 = cv2.rectangle(result, (0, 20), (60, 90), (0, 0, 255), 3)
        # result2 = cv2.rectangle(result2, (0, 90), (50, 140), (0, 0, 255), 3)

        # result2 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        # ret, result3 = cv2.threshold(result2, 200, 255, 0), pattern_img4
        # result3 = cv2.erode(result3, kernel, iterations = 3)
        # result4 = cv2.bitwise_not(result)

        # mask2 = np.zeros((200, 350))
        # cv2.rectangle(mask, (100, 0), (520, 340), (255, 255, 255), -1)
        # cv2.rectangle(mask, (600, 200), (1320, 680), (255, 255, 255), -1)

        # Cut images
        # pattern_img = result4[0:80, 0:60].copy()
    except:
        # result4 = np.zeros((480, 360))
        pass
    return result4, center, isContour

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
    temp2 = K_mat@temp/z  # [u, v, 1]^T
    pixel = temp2[0:2]

    pixel[0] = int(m.floor(pixel[0]))
    pixel[1] = int(m.floor(pixel[1]))

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
    # print("temp111: ", temp)
    # print("inv111: ", np.linalg.inv(K_mat))
    # print("z111: ", z)
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
    return pos_robot


path = 'Card_Imgs/Cards'
# orb = cv2.ORB_create(nfeatures = 1000)
detector = cv2.xfeatures2d.SIFT_create()
aa = np.load('camera_intrinsics/D.npy')
trans = np.load('camera_intrinsics/K.npy')


#### Import Images
images = []
classNames = []

myList = os.listdir(path) # Read files in images
# print(myList)
# print('Total classes detected ', len(myList))

for cl in myList:
    imgCur = cv2.imread(f'{path}/{cl}', 0) # Import image as greyscale
    images.append(imgCur)
    classNames.append(os.path.splitext(cl)[0])
print(classNames)


desList = findDes(images)
print(len(desList))

cap = cv2.VideoCapture(2)
cap.set(15, 5)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(3, 1920)
cap.set(4, 1080)

height = 0.35
dist = 0.32

#### While loop
while True:
    success, img2 = cap.read()
    imgOriginal = img2.copy()
    img3, img2, imgOriginal = preprocess_img(img2, drawContour = True, drawMask = True, K_mat = trans)


    # cv2.imshow('simg2', img3)
    result, center, isContour = extract_card(img3, img2, trans)
    if isContour == True:
        cv2.circle(imgOriginal, center, 20, (0, 0, 255), thickness = 3)
        robot_pose = pixel_from_robot(trans, center, height, dist)

        print('Robot pose: ', robot_pose)

        cv2.imshow('Card', result)
        # cv2.imshow('car_pattern', img_card)

        id = findID(result, desList, 0)

        if id != -1:
            print(classNames[id])
            cv2.putText(imgOriginal, classNames[id], (960, 300), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255),5)

        cv2.imshow('OriginalImage', imgOriginal)
        # Find descriptor of webcam image


    cv2.waitKey(1)

