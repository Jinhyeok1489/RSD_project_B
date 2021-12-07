#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
import os  # To read image and find
import math as m

def preprocess_img(img, drawContour, drawMask, K_mat):
    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    kernel_size_row = 3
    kernel_size_col = 3
    kernel = np.ones((3, 3), np.uint8)
    img2 = cv2.dilate(img2, kernel, iterations = 15)


    mask = np.zeros_like(img2)
    # cv2.rectangle(mask, (100, 0), (520, 340), (255, 255, 255), -1)
    # cv2.rectangle(mask, (600, 200), (1320, 680), (255, 255, 255), -1)
    hh = 0.35
    start = [-0.1, -0.1]
    end = [0.1, 0.1]
    start_px = position_to_pixel(K_mat, start, hh)
    end_px = position_to_pixel(K_mat, end, hh) 
    # print(start_px[0,0])

    start_pt = (int(start_px[0,0]), int(start_px[1,0]))
    end_pt = (int(end_px[0,0]), int(end_px[1,0]))
    # print(start_pt)

    cv2.rectangle(img = mask, pt1 = start_pt, pt2 = end_pt, color = (255, 255, 255), thickness =-1)

    masked = cv2.bitwise_and(img2, mask)    

    # frame_g = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)
    # ret, img3 = cv2.threshold(masked, 225, 255, 0)
    ret, img3 = cv2.threshold(masked, 227, 255, 0)

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


if __name__ == '__main__':
    aa = np.load('camera_intrinsics/D.npy')
    trans = np.load('camera_intrinsics/K.npy')
 
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

        # edge1 = cv2.Canny(img2, 50, 200)

        # dst = cv2.cornerHarris(img2,2,3,0.04)
        # dst = cv2.dilate(dst, None)

        # img2[dst > 0.01*dst.max()] = [0, 0, 255]

        # cv2.imshow('binary', edge1)
        cv2.imshow('grayscale', img2)
        cv2.imshow('origin', imgOriginal)

        cv2.waitKey(1)