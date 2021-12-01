#! /usr/bin/env python3
import rospy
import math
import numpy as np
import cv2

cap = cv2.VideoCapture(0)
cap.set(15, 5)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(3, 1920)
cap.set(4, 1080)

def solving_vertex(pts):
    points = np.zeros((4,2), dtype= "uint32") #x,y쌍이 4개 쌍이기 때문
    s = pts.sum(axis = 1)
    points[0] = pts[np.argmin(s)] #좌상
    points[3] = pts[np.argmax(s)] #우하
    #원점 (0,0)은 맨 왼쪽 상단에 있으므로, y-x의 값이 가장 작으면 우상의 꼭짓점 / y-x의 값이 가장 크면 좌하의 꼭짓점
    diff = np.diff(pts, axis = 1)
    points[2] = pts[np.argmin(diff)] #우상
    points[1] = pts[np.argmax(diff)] #좌하
    # src.append(points[0])
    # src.append(points[1])
    # src.append(points[2])
    # src.append(points[3])

    return points


img1 = cv2.imread('/home/jinhyeok/catkin_ws/src/rsd_project/RSD_project_B/rsd_vision/script/Card_Imgs/Four.jpg')
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)

while(True):
    ret, frame = cap.read()

    # print(frame.shape)
    mask = np.zeros_like(frame)
    # cv2.rectangle(mask, (100, 0), (520, 340), (255, 255, 255), -1)
    # cv2.rectangle(mask, (600, 200), (1320, 680), (255, 255, 255), -1)
    cv2.rectangle(mask, (300, 200), (1620, 980), (255, 255, 255), -1)

    masked = cv2.bitwise_and(frame, mask)

    # # Transform image into HSV space and histogram equalization on V channel
    # # https://pythonq.com/so/opencv/358822
    # hsv_frame = cv2.cvtColor(masked, cv2.COLOR_RGB2HSV)
    # h, s, v = cv2.split(hsv_frame)

    # clahe = cv2.createCLAHE(clipLimit = 2.0 , tileGridSize=(8,8))
    # v = clahe.apply(v)

    # hsv_frame = cv2.merge([h, s, v])
    # hsv_frame = cv2.cvtColor(hsv_frame, cv2.COLOR_HSV2RGB)

    # cv2.imshow('hsv_filtered', hsv_frame)
    cv2.imshow('mask', mask)

    frame_g = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)
    ret, frame_b = cv2.threshold(frame_g, 200, 255, 0)
    contours, hierarchy = cv2.findContours(frame_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    print('length of contours: ', len(contours))
    # cv2.imshow('contour0', contours[0])

    # Approximate contours

    for contour in contours:
        epsilon = cv2.arcLength(contour, True)*0.02
        approx_poly = cv2.approxPolyDP(contour, epsilon, True)
        # for approx in approx_poly:
        #     cv2.circle(frame, tuple(approx[0]), 10, (0, 255, 0), -1)
            # print(approx)
    contours = sorted(contours, key=cv2.contourArea, reverse= True)[:5]

    aaa = np.zeros((len(contours), 4, 2))
    # print(aaa[0, :, :])

    i = 0 # Temporal variable
    print("length of contours: ", len(contours))
    for contour in contours:
    
        epsilon = cv2.arcLength(contour, True)*0.02
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 4: # rectangle is drawble
            aaa[i, :, :] = approx.reshape(4,2)
            size = len(aaa)
            print("approx: ", approx.reshape(4,2))
            print("aaa: ", aaa)
        i = i+1
    
    
    
        # print(len(approx_poly))
        axis = np.zeros(4)
        # print("Shape: ", approx_poly.shape)
        # Compute axis 
        # for k in range(3):
        #     axis[k] = (approx_poly[k][0][1]-approx_poly[k+1][0][1])/(approx_poly[k][0][0]-approx_poly[k+1][0][0])
    
    # print("axis: ", axis)


        # Draw line
        # c_size = len(contours)
        # print(contour[0][0])
        # if c_size > 0:
        #     cv2.line(frame, tuple(contour[0][0]), tuple(contour[c_size][0]), (0,0, 255), 15)
        #     for j in range(c_size-1):
        #         color = list(np.random.random(size=3)*255)
        #         cv2.line(frame, tuple(contour[j][0]), tuple(contour[j+1][0]), color, 3)
    
    resize_img = cv2.resize(frame, dsize=(360,480), interpolation = cv2.INTER_AREA)
    # for i in range(len(aaa)):
    #     edge = aaa[i, :, :].reshape(4,2)
    #     pts = solving_vertex(edge)
    #     edge_np = np.array(pts, dtype = np.float32)
    #     print("Edges: ", edge_np)
    #     dst_np = np.array([[0,0], [0, 480], [360, 0], [360,480]], dtype = np.float32)
        
    #     print("Move: ", dst_np)
    #     M = cv2.getPerspectiveTransform(edge_np, dst_np)
    #     result = cv2.warpPerspective(frame, M = M, dsize = (360, 480))
    #     kernel = np.ones((3,3), np.uint8)
    #     result = cv2.erode(result, kernel, iterations = 1)
    #     # result2 = cv2.rectangle(result, (0, 20), (60, 90), (0, 0, 255), 3)
    #     # result2 = cv2.rectangle(result2, (0, 90), (50, 140), (0, 0, 255), 3)

    #     result2 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    #     ret, result3 = cv2.threshold(result2, 200, 255, 0)
    #     # result3 = cv2.erode(result3, kernel, iterations = 3)
    #     result4 = cv2.bitwise_not(result3)

    #     mask2 = np.zeros((200, 350))
    #     # cv2.rectangle(mask, (100, 0), (520, 340), (255, 255, 255), -1)
    #     # cv2.rectangle(mask, (600, 200), (1320, 680), (255, 255, 255), -1)
    #     cv2.rectangle(result4, (50, 30), (350, 450), (0, 0, 0), -1)

    #     # masked = cv2.bitwise_and(frame, mask)

    #     cv2.imshow("cut"+str(i), result4)

    #     detector = cv2.xfeatures2d.SIFT_create()

    #     kp1, desc1 = detector.detectAndCompute(result4, None)
    #     kp2, desc2 = detector.detectAndCompute(gray1, None)

    #     FLANN_INDEX_KDTREE = 1
    #     index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees=5)
    #     search_params = dict(checks = 50)

    #     matcher = cv2.FlannBasedMatcher(index_params, search_params)
    #     matches = matcher.match(desc1, desc2)

    #     res = cv2.drawMatches(result4, kp1, gray1, kp2, matches, None, 
    #                         flags = cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
    #     cv2.imshow('Flann + SIFT', res)

    edge = aaa[0, :, :].reshape(4,2)
    pts = solving_vertex(edge)
    edge_np = np.array(pts, dtype = np.float32)
    print("Edges: ", edge_np)
    dst_np = np.array([[0,0], [0, 480], [360, 0], [360,480]], dtype = np.float32)
    
    print("Move: ", dst_np)
    M = cv2.getPerspectiveTransform(edge_np, dst_np)
    result = cv2.warpPerspective(frame, M = M, dsize = (360, 480))
    kernel = np.ones((3,3), np.uint8)
    result = cv2.erode(result, kernel, iterations = 1)
    # result2 = cv2.rectangle(result, (0, 20), (60, 90), (0, 0, 255), 3)
    # result2 = cv2.rectangle(result2, (0, 90), (50, 140), (0, 0, 255), 3)

    result2 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    ret, result3 = cv2.threshold(result2, 200, 255, 0)
    # result3 = cv2.erode(result3, kernel, iterations = 3)
    result4 = cv2.bitwise_not(result3)

    mask2 = np.zeros((200, 350))
    # cv2.rectangle(mask, (100, 0), (520, 340), (255, 255, 255), -1)
    # cv2.rectangle(mask, (600, 200), (1320, 680), (255, 255, 255), -1)
    cv2.rectangle(result4, (50, 30), (320, 450), (0, 0, 0), -1)

    # masked = cv2.bitwise_and(frame, mask)

    cv2.imshow("cut"+str(0), result4)

    detector = cv2.xfeatures2d.SIFT_create()


    kp1, desc1 = detector.detectAndCompute(result4, None)
    kp2, desc2 = detector.detectAndCompute(gray1, None)

    FLANN_INDEX_KDTREE = 1

    # orb = cv2.ORB_create()

    # kp1, des1 = orb.detectAndCompute(result4, None)
    # kp2, des2 = orb.detectAndCompute(gray1, None)

    # bf = cv2.BFMatcher()
    # matches = bf.knnMatch(des1, des2, k=2)

    # good = []


    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks = 50)

    matcher = cv2.FlannBasedMatcher(index_params, search_params)
    matches = matcher.match(desc1, desc2)

    # good = []
    # for m, n in matches:
    #     if m.distance < 0.75*n.distance:
    #         good.append([m])

    # print("matches: ", good)
    # res = cv2.drawMatchesKnn(result4, kp1, gray1, kp2, matches, None, 
    #                     flags = cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)

    res = cv2.drawMatches(result4, kp1, gray1, kp2, matches, None, \
                flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
    cv2.imshow('Flann + SIFT', res)


    for cnt in contours:
        cv2.drawContours(frame, [cnt], 0, (255, 0, 0), 3)
    
    

    # cv2.imshow('binary', frame_b)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        print(area)

    # biggest_contour = max(contours, key= cv2.contourArea)
    # cv2.drawContours(frame, biggest_contour, -1, (0, 255,0), 4)

    cv2.imshow("result", frame)


    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()