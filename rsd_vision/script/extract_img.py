import cv2
import numpy as np
import os  # To read image and find

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
    # print("len kp2: ", len(kp2))
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
                if m.distance < 0.5 * n.distance:
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

def preprocess_img(img, drawContour):
    img2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    mask = np.zeros_like(img2)
    # cv2.rectangle(mask, (100, 0), (520, 340), (255, 255, 255), -1)
    # cv2.rectangle(mask, (600, 200), (1320, 680), (255, 255, 255), -1)
    cv2.rectangle(mask, (400, 300), (1520, 780), (255, 255, 255), -1)

    masked = cv2.bitwise_and(img2, mask)    

    # frame_g = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)
    ret, img3 = cv2.threshold(masked, 220, 255, 0)
    if drawContour == True:
        contours, hierarchy = cv2.findContours(img3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            cv2.drawContours(img, [cnt], 0, (255, 0, 0), 3)

    return img3, img

def extract_card(img):
    """
    * Input:
            img: binary_img
    """
    result4 = cv2.resize(img, dsize=(360,480), interpolation = cv2.INTER_AREA)
    # pattern_img = result[0:80, 0:60]
    try:
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print("len of contours: ", len(contours))
        # aaa = np.zeros((len(contours), 4, 2))
        # print(contours)
        ctr = contours[0]
        # print(len(contours))
        # print(ctr)
        # ctr = ctr_list[0, :, :]
        epsilon = cv2.arcLength(ctr, True)*0.02
        approx = cv2.approxPolyDP(ctr, epsilon, True)

        print(len(approx))
        edge = approx.reshape(4,2)
        # print(edge)
        pts = solving_vertex(edge)
        edge_np = np.array(pts, dtype = np.float32)

        dst_np = np.array([[0,0], [0, 480], [360, 0], [360,480]], dtype = np.float32)
        
        M = cv2.getPerspectiveTransform(edge_np, dst_np)
        result = cv2.warpPerspective(img, M = M, dsize = (360, 480))
        kernel = np.ones((3,3), np.uint8)
        result = cv2.erode(result, kernel, iterations = 1)
        # result2 = cv2.rectangle(result, (0, 20), (60, 90), (0, 0, 255), 3)
        # result2 = cv2.rectangle(result2, (0, 90), (50, 140), (0, 0, 255), 3)

        # result2 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        # ret, result3 = cv2.threshold(result2, 200, 255, 0), pattern_img4
        # result3 = cv2.erode(result3, kernel, iterations = 3)
        result4 = cv2.bitwise_not(result)

        # mask2 = np.zeros((200, 350))
        # cv2.rectangle(mask, (100, 0), (520, 340), (255, 255, 255), -1)
        # cv2.rectangle(mask, (600, 200), (1320, 680), (255, 255, 255), -1)

        # Cut images
        # pattern_img = result4[0:80, 0:60].copy()
    except:
        # result4 = np.zeros((480, 360))
        pass
    return result4

path = 'Card_Imgs/Cards'
# orb = cv2.ORB_create(nfeatures = 1000)
detector = cv2.xfeatures2d.SIFT_create()

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

cap = cv2.VideoCapture(0)
cap.set(15, 5)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(3, 1920)
cap.set(4, 1080)

#### While loop
while True:
    success, img2 = cap.read()
    imgOriginal = img2.copy()
    img3, img2 = preprocess_img(img2, drawContour = True)


    cv2.imshow('simg2', img2)
    result = extract_card(img3)
    # print(img_card)
    cv2.imshow('Card', result)

    cv2.waitKey(1)

