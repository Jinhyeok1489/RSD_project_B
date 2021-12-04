import cv2
import numpy as np
import os  # To read image and find


img = cv2.imread("Card_Imgs/Dummies/Spade_Q.png", cv2.IMREAD_COLOR)

img2 = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

# ret, img3 = cv2.threshold(img2, 220, 255, 0)
cv2.imshow('Transfor_img', img2)
cv2.imwrite('Card_Imgs/Cards/Spade_Q.png', img2)
# cv2.imwrite('Card_Imgs/Cards/joker.png',img3)

cv2.waitKey(0)
cv2.destroyAllWindows()