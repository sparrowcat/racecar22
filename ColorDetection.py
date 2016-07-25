import cv2
import numpy as np

cap = cv2.imread('frame0070.jpg')

while(1):

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower = np.array([115,0,0])
    upper = np.array([160,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower, upper)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cap,cap, mask= mask)

    cv2.imshow('frame',cap)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
