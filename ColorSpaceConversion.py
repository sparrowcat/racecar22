import numpy as np
import matplotlib.pyplot as plt
import cv2

rgb = cv2.imread('frame0070.jpg')
hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
h = hsv[:,:,0]
#hist = plt.hist(h, 100)
plt.imshow(h)
plt.show()
