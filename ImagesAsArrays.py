import numpy as np
import matplotlib.pyplot as plt
import cv2
rgb = cv2.imread('BigSquares.png')
# flatten the pixels into a 1-D array

# [Row, Colum, RBG]
r = rgb[:,:,2]
plt.imshow(r)
plt.show()
