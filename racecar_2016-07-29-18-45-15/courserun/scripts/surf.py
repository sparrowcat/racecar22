#http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_matcher/py_matcher.html
#if the memory is too bad, then may have to split up images
#can have threshold of count 20
#hopefully it has a better processor
#will need to replace print statements with appropriate topic

import cv2
import numpy as np
#from matplotlib import pyplot as plt
import time
from scipy import misc

START = time.time()

threshold = 40#20
#boundary = 20
names = ["ari", "cat", "professor karaman", "racecar"]
img2 = misc.imread("test6.jpg","L").astype(np.uint8)

# Initiate SIFT detector
surf = cv2.ORB()#cv2.xfeatures2d.SURF_create(400)#sift = cv2.SIFT()

## find the keypoints and descriptors with SIFT - image
kp2, des2 = surf.detectAndCompute(img2,None)#sift

# FLANN parameters
#FLANN_INDEX_KDTREE = 0
#index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
#search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)#cv2.FlannBasedMatcher(index_params,search_params)
20
# Initalize good counts, function
counts = []
function = lambda d: d.distance < threshold# lambda (m, n): m.distance < 0.7*n.distance

for name in names:
    img1 = misc.imread(name+".jpg","L").astype(np.uint8)
    
    # find the keypoints and descriptors with SIFT - reference
    kp1, des1 = surf.detectAndCompute(img1,None)#sift

    matches = flann.match(des1,des2)#flann.knnMatch(des1,des2,k=2)
    #matches = sorted(matches,key = lambda x: x.distance)
    bools = map(function,matches)
    count = sum(bools)#count = len(matches)
    print(count)
    counts.append(count)

#find max count
maxi = max(counts)

#check if max count is over threshold
'''if maxi < boundary:
    print("None")
else:'''
#find index of max count
ind = counts.index(maxi)
    
#get picture name of the ind
print(names[ind])

END = time.time()
print(END - START)

