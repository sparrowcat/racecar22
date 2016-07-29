#http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_matcher/py_matcher.html

import cv2
import numpy as np
import time
from scipy import misc

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading

class Challenge:

	def __init__(self):
		self.thread_lock = threading.Lock()
		self.bridge = CvBridge()

		self.threshold = 40
		self.names = ["ari", "cat", "professor karaman", "racecar"]
		# Initiate SIFT detector
		self.surf = cv2.ORB()
		self.function = lambda d: d.distance < self.threshold
	
	def cbImage(self,image_msg):
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()

	def processImage(self,image_msg):
		if not self.thread_lock.acquire(False):
			return
		img2 = self.bridge.imgmsg_to_cv2(image_msg).cvtColor(img2, cv2.COLOR_BGR2GRAY)
		width, height = img2.shape
		#img2 = misc.imread("test6.jpg","L").astype(np.uint8)

		## find the keypoints and descriptors with SIFT - image
		kp2, des2 = self.surf.detectAndCompute(img2,None)

		flann = cv2.BFMatcher(cv2.NORM_HAMMING,crossCheck=True)

		# Initalize good counts, function
		counts = []

		for name in names:
		    img1 = misc.imread(name+".jpg","L").astype(np.uint8)
		    
		    # find the keypoints and descriptors with SIFT - reference
		    kp1, des1 = surf.detectAndCompute(img1,None)

		    matches = flann.match(des1,des2)
		    bools = map(self.function,matches)
		    count = sum(bools)
		    counts.append(count)

		#find max count
		maxi = max(counts)

		#find index of max count
		ind = counts.index(maxi)
		    
		#get picture name of the ind
		chosen = names[ind]

		#put string on image
		img2 = cv2.putText(img2, chosen, (height, 0), cv2.FONT_HERSHEY_PLAIN, 10)

		#save file
		cv2.imwrite("/home/racecar/challenge_photos/"+chosen+".jpg",img2)

def main():
	rospy.init_node("challenge")
	chal = Challenge()
	sub = rospy.Subscriber("/challenge_images", Image, chal.cbImage)
	rospy.spin()

if __name__ == "__main__":
	main()
