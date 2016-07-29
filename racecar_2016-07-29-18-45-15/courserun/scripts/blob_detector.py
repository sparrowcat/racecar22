#!/usr/bin/env python
	
import cv2
import rospy
from courserun.msg import BlobDetections####need to make color field a STRING
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image	
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np
	
global colors, color_map
colors = ["red", "green", "yellow", "blue", "pink"]
color_map = {"red": (np.array([0,190,100]), np.array([15,255,255])),\
	"green": (np.array([35,100,200]), np.array([80,255,255])),\
	"yellow": (np.array([20,200,150]), np.array([30,255,255])),\
	"blue": (np.array([90,100,100]), np.array ([150,255,255])),\
	"pink": (np.array([160, 120, 120]), np.array ([180, 255, 255]))}
	
class blob_detector:
	def __init__(self):
		self.node_name = "blob_detector"
		self.thread_lock = threading.Lock()

		self.pub_image = rospy.Publisher("/blob_image", Image, queue_size=1)
		self.pub_msg = rospy.Publisher("/blob_detection", BlobDetections, queue_size=1)
		self.bridge = CvBridge()
	
		rospy.loginfo("[%s] Initialized." %(self.node_name))
	
		self.message = BlobDetections()
	
	def cbImage(self,image_msg):
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
	
	def processImage(self, image_msg):
		
		if not self.thread_lock.acquire(False):
			return
		cropped = None
		image_cv = self.bridge.imgmsg_to_cv2(image_msg)
		width, height, cha = image_cv.shape
		lowthresarea = 2500

		# Convert BGR to HSV
		hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

		Arr = []
		BoxArr = []
		cent = Point()

		for c in colors:
				
			#Create mask
			(lower, upper) = color_map[c]
			mask = cv2.inRange(hsv, lower, upper)
	
			#Create contours
			contours = cv2.findContours(mask, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE)[0]

				
			for co in contours:
				rect = cv2.minAreaRect(co)
			    	(w,h) = rect[1]
			    	area = w*h
			    	if area < lowthresarea: continue
				#angle = rect[2]
				#if angle < -30 and angle > -60: continue#too angled
				#p = max(w/h,h/w)
				#if p > 2: continue#the rectangle probably does not enclose the paper
				center = rect[0]
				(y,x) = center
				cent.x = x/width
				cent.y = y/height
				box = cv2.cv.BoxPoints(rect)
				box = np.int0(box)
				Arr.append((Float64(area),cent,String(c)))
				BoxArr.append(box)#does not matter the order
		Arr.sort()
        	Arr = Arr[::-1]
	    	
        	self.message.sizes = [o[0] for o in Arr]
        	self.message.locations = [o[1] for o in Arr]
        	self.message.colors = [o[2] for o in Arr]
	
        	cv2.drawContours(image_cv,BoxArr,0,(255,255,255),4)
	
        	try:
        		self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
        		self.pub_msg.publish(self.message)
        	except CvBridgeError as e:
        		print(e)
        	self.thread_lock.release()
	
	
if __name__=="__main__":
	rospy.init_node('blob_detector')
	e = blob_detector()
	sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, e.cbImage, queue_size=1)
	rospy.spin()
