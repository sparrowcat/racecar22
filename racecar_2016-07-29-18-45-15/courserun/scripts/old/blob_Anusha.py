#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge, CvBridgeError
import threading



class Echo:
    def __init__(self, color):
        rospy.init_node("blobDetForDrive")
	self.node_name = "Echo"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage)
        self.pub_pos = rospy.Publisher("~position",\
                Int32, queue_size=1)
	self.pub_image= rospy.Publisher("~image",\
		Image, queue_size=1)
	self.shutdown_sub = rospy.Subscriber("/shutdown", \
			String, self.shutdown)	
        self.color = color
	if self.color == "red":
		self.upper = np.array([0,0,0])
		self.lower = np.array([0,0,0])
	else:
		self.upper = np.array([0,0,0])
		self.lower = np.array([0,0,0])
	self.bridge = CvBridge()


        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
	rospy.loginfo("cbImage")        
	thread = threading.Thread(target=self.processImage,args=(image_msg,))
	thread.setDaemon(True)
        thread.start()

    def contour(self, image_cv):
	hsv = cv2.cvtColor(image_cv, cv2.cv.CV_BGR2HSV)	

	binMask = cv2.inRange(hsv, np.array([65,100,100], np.uint8), np.array([77,255,255],np.uint8))	
	masked = cv2.bitwise_and(hsv,hsv,mask=binMask)
	
	contours, _ = cv2.findContours(binMask,cv2.cv.CV_RETR_EXTERNAL,cv2.cv.CV_CHAIN_APPROX_SIMPLE)
	print len(contours)
	for c in contours:
		#print "contour loop"
		x,y,w,h = cv2.boundingRect(c)
		if w*h < 2000:
			#print "cont'd"
			continue
		elif w*h > 1e5:
			
			print "big"
			self.pub_pos.publish(-1337)
			continue
		msg = x + 0.5 * w
		cv2.rectangle(image_cv, (x,y),(x+w, y+h),(0,255,0),2)
		print msg
		self.pub_pos.publish(msg)
	#cv2.imshow("masked",image_cv)
	#cv2.waitKey(0)
	#cv2.destroyAllWindows()
	self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, encoding="bgr8"))
	
	
    def processImage(self, image_msg):
	if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        try:
		print "tried"            
		self.contour(image_cv)
	    
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()

    def shutdown(self, msg):
		if msg == "blobDetector shutdown": 
			rospy.loginfo("blobDetector shutting down")
			rospy.signal_shutdown("bd finished with ops")
		else:
			return

if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

