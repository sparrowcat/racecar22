#!/usr/bin/env python

#image is 1280X720
import cv2
import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np

Min_Size = 0 #Always set to 0
class blob_detector:
    def __init__(self):
        self.node_name = "blob_detector"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~blob_detection",\
                Image, queue_size=1)
	self.pub_error = rospy.Publisher("imagepos_error", Float32, queue_size=1)
	self.pub_size = rospy.Publisher("image_size", Float32, queue_size=1)
        self.bridge = CvBridge()
#        self.button = rospy.Publisher('vesc/joy', Joy)
	self.pub_color = rospy.Publisher("colorRed", Bool)

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()


    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

    # Convert BGR to HSV
        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
        gr_lower = np.array([50,100,100])
        gr_upper = np.array([70,255,255])
        rd_lower = np.array([0, 100, 100])
        rd_upper = np.array([12, 255, 255])#12

    # Threshold the HSV image to get only blue colors
        mask_gr = cv2.inRange(hsv, gr_lower, gr_upper)
        mask_rd = cv2.inRange(hsv, rd_lower, rd_upper)
        #mask_gr_ba = cv2.bitwise_and(hsv, hsv, mask=mask_gr)
	#mask_rd_ba = cv2.bitwise_and(hsv, hsv, mask=mask_rd)
	mask_both = cv2.add(mask_gr, mask_rd)
	
#Create contours
	contours = cv2.findContours(mask_both, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE)[0]
#Draw contours, nonadjusted
#        cv2.drawContours(image_cv, contours, -1, np.array([12,240,210]))
        c_size = 0
        error = 640
 
        for i in contours: #Check the size of the contour
              c_size = cv2.contourArea(i)
              if c_size > 500: #If the contour is larger than 1000 pixels
                  print ('Box size in pixels: %s' %c_size)
                  x,y,w,h = cv2.boundingRect(i) #Draw a box around the contour
                  cv2.rectangle(image_cv, (x,y) , (x+w, y+h), (0,0,255),2)
                  centre = cv2.moments(i)
                  error = centre['m10'] / centre['m00']
		  print "Testing casting: %s" %float(x)
		  blob_color = hsv[int(centre['m01'] / centre['m00']), int(centre['m10'] / centre['m00']), 0]
		  if (blob_color < 12):
			self.pub_color.publish(True)
			print "The blob is red"
		  else:
			self.pub_color.publish(False)							
                  print error
        
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))
	    self.pub_error.publish(error) #error tells how far left/right
	    self.pub_size.publish(c_size) #size used fro distance 
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('blob_detector')
    e = blob_detector()
    rospy.spin()

