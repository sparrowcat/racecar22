#!/usr/bin/env python

#imports
import rospy
import threading
import cv2
import numpy as np
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from courserun.msg import BlobDetections

#define global vars
global colors, color_map, color_ideal
colors = ["red", "green"]#, "yellow"]

color_map = {"green": (np.array([50,100,100]),np.array([70,255,255])),\
	"red": (np.array([0, 100, 100]),np.array([12, 255, 255]))}

redB = ColorRGBA()
redB.r = 255
yellowB = ColorRGBA()
yellowB.r = 255
yellowB.g = 255
greenB = ColorRGBA()
greenB.g = 255
color_ideal = {"red": redB,"yellow":yellowB,"green":greenB}

Min_Size = 0 #Always set to 0
class blob_detector:
    def __init__(self):
        self.node_name = "blob_detector"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("/blob_image",\
                Image, queue_size=1)
	self.pub_mess = rospy.Publisher("/blob", Float32, queue_size=1)
        self.bridge = CvBridge()

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

    	# Threshold the HSV image to get only red or only green
        mask_gr = cv2.inRange(hsv, gr_lower, gr_upper)
        mask_rd = cv2.inRange(hsv, rd_lower, rd_upper)

	#mask_both = cv2.add(mask_gr, mask_rd)
	
	#Create contours
	#contours = cv2.findContours(mask_both, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE)[0]

	for i in range(len(colors)):
		col = color_dims[colors[i]]
		(lower, upper) = col
		comb = cv2.inRange(hsv,lower,upper)
		colrgba = color_map[colors[i]]
		contours = cv2.findContours(comb, mode = cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)[0]
		Boxes = []
		ImgBox = []
		for co in contours:
			rect = cv2.minAreaRect(co)
			(w,h) = rect[1]
			area = w*h
			if not area > lowthresarea: continue
			angle = rect[2]
			#if angle < -30 and angle > -60: continue#too angled
			p = max(w/h,h/w) 
			#if p > 2: continue#the rectangle probably does not enclose the paper 
			center = rect[0]
			(y,x) = center
			cent = Point()
			cent.x = x/width
			cent.y = y/height
			box = cv2.cv.BoxPoints(rect)
			box = np.int0(box)
			Boxes.append((Float64(area),cent,colrgba))
			ImgBox.append(box)#does not matter the order
		
		Arr += Boxes
		BoxArr += ImgBox
		combC = cv2.cvtColor(comb, cv2.COLOR_GRAY2BGR).astype(np.uint8)
		combAll = np.add(combAll, combC)

	    Arr.sort()#BoxArr does not need to be sorted
	    Arr = Arr[::-1]
	    Sn = [o[0] for o in Arr]
	    Ln = [o[1] for o in Arr]
	    Cn= [o[2] for o in Arr]
	   #create message
	    mesg = BlobDetections()
	    mesg.sizes = list(Sn)
	    mesg.locations = list(Ln)
	    mesg.colors = list(Cn)

	    objimage = np.empty(img.shape).astype(np.uint8)
	    np.copyto(objimage, img)
	    cv2.drawContours(objimage,BoxArr,0,(255,255,255),4)

	'''
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
        '''
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(objimage, "bgr8"))#image_cv
	    self.pub_mess.publish(mesg)

        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()


if __name__=="__main__":
    rospy.init_node('blob_detector')
    e = blob_detector()
    rospy.spin()

