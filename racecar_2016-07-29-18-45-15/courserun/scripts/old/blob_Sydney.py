#!/usr/bin/env python

import cv2
import numpy as np
import math
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
from std_msgs.msg import Float64
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from courserun.msg import BlobDetections

global colors, color_dims, color_map

colors = ["green","red"]

color_dims = {"red":(np.array([0,190,100]).astype(np.uint8), np.array([15,255,255]).astype(np.uint8)),\
              "green":(np.array([35,100,200]).astype(np.uint8), np.array([80,255,255]).astype(np.uint8)),\
              "yellow": (np.array([20,200,150]).astype(np.uint8), np.array([30,255,255]).astype(np.uint8))}

redB = ColorRGBA()
redB.r = 255
yellowB = ColorRGBA()
yellowB.r = 255
yellowB.g = 255
greenB = ColorRGBA()
greenB.g = 255
color_map = {"red": redB,"yellow":yellowB,"green":greenB}

    
#main process
def process(img):

    width, height, cha = img.shape
    combAll = np.array(img.shape).astype(np.uint8)

    Arr = []
    BoxArr = []

    lowthresarea = 2500#1000

    hsv = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.uint8)
        
    for i in range(len(colors)):
        col = color_dims[colors[i]]
        (lower, upper) = col
        comb = cv2.inRange(hsv,lower,upper).astype(np.uint8)
        colrgba = color_map[colors[i]]
       # cv2.imshow("c",comb)
        #cv2.waitKey(0)
        contours, hierarchy = cv2.findContours(comb, mode = cv2.RETR_CCOMP, method=cv2.CHAIN_APPROX_SIMPLE)#change later for cv
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

    if len(Arr) == 0:
        rospy.loginfo("Here")
        return BlobDetections(), img
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

    return mesg, objimage

class Echo:
    def __init__(self):
        self.node_name = "Blob_Detector"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("/blob_image",\
                Image, queue_size=1)
        self.pub_data = rospy.Publisher("/blob_detections", BlobDetections, queue_size = 1)
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

        #self.sub_image.unregister()
        
        msg, img = process(image_cv)
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            self.pub_data.publish(msg)
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()

    #start running
if __name__ == "__main__":#may want to set a hertz value
    rospy.init_node("BlobDetector")
    bd = Echo()
    rospy.spin()

