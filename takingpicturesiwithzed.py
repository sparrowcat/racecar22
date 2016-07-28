#!/usr/env/bin python
#not goin to bother writing it all out because it's literally one function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import rospy

#subscribe to the camera's topic, and then we can run this just when blob is detected- will have to change filenames etc, but this is a start
def save_image(msg):
    #convert msg to cv2
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    #to save image 
    cv2.imwrite(filename, img[, params]) → retval
    #filename ends with jpg, png, etc. and params dictate compression- optional 
