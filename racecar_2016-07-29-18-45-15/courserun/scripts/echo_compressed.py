#!/usr/bin/env python
""" 
    echo_compressed.py
    author: Ariel Anders
    Listen to a ros topic image and publish a compressed one
    
    
    Run echo compressed by giving it the image topic name as a parameter:
 
    > python echo_compressed.py _image:=/camera/rgb/image_rect_color
    
   this will publish two topics:
      /EchoCompression/echo_image
      /EchoCompression/echo_image/compressed
    
See http://wiki.ros.org/image_transport for image transport plugin information
"""

import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class EchoCompression:
    def __init__(self):
        self.node_name = "EchoCompression"
        sub_topic = rospy.get_param('~image')
        self.sub_image = rospy.Subscriber(sub_topic, Image,\
                self.processImage, queue_size=1)
        self.pub_image_comp = rospy.Publisher("~echo_image/compressed",\
                CompressedImage, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def processImage(self, image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_cv)[1]).tostring()
        self.pub_image_comp.publish(msg)

if __name__=="__main__":
    rospy.init_node('EchoCompression')
    e = EchoCompression()
    rospy.spin()
