# usr/bin/env python
import cv2
import rospy
import numpy
from cv_bridge import CvBridge

from courserun.msg import BlobDetections
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Image

class Screenshot():
	def __init__ (self):
		rospy.Subscriber('/blob_image', Image, self.callback)
		rospy.Subscriber('/blob_detection', BlobDetections, self.isBlob)
		self.pub_msg = rospy.Publisher('/exploring_challenge', String, queue_size = 1)

		self.bridge = CvBridge()

		self.seeBlob = ""
		self.takenScreenshot = False

	def callback(self, msg):
		if self.seeBlob != "":
			if self.takenScreenshot == False:
				img = self.bridge.imgmsg_to_cv2(msg)
				chosen = self.seeBlob.data
				cv2.putText(img, chosen, (100, 100), cv2.FONT_HERSHEY_PLAIN,10, (0,0,0))
				filename = "/home/racecar/challenge_photos/%s.jpg"%(chosen)
				cv2.imwrite(filename,img)
				
				rospy.loginfo("IMAGE IS SAVED")
			self.takenScreenshot = True
#		else:#need a more robust Falsifier
#			self.takenScreenshot = False


	def isBlob(self, msg):
		#if there's a blob and the size is above threshold, then we see a blob
		threshold = 0 #0.5 #tbd
		if len(msg.sizes) > 0 and msg.sizes[0] > threshold:
			self.seeBlob = msg.colors[0]

	#def saveScreenshot(self, msg):

if __name__ == "__main__":
	rospy.init_node('Screenshot')
	s = Screenshot()
	rospy.spin()
