#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped


from racecar.msg import BlobDetections
from std_msgs.msg import Float64,ColorRGBA,String
from geometry_msgs.msg import Point

import numpy as np

global Wall_Follow, Turning
Wall_Follow = False
Turning = False
class MoveTowardImage:
	def __init__(self):			#Don't put while lops in call backs. It will overload a message stream
		rospy.loginfo("Intializing node")
		#Listeners. Listeners's third parameter are always callback functions. Listeners never have queue_sizes.

		self.blob_detect = rospy.Subscriber("/blob_detections", BlobDetections, self.MoveToBlob)	#blob_detect is an object that contains data about multiple blobs. The biggest blob is the one that has index 0. The same indices correspond to the same blob over the multiple data types in the message fields.


		#Talkers
		self.pub_mov_func = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)
		self.pub_blob_color_func = rospy.Publisher("/blob_color", String, queue_size=1)	#This is used to talk to the wall follow node. talkers setup the topics that subscribers/listeners subscribe/listen to

		#Fields/Constant. Most of the things here are not used yet (ie the things in the state and constants sections)
		# Messages

		# Constants
		self.speed = 1.2
		
		

		
	def MoveToBlob(self, blob_detect): #Moves to the largest blob.
		global Wall_Follow, Turning
                drive_msg = AckermannDriveStamped() #Sets the drive message to the  AckermannDriveStamped message type
		blob_Color = ColorRGBA()
		
		drive_msg.drive.speed = self.speed	#Sets a field of the drivemsg message.
		drive_msg.drive.steering_angle = -.06	#Gets the position of the largest blob. The first will be the largest  blob because the blobs are sorted by size. (index 0 is biggest size).
		
		
		if (len(blob_detect.colors) == 0):	#Make sure the blob detect arrays are not empty
                        self.pub_mov_func.publish(drive_msg)
			rospy.loginfo("No blobs detected")
                        return#exit
		
		turning_start_time = rospy.get_time()
                #if there are blobs
                largest_blob_area = blob_detect.sizes[0].data
		largest_blob_pos = blob_detect.locations[0]
		blob_x_pos = largest_blob_pos.x		#This will be in between 0 and 1 (inclusive) (0 is the left and 1 is the right)
		blob_y_pos = largest_blob_pos.y		#Same thing here
		blob_color = blob_detect.colors[0]	#In rgba

                if (Wall_Follow == True and Turning == False):       #Stage 4. Stop subscriptions from this node.
			rospy.loginfo("Stopping control system node and publishing to wall follower.")
			#Intiate wall follow by publishing the color to the wall follow's topic.
			color = "r" if (blob_color.r == 255) else "g"
			self.pub_blob_color_func(color)

			#stop further subscriptions and basically end the node
			self.blob_detect.unregister()
                elif (Wall_Follow == False and Turning == False):
                        if (largest_blob_area > 8000):	#Stage 2. We are close to the blob, so we initiate turning
                                #PUT TURN HERE
				rospy.loginfo("Intializing Turning.")
				Turning = True
				turning_start_time = rospy.get_time()

                        else:			#Stage 1
                        #We are far from the blob, so we continue to navigate towards the blob.
                        #While we are relatively far away from the blob (ie blob size is small relative to screen), we drive towards the center of the blob. 
				rospy.loginfo("Navigating to blob")
                                if (blob_x_pos > 0.60): #If Blob is On the right side
                                        drive_msg.drive.steering_angle = .3 #Turn left
                                elif (blob_x_pos < 0.40): #If Blob is on the left side.
                                        drive_msg.drive.steering_angle = -.3 #Turn right
                                else:	#If blob_x_pos (the x component of he blob's centroid) is in the middle 200 pixels, just move forward
                                        drive_msg.drive.steering_angle = 0

		if (Turning == True): 	#Stage 3. Start turning
			rospy.loginfo("Turning")
			if (turning_start_time + 4 > rospy.get_time()):	#Keeps turning for three seconds (approx)
				if (blob_color.r == 255):
					drive_msg.drive.steering_angle = 1.5
				elif (blob_color.g == 255):
					drive_msg.drive.steering_angle = -1.5
					
				#If the blob color is red, turn left, else if it is green, turn right.
			else:
				Turning = False #After turning process finishes, change Turning to False and Wall_Follow to true
				Wall_Follow = True		

                #publish the message
                self.pub_mov_func.publish(drive_msg)
		
if __name__=="__main__":
	rospy.init_node('ControlSystem')
	CC = MoveTowardImage()
	rospy.loginfo("Initializing Control System")
	rospy.spin()

