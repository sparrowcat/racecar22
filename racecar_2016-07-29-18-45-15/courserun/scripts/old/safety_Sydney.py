#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class EmergencyStop:
	def __init__(self):
		#Listeners
		self.lidar_detect = rospy.Subscriber("/scan", LaserScan, self.LidarScanReceived) #Only used to call the emergency stop program if necessary.
		
		#self.pub_stop_func = rospy.Publisher("/racecar/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=5)
		self.pub_stop_func = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=5) #Overrides any other command to the input/navigation topic
		

	def LidarScanReceived(self, msg):
                drive_msg = AckermannDriveStamped()
		distances = msg.ranges #Gets a list that contains data about the distance of the nearest object for each degree measure
		if (min(msg.ranges[720:760]) < .2):	#If the mininimum of the distances to the nearest object for all angles measured between the array locations 520 and 560 is less than 1.5, stop moving.	
			drive_msg.drive.speed = 0
			self.pub_stop_func.publish(drive_msg)


if __name__=="__main__":
	rospy.init_node('EmergencyStop')
	e = EmergencyStop()
	rospy.loginfo("Intializing Emergency Stop")
	rospy.spin()


