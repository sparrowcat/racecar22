#/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
import math

class Potential:
	def __init__(self):
		#CONSTANTS
		self.charge = 0.07

		#drive vector
		self.driveX = 35
		self.driveY = 0

		#p values
		self.pSpeed = 0.03#?
		self.pAngle = 0.125

		#window stats
		self.midInd = 540.0
		self.sideWindow = 540.0

		#functions
		#generate theta finder function
		theta_lambda = lambda x: (x - self.midInd)/self.sideWindow*math.pi*3/4
		self.theta_function = np.vectorize(theta_lambda)
		#electric field function
		e_lambda = lambda x: self.charge/(x**2)
		self.e_function = np.vectorize(e_lambda)

		#preprocess
		#make list of consecutive numbers
		indices = np.arange(0, 1081)
		#apply function on indices
		thetas = self.theta_function(indices)
		#thetas = np.sum(np.multiply(np.arange(0, 1081),msg.angle_increment), msg.angle_min)		
		#find cosines
		self.cosines = np.cos(thetas)
		#find sines
		self.sines = np.sin(thetas)

		#message
		self.message = AckermannDriveStamped()
		#self.message.drive.acceleration = 1#set

		#pub
		self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)

		#previous vals
		self.prev_speed = 0
		self.prev_angle = 0
		

	def laser_callback(self, msg):
		#laser data
		sample = np.array(msg.ranges)

		#electric forces
		eForce = self.e_function(sample)

		#x-Coords, oppo dir (repulsive)
		x_Coords = -eForce * self.cosines

		#y-Coords, oppo dir (repulsive)
		y_Coords = -eForce * self.sines

		#x sum
		xSum = np.sum(x_Coords) + self.driveX

		#y sum
		ySum = np.sum(y_Coords) + self.driveY
		rospy.loginfo((xSum,ySum))

		#speed
		speed = self.pSpeed*math.sqrt(xSum**2 + ySum**2)*np.sign(xSum)

		#angle
		angle = self.pAngle*math.atan2(ySum,xSum)*np.sign(xSum)#turn to, so when go backward, will also turn to
		
		#do timer thing?
		if np.sign(speed) != np.sign(self.prev_speed):
			speed = 0

		#message builder
		self.message.drive.speed = speed
		self.message.drive.steering_angle = angle
		self.prev_speed = speed
		self.prev_angle = angle

		
		#message publish
		self.pub.publish(self.message)


def main():
	rospy.init_node("Potential")
	po = Potential()
	rospy.Subscriber("/scan", LaserScan, po.laser_callback)
	rospy.spin()

if __name__ == "__main__":
	main()
		



