import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np
import math

class Potential:
	def __init__(self):
		#CONSTANTS
		self.charge = .1#0.0007

		#drive vector
		self.driveX = 35
		self.driveY = 0

		#p values
		self.pSpeed = .1#.1#0.03#?
		self.pAngle = .5#0.125

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
		#CHANGE 1081
		indices = np.arange(0, 1080)
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
		#CHANGE vesc
		self.pub = rospy.Publisher("/racecar/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)

                #previous vals
		self.prev_speed = 0
		self.prev_angle = 0

		#safety bounds
                self.bounds = 100
                self.lower = 540 - self.bounds
                self.upper = 540 + self.bounds
        '''      
                #safety message
		#CHANGE vesc
                self.reverse = AckermannDriveStamped()
		self.reverse.header.stamp = rospy.Time.now()
		self.reverse.drive.speed = -0.5

		#safety pub
		self.pub_safety = rospy.Publisher("/racecar/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=0)
        
	def set_angles(self, ind):
                rospy.loginfo(ind)
		if ind < 100:#on right side
			self.reverse.drive.steering_angle = -0.4
		else:#on left side
			self.reverse.drive.steering_angle = 0.4
        '''
	def laser_callback(self, msg):
                #safety
		sample = msg.ranges[self.lower:self.upper]#200 indices
		mindist = min(sample)
		rospy.loginfo(mindist)
		if mindist < 1:
                        self.driveX = -35
                        index = sample.index(mindist)
                        self.driveY = 0.4 if index > 100 else -0.4
                        '''
                        #for high velocity, .6
                        index = sample.index(mindist)
                        self.set_angles(index)
                        self.pub_safety.publish(self.reverse)
                        '''
                else:
                        self.driveX = 35
                        self.driveY = 0

                #potential
                
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
		rospy.loginfo(speed)
		#angle
		angle = self.pAngle*math.atan2(ySum,xSum)*np.sign(xSum)#turn to, so when go backward, will also turn to

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
	#CHANGE laser
	rospy.Subscriber("racecar/laser/scan", LaserScan, po.laser_callback)
	rospy.spin()

if __name__ == "__main__":
	main()
