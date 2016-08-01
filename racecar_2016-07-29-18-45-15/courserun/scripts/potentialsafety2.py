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
		self.pSpeed = .1#.1#0.03
		self.pAngle = .5#0.125
		#d values
		self.dSpeed = 0
		self.dAngle = .1#.02#.01

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
		self.queue_length = 10
		self.default_speed = 3
		self.prev_speeds = [self.default_speed] * self.queue_length
		self.prev_angles = [0] * self.queue_length

		#safety bounds
                self.bounds = 100
                self.lower = 540 - self.bounds
                self.upper = 540 + self.bounds

	def laser_callback(self, msg):
                #safety
		sample = msg.ranges[self.lower:self.upper]#200 indices
		mindist = min(sample)
		rospy.loginfo(mindist)
		if mindist < 1:
                        self.driveX = -35
                        index = sample.index(mindist)
                        self.driveY = 0.4 if index > 100 else -0.4
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

                #unprocessed
		speed = math.sqrt(xSum**2 + ySum**2)*np.sign(xSum)
		angle = math.atan2(ySum,xSum)*np.sign(xSum)
		
                #proportional component
		#speed
		pspeed = self.pSpeed*speed
		#angle
		pangle = self.pAngle*angle#turn to, so when go backward, will also turn to

                #derivative component
                dspeed = -self.dSpeed * (speed - self.prev_speeds[0])
                dangle = -self.dAngle * (angle - self.prev_angles[0])
		
		#message builder
		self.message.drive.speed = pspeed + dspeed
		self.message.drive.steering_angle = pangle + dangle
		
		self.prev_speeds.append(speed)
		self.prev_speeds.pop(0)#take out first ele
		self.prev_angles.append(angle)
		self.prev_angles.pop(0)

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
		




