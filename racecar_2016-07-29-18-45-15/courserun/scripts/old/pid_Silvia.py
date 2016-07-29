#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import time
t = rospy.Time(0)

class SafetyControllerNode:
    def __init__(self):

        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=1)
        self.counter = 0
        self.Pre()
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)
        

    def Pre(self):
	print t.to_sec()
        for repeat in range(0,1799):
            drivemsg = AckermannDriveStamped()
            drivemsg.drive.speed = 2  #for pid right, this value is 0.5 (idk why) -Silvia
	    drivemsg.drive.steering_angle = -1  #for pid left, this value is 1
	    print drivemsg
	    self.pub.publish(drivemsg)  
        
        print "done running pre set"

    def laser_callback(self, msg):
        self.counter += 1
        self.counter %= 4
        if self.counter != 0:
            return
        left = 900 #45 degrees
        right = 180 #default side
        avg = 0
        for i in range(0, 10):
            avg += msg.ranges[left - 5 + i]
        avg /= 10

        drivemsg = AckermannDriveStamped()
        drivemsg.drive.speed = .5
        if avg < 0.5:
            drivemsg.drive.steering_angle = -0.1  #for pid right, this value is 0.1
        elif avg > 0.6:
            drivemsg.drive.steering_angle = 0.1  #for pid right, this value is -0.1
        else:
            drivemsg.drive.steering_angle = 0
        self.pub.publish(drivemsg)

        rospy.loginfo("center range(s): %f", avg)

if __name__ == "__main__":
    rospy.init_node("SafetyControllerNode")
    node = SafetyControllerNode()
    rospy.spin()
