#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

def scan_callback(msg):
    ranges =  msg.ranges[450:651]
    for i in range(len(ranges)):
        if ranges[i] < .75 and i<630:
            if abs(sum(ranges[i+1:i+20])/19 - ranges[i]) < .1:
		safety_pub.publish(go_back)

rospy.init_node("safety")
go_back = AckermannDriveStamped()
go_back.header.stamp = rospy.Time.now()
go_back.drive.steering_angle = -.4
go_back.drive.speed = -.2

safety_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=0)
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)

rospy.spin()
