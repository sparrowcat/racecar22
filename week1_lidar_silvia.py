#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan, Joy # joystick and laser scanner msgs
from numpy.core.defchararray import lower

class WallE():
    
    # variable to return distance to wall from callback
    # (at 90 and 60 degrees to left of vehicle)
    #scan = [0.0, 0.0] # Ensure it is created before being used
    angle=0
    e1=0
    e2=0
    right=True
    death=False
    error = []
    
    def getIntegral(newError):
        if len(self.error) > 39:
            self.error.pop(0)
        self.error.append(newError)
        return (i for i in self.error)

    def getError(self,goal,L,begin,end):
        return goal-(min(L[begin:end]))
    
    def clip(self,low,high,value):
        if high<value:
            return high
        elif value<low:
            return low
        else:
            return value
        
    #get the angle
    def getSteeringCmd(self,error,fullLeft,fullRight):
        Kp =.6
        Kd = .7
        de= ((self.e1-self.e2)+(error-self.e1))/.2
        self.e2=self.e1
        self.e1=error
        u=Kp*error+Kd*de
        return self.clip(fullLeft,fullRight,u)
    
    #passed to the subscriber
    def callback(self,msg):
        #get the laser information
        if self.right: #right
            error=self.getError(.75, msg.ranges, 200, 540)
            self.angle=self.getSteeringCmd(error, -1, 1)
            self.death=min(msg.ranges[525:555])<.5
        else: #left
            error=self.getError(.75, msg.ranges, 540,900)
            self.angle=self.getSteeringCmd(-error, -1, 1)
            self.death=min(msg.ranges[525:555])<.5

    def callback2(self, msg):
        if(msg.buttons[0]):
            self.right = True
        elif(msg.buttons[1]):
            self.right = False
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
        rospy.sleep(1)
    
    def __init__(self):
        #setup the node
        rospy.init_node('wall_bang', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # output messages/sec (also impacts latency)
        rate = 10 
        r = rospy.Rate(rate)
        
        # node specific topics (remap on command line or in launch file)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        
        #sets the subscriber
        rospy.Subscriber('scan', LaserScan, self.callback)
        rospy.Subscriber('vesc/joy', Joy, self.callback2)
        
        # set control parameters
        speed = 2.0 # constant travel speed in meters/second
        dist_trav = 20.0 # meters to travel in time travel mode
        
        # fill out fields in ackermann steering message (to go straight)
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = self.angle
        #r60 = dist_wall / math.cos(math.radians(30.0)) # expected distance if parallel to wall
        
        # assume correct vehicle pose (at start)
        #scan = [dist_wall, r60] 
        
        # main processing loop (runs for pre-determined duration in time travel mode)
        time = dist_trav / speed
        ticks = int(time * rate) # convert drive time to ticks
        for t in range(ticks):
        #while True:
            # bang-bang controller for steering direction (single point version - don't use)
            # turn full left (1.0) or right (-1.0)
            #drive_cmd.drive.steering_angle = math.copysign(1.0,-(scan[1]-r60))
            drive_cmd.drive.steering_angle=self.angle
            
            if self.death:
                drive_cmd.drive.speed=-.1
            elif drive_cmd.drive.speed>0:
                drive_cmd.drive.speed = speed

            self.drive.publish(drive_cmd) # post this message
            #Chill out for a bit
            r.sleep() 
        # always make sure to leave the robot stopped
        self.drive.publish(AckermannDriveStamped())
        
if __name__ == '__main__':
#    try:
    WallE()
# except:
#    pass
#        rospy.loginfo("WallE node terminated.")
