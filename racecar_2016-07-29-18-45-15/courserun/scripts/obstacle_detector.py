#GENERAL ALGORITHM
#use lidar to detect objects in the front range
#find the object that is closest
#if object is farther than a certain distance, reject*
#record the index of that object

#wallfollower
#if object is right in front, then do a turn:
#if the left side is clear, go left
#if the right side is clear, go right
#else, go backwards

#based on the angle of the object, create the steering angle with pid

#*we only want this to run once the robot is near an obstacle so that it does not
#always override the wallfollower's messages


import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ObstacleAvoider:
    def __init__(self):
        
        self.pub = rospy.Publisher("/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size = 1) 
        
        #window dimensions
        self.midindex = 540
        self.windowrange = 200
        self.leftindex = self.midindex - self.windowrange
        self.rightindex = self.midindex + self.windowrange
    
        #pid values
        self.P = .001
        #self.I
        #self.D
        
        #Ackermann message
        self.message = AckermannDriveStamped()
        self.message.drive.speed = 1
        self.message.header.stamp = rospy.Time.now()
        
        self.condistance = 1.5#meters
        
    def detectCallback(self, msg):
        #the sample window
        sample = msg.ranges[self.leftindex:self.rightindex]
        
        #determine the minimum distance
        minele = min(sample)
        
        #decide whether to reject
        if minele > self.condistance: return
        
        #find index of the minimum distance
        ind = sample.index(minele)
        
        #find steering angle, if on right, go left (+); if on left, go right (-)
        v = self.P * (ind - self.midindex)
        
        #create message
        self.message.drive.steering_angle = v
        
        #publish message
        self.pub.publish(self.message)
        
        
def main():
    rospy.init_node("obstacle_pid")
    oa = ObstacleAvoider()
    sub = rospy.Subscriber("/scan", LaserScan, oa.detectCallback)
    rospy.spin()


if __name__ == "__main__":
    main()
