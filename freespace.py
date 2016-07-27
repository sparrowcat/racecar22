#GENERAL ALGORITHM
#freespace

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ObstacleAvoider:
    def __init__(self):
        
        self.pub = rospy.Publisher("/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1) 
        
        #window dimensions
        self.midindex = 540
        self.windowrange = 360
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
        
        #self.condistance = 1.5#meters
        
    def detectCallback(self, msg):
        #the sample window
        sample = msg.ranges[self.leftindex:self.rightindex]
        
        #determine the maximum distance
        maxele = min(sample)
        
        #find index of the maximum distance
        ind = sample.index(maxele)
        
        #find steering angle, left is +, right is -
        v = self.P * (self.midindex - ind)
        
        #create message
        self.message.drive.steering_angle = v
        
        #publish message
        self.pub.publish(self.message)
        
        
def main():
    rospy.init_node("freespace_pid")
    oa = ObstacleAvoider()
    rospy.Subscriber("/scan", LaserScan, oa.detectCallback)
    rospy.spin()


if __name__ == "__main__":
    main()
