#GENERAL ALGORITHM
#freespace

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time, collections

class ObstacleAvoider:
    def __init__(self):
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1) 
        
        #window dimensions
        self.midindex = 540
        self.windowrange = 360
        self.leftindex = self.midindex - self.windowrange
        self.rightindex = self.midindex + self.windowrange
	self.samplingsize = 10
	#averaged is 2*self.windowrange/self.samplingsize big
	self.sampmid = int(self.windowrange/self.samplingsize)
    
        #pid values
        self.P = .01 #.0005 .0004
        #self.I
        self.D = 0 #.00015
	self.prev_times = collections.deque([time.clock() for _ in range(10)])
        self.prev_errors = collections.deque([0 for _ in range(4)])
                

        #Ackermann message
        self.message = AckermannDriveStamped()
        self.message.drive.speed = 1
        self.message.header.stamp = rospy.Time.now()
        
        #self.condistance = 1.5#meters

        
    def detectCallback(self, msg):
        #the sample window
        sample = msg.ranges[self.leftindex:self.rightindex]

	#turn sample window into range of averaged vals
	averaged = []
	prevind = 0
	for i in range(self.samplingsize, len(sample)+self.samplingsize, self.samplingsize):
		samp = sample[prevind:i]
		averaged.append(reduce(lambda x, y: x+y, samp)/len(samp))
		prevind = i
        
        #determine the maximum distance
        maxele = max(averaged)
        
        #find index of the maximum distance
        ind = averaged.index(maxele)
       
        #find steering angle, left is +, right is -
	error = ind - self.sampmid #carefully check mid value
	prev_error = self.prev_errors.popleft()
        prev_time = self.prev_times.popleft()
        e_deriv = (error - prev_error) / (time.clock() - prev_time)
        e_int = (error + prev_error) / 2 * (time.clock() - prev_time)
        self.prev_times.append(time.clock())
        self.prev_errors.append(error)
        v = self.P * error - self.D * e_deriv
	rospy.loginfo(v)

        
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
