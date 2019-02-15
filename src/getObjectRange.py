#!/usr/bin/env python

#basic imports
import rospy
import sys
import numpy as np
import math

#msg imports
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan



class getObjectRange:
    def __init__(self, debug_mode):
        rospy.init_node('getObjectRange', anonymous=True, log_level=rospy.DEBUG)
        rospy.Subscriber("/scan", LaserScan, self.callback_laser, queue_size = 1)
        self.loc_pub = rospy.Publisher("/location", Point, queue_size = 1)
        self.limit = 0.5
        self.pub_rate = rospy.Rate(10)
        rospy.spin()

    def callback_laser(self, data):
        length = len(data.ranges)
        left_limit = length / 12 #looking amint 0 - 30 degrees and 330 - 360 degrees
        
	#defininf points of interest
	points_of_interest = np.array(data.ranges[:left_limit][::-1] + data.ranges[-left_limit:][::-1])	
        points_of_interest[points_of_interest < data.range_min] = float('inf')

	#defining minimum distance
        min_dist = min(points_of_interest)

        if min_dist < self.limit:
            pub_point = Point()
            pub_point.x = min_dist
            pub_point.y = np.where(points_of_interest == min_dist)[0][0] * 1.04 / len(points_of_interest) #60 degrees is 1.04 rad
            rospy.loginfo("x: " + str(pub_point.x) + "m\t"  + "y: " + str(math.degrees(pub_point.y)) + " deg\t")
            self.loc_pub.publish(pub_point)
            self.pub_rate.sleep()


if __name__ == '__main__':
  args = sys.argv
  if len(args)  == 2 and args[1] == 'debug':
    getObjectRange(True)
  else:
    getObjectRange(False)
