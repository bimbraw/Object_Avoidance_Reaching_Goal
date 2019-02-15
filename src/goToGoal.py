#!/usr/bin/env python

#basic imports
import os
import rospy
import cv2
import sys
import numpy as np
import math
import signal
import sys

#import different msgs
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class goToGoal:

  def __init__(self):
    rospy.init_node('goToGoal', anonymous=True, log_level=rospy.DEBUG, disable_signals = True)
    self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.update_odometry, queue_size = 1)
    self.emergency_subscriber = rospy.Subscriber("/location", Point,
                                                 self.emergency_callback, queue_size = 1)
    self.restart_subscriber = rospy.Subscriber("/turtlebot3/restart", String,
                                               self.restart_callback, queue_size = 1)
    self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    self.init_scheme()
    self.publish_rate = rospy.Rate(10)

    rospy.on_shutdown(self.stop_wheels)
    signal.signal(signal.SIGINT, self.signal_handler)
    rospy.spin()

  #shutdown part
  def signal_handler(self, sig, frame):
    self.stop_wheels()
    rospy.signal_shutdown("Shutdown")

  #initializing
  def init_scheme(self):
    self.way_points = []
    self.current_waypoint = 0
    self.parse_txt()
    self.waypoint_counter = 0
    self.init = True
    self.init_pos = Point()
    self.global_pos = Point()
    self.init_ang = 0
    self.global_ang = 0
    self.distance_epsilon = 0.01
    self.obstacle_loc = Point()
    self.obstacle_time = rospy.Time.now()
    self.ignore_emergency = True
    self.initial_encounter = True

  #restarting the callback
  def restart_callback(self, string):
    if string == "restart":
      self.init_scheme()

  #parsing the text
  def parse_txt(self):
    current_path = os.path.dirname(os.path.abspath(__file__))
    input_file = os.path.join(current_path, "wayPoints.txt")
    text_file = open(input_file, "r")
    lines = text_file.readlines()

    for line in lines:
      line = line.strip()
      sentence_list = line.split(" ")

      if sentence_list[0] == "#" or sentence_list[0] == "":
        continue

      assert len(sentence_list) == 2
      self.way_points.append( (float(sentence_list[0]), float(sentence_list[1])) )

  #from Sean
  def update_odometry(self, Odom):
    position = Odom.pose.pose.position

    # Orientation uses the quaternion aprametrization.
    # To get the angular position along the z-axis, the following equation is required.
    q = Odom.pose.pose.orientation
    orientation = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    if self.init:
      # The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
      self.init = False
      self.init_ang = orientation
      self.global_ang = self.init_ang
      Mrot = np.matrix(
        [[np.cos(self.init_ang), np.sin(self.init_ang)], [-np.sin(self.init_ang), np.cos(self.init_ang)]])
      self.init_pos.x = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y
      self.init_pos.y = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y
      self.init_pos.z = position.z

    Mrot = np.matrix([[np.cos(self.init_ang), np.sin(self.init_ang)], [-np.sin(self.init_ang), np.cos(self.init_ang)]])

    # We subtract the initial values
    self.global_pos.x = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y - self.init_pos.x
    self.global_pos.y = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y - self.init_pos.y
    self.global_ang = orientation - self.init_ang
    self.go_to_goal()

  #within bounds
  def within_bounds(self):
    goal = self.way_points[self.current_waypoint]
    curr_pos = self.global_pos
    return math.sqrt((goal[0] - curr_pos.x) ** 2 + (goal[1] - curr_pos.y) ** 2) < self.distance_epsilon

  #go to goal
  def go_to_goal(self):
    goal = self.way_points[self.current_waypoint]
    dist_to_goal = Point()

    if self.within_bounds():
      self.current_waypoint += 1

      if self.current_waypoint == 3:
        self.ignore_emergency = False

      if self.current_waypoint == len(self.way_points):
        rospy.signal_shutdown("I am done!")

      goal = self.way_points[self.current_waypoint]
      rospy.loginfo("----------------------------")
      rospy.loginfo("Updating waypoint!! " + str(goal[0]) + "," + str(goal[1]))
      rospy.loginfo("----------------------------")

      if self.current_waypoint < len(self.way_points):
        dist_to_goal.x = goal[0] - self.global_pos.x  
        dist_to_goal.y = goal[1] - self.global_pos.y  
	#offset
        dist_to_goal.z = math.atan2(dist_to_goal.y, dist_to_goal.x) - self.global_ang

      else:
        # you are done with planning, send velocity 0
        dist_to_goal.x = 0
        dist_to_goal.y = 0
        dist_to_goal.z = 0

    else:
      dist_to_goal.x = goal[0] - self.global_pos.x
      dist_to_goal.y = goal[1] - self.global_pos.y
      #offset
      dist_to_goal.z = math.atan2(dist_to_goal.y, dist_to_goal.x) - self.global_ang
    self.drive_wheels(dist_to_goal)

  #emergency callback
  def emergency_callback(self, data):
    self.obstacle_loc = data
    self.obstacle_time = rospy.Time.now()

  #stop the wheels
  def stop_wheels(self):
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    self.vel_publisher.publish(twist)
    self.publish_rate.sleep()
    return

  #drive wheels
  def drive_wheels(self, data):
    length_offset = math.sqrt(data.x ** 2 + data.y ** 2)
    angle_error = data.z
    rospy.loginfo(str(data.x) + "," + str(data.y) + "," + str(data.z))
    p_value_linear = 0.5
    p_value_angular = 1
    p_value_angular_emergency = 0.3
    if math.degrees(angle_error) > 180:
      angle_error = -(2*math.pi - angle_error)

    #if everything zero, don't move
    if data.x == 0 and data.y == 0 and data.z == 0:
      rospy.loginfo("Not moving")
      twist = Twist()
      twist.angular.z = 0
      self.vel_publisher.publish(twist)
      self.publish_rate.sleep()

    elif (rospy.Time.now() - self.obstacle_time < rospy.Duration(1)) and \
            (self.obstacle_loc.x != 0 or self.obstacle_loc.y != 0) and \
            self.ignore_emergency == False:

      # x is the distance from obstacle to vehicle
      # y is the angle in which the obstacle is located
      if self.initial_encounter:
        twist = Twist()
        twist.linear.x = -0.3
        self.vel_publisher.publish(twist)
        for i in xrange(10):
          self.publish_rate.sleep()
        self.initial_encounter = False

      rospy.loginfo("Obstacle")
      x = self.obstacle_loc.x
      y = self.obstacle_loc.y
      twist = Twist()
	
      # 0.523 is 30 degrees; the closer you are, the more you turn
      twist.angular.z = p_value_angular_emergency * (y - math.pi / 6) / (x ** 2) 
      self.vel_publisher.publish(twist)
      self.publish_rate.sleep()
      twist.angular.z = 0
      twist.linear.x = 0.1
      self.vel_publisher.publish(twist)
      self.publish_rate.sleep()


    else:
      rospy.loginfo("Moving")
      rospy.loginfo("Angle error: " + str(math.degrees(angle_error)) + " deg")
      
      if math.degrees(angle_error) > 10 or math.degrees(angle_error) < -10:
        movement_vel = p_value_angular * angle_error

        twist = Twist()
        twist.angular.z = movement_vel
        self.vel_publisher.publish(twist)
        self.publish_rate.sleep()

      else:

        twist = Twist()

        dist_error = length_offset
        rospy.loginfo("Dist error: " + str(dist_error) + " m")
        if dist_error > 0.2:
          twist.linear.x = 1
        else:

          twist.linear.x = p_value_linear * dist_error

        self.vel_publisher.publish(twist)
        self.publish_rate.sleep()

if __name__ == "__main__":
  goToGoal()
