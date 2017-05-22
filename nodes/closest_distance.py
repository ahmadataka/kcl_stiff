#!/usr/bin/env python
# This code is used to get the closest distance between points on the robot and points on the human body
import roslib
roslib.load_manifest('kcl_stiff')
import rospy
import math
import tf
import numpy as np
from sympy import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
from sympy.mpmath import norm

class closest_distance(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('closest_distance')
    # Define the number of frames on the robot body
    number_robot = 3
    # Define the number of frames on the human body
    number_human = 3
    # Calculate the number of all possible frame combinations
    self.bound = number_human*number_robot
    # Define the variable used to send the closest distance magnitude
    self.dist_sent = Float64MultiArray()
    # Define the variable used to send the closest distance vector
    self.dist_sent_vec = Float64MultiArray()

    # Define the ROS Publisher
    # Used to publish the closest distance magnitude
    dist_pub = rospy.Publisher('/fourbythree_topics/stiffness/closest_distance/scalar', Float64MultiArray, queue_size = 10)
    # Used to publish the closest distance vector
    dist_vec_pub = rospy.Publisher('/fourbythree_topics/stiffness/closest_distance/vector', Float64MultiArray, queue_size = 10)

    # Define the ROS Subscriber
    # Used to receive the distance of all possible frame combinations
    sub = rospy.Subscriber('/fourbythree_topics/stiffness/human_robot_distance', PoseArray, self.get_distance)

    # Initialize the distance array
    self.initialize()

    # Set the rate to be 40 Hz
    rate = rospy.Rate(40.0)
    while not rospy.is_shutdown():
        # Calculate the closest distance
        self.close()
        # Prepare to publish
        self.transform_to_sent()
        # Publish the magnitude of closest distance between robot and human
        dist_pub.publish(self.dist_sent)
        # Publish the vector of closest distance between robot and human
        dist_vec_pub.publish(self.dist_sent_vec)

        # Sleep until 1/40 ss
        rate.sleep()

  # This function is used to receive the distance information between robot and human
  def get_distance(self,msg):
    for i in range(0,self.bound):
      self.dist[i] = Matrix([[msg.poses[i].position.x],[msg.poses[i].position.y],[msg.poses[i].position.z]])

  # This function is used to initialize the distance array
  def initialize(self):
    # Set an empty array
    self.dist = []
    # Put zero as many as the number of all possible frame combinations
    for i in range(0,self.bound):
      self.dist.append(ones(3,1))

  # This functoin is used to calculate the closest distance between robot and human
  def close(self):
    # Define some high value
    self.maks = 100
    # Reset the index
    self.index = 0

    for i in range(0,9):
      # Calculate the magnitude of the distance
      magni = norm(self.dist[i])
      # Check whether the distance is less than the maximum value
      if(magni<self.maks):
        # If so, update the maximum value and the index
    	self.maks = magni
    	self.index = i

  # This function is used to prepare the data being published
  def transform_to_sent(self):
    # Empty the array
    self.dist_sent.data = []
    self.dist_sent_vec.data = []

    # Put the minimum distance in the array
    self.dist_sent.data.append(self.maks)
    # Put the closest distance vector in the array
    for i in range(0,3):
      self.dist_sent_vec.data.append(self.dist[self.index][i,0])

if __name__ == '__main__':
    try:
        closest_distance()
    except rospy.ROSInterruptException: pass
