#!/usr/bin/env python
# This code is used to estimate the relative velocity between the human and the robot
import roslib
roslib.load_manifest('kcl_stiff')
import rospy
import math
import tf
import numpy as np
from sympy import *
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from sympy.mpmath import norm

class closest_distance(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('velocity_estimate')
    # Define the velocity magnitude variable
    self.vel_sent = Float64MultiArray()
    # Define the velocity variable
    self.vel_vec_sent = Float64MultiArray()

    # Define the ROS Subscriber
    # Used to receive the distance magnitude between robot and human
    dist_sub = rospy.Subscriber('/fourbythree_topics/stiffness/closest_distance/scalar', Float64MultiArray, self.get_dist)
    # Used to receive the distance vector between robot and human
    dist_vec_sub = rospy.Subscriber('/fourbythree_topics/stiffness/closest_distance/vector', Float64MultiArray, self.get_dist_vec)

    # Define the ROS Publisher
    # Used to publish the relative speed between the robot and human
    vel_pub = rospy.Publisher('/fourbythree_topics/stiffness/velocity', Float64MultiArray, queue_size = 10)
    # Used to publish the relative velocity vector between the robot and human
    vel_vec_pub = rospy.Publisher('/fourbythree_topics/stiffness/velocity_vector', Float64MultiArray, queue_size = 10)

    # Initialize the variable array
    self.initialize()
    # Set the rate to be 40 Hz
    self.freq = 40.0
    rate = rospy.Rate(self.freq)
    while not rospy.is_shutdown():
        # Estimate the velocity
        self.get_vel()
        # Prepare the data to be sent
        self.transform_to_sent()
        # Publish speed and velocity
        vel_pub.publish(self.vel_sent)
        vel_vec_pub.publish(self.vel_vec_sent)
        # Sleep until 1/40 s
        rate.sleep()

  # This function is used to get the scalar distance
  def get_dist(self,msg):
    self.dist = msg.data[0]

  # This function is used to get the vector distance
  def get_dist_vec(self,msg):
    self.dist_vec[0] = msg.data[0]
    self.dist_vec[1] = msg.data[1]
    self.dist_vec[2] = msg.data[2]

  # This function is used to initialize the distance and velocity array
  def initialize(self):
    # Initialize speed and velocity vector
    self.vel = 0.0
    self.vel_vec = [0.0, 0.0, 0.0]

    # Initialize distance scalar and distance vector
    self.dist_save = 0.0
    self.dist = 0.0
    self.dist_vec = [0.0, 0.0, 0.0]
    self.dist_vec_save = [0.0, 0.0, 0.0]

  # This function is used to calculate velocity
  def get_vel(self):
    # Get the velocity magnitude
    self.vel = (self.dist - self.dist_save)*self.freq
    # Save previous distance
    self.dist_save = self.dist

    for i in range(0,3):
      # Get the velocity vector
      self.vel_vec[i] = (abs(self.dist_vec[i]) - abs(self.dist_vec_save[i]))*self.freq
      # Save previous distance vector
      self.dist_vec_save[i] = self.dist_vec[i]

  # This function is used to prepare data before being sent
  def transform_to_sent(self):
    # Empty the array
    self.vel_sent.data = []
    self.vel_vec_sent.data = []
    # Put the velocity magnitude into array
    self.vel_sent.data.append(self.vel)
    for i in range(0,3):
      # Put the velocity vector into array
      self.vel_vec_sent.data.append(self.vel_vec[i])

if __name__ == '__main__':
    try:
        closest_distance()
    except rospy.ROSInterruptException: pass
