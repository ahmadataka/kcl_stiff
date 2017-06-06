#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#


import roslib; roslib.load_manifest('kcl_stiff')
import rospy
from geometry_msgs.msg import Pose
import tf
from sympy import *
import math

class goal_broadcast(object):
  def __init__(self):
    rospy.init_node('camera_baxter_broadcast')
    #self.ref_frame = "head"
    self.ref_frame = "torso"
    self.active_frame = "openni_depth_frame"
    self.active_frame = "openni_link"
    rate = rospy.Rate(10.0)
    angle = 15.0*3.14/180.0
    R = Matrix([[cos(angle), 0, sin(angle)],
		[0, 1, 0],
		[-sin(angle), 0, cos(angle)]])
    trace = R[0,0] + R[1,1] + R[2,2];
    if( trace > 0 ):
      s = 0.5 / sqrt(trace+ 1.0);
      w = 0.25 / s;
      x = ( R[2,1] - R[1,2] ) * s;
      y = ( R[0,2] - R[2,0] ) * s;
      z = ( R[1,0] - R[0,1] ) * s;
    else:
      if ( R[0,0] > R[1,1] and R[0,0] > R[2,2] ):
	s = 2.0 * sqrt( 1.0 + R[0,0] - R[1,1] - R[2,2]);
	w = (R[2,1] - R[1,2] ) / s;
	x = 0.25 * s;
	y = (R[0,1] + R[1,0] ) / s;
	z = (R[0,2] + R[2,0] ) / s;
      elif (R[1,1] > R[2,2]):
	s = 2.0 * sqrt( 1.0 + R[1,1] - R[0,0] - R[2,2]);
	w = (R[0,2] - R[2,0] ) / s;
	x = (R[0,1] + R[1,0] ) / s;
	y = 0.25 * s;
	z = (R[1,2] + R[2,1] ) / s;
      else:
	s = 2.0 * sqrt( 1.0 + R[2,2] - R[0,0] - R[1,1] );
	w = (R[1,0] - R[0,1] ) / s;
	x = (R[0,2] + R[2,0] ) / s;
	y = (R[1,2] + R[2,1] ) / s;
	z = 0.25 * s;
    while (not rospy.is_shutdown()):
      goal = tf.TransformBroadcaster()

      goal.sendTransform((0.21,0.0,0.22+0.506),
			(x, y, z, w),
			rospy.Time.now(),
			self.active_frame,
			self.ref_frame)
      rate.sleep()



if __name__ == '__main__':
    try:
        goal_broadcast()
    except rospy.ROSInterruptException: pass
