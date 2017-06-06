#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#


import roslib; roslib.load_manifest('kcl_stiff')
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform
import tf
from sympy import *
import math

class goal_broadcast(object):
  def __init__(self):
    rospy.init_node('camera_baxter_broadcast')
    #self.ref_frame = "head"
    self.ref_frame = "base"
    self.active_frame = "openni_link"
    # self.active_frame = "openni_rgb_optical_frame"
    deg_to_rad = math.pi/180.0

    yaw = 2.54857
    pitch = -0.02827
    roll = -0.01238
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    rate = rospy.Rate(10.0)
    # angle = 15.0*3.14/180.0
    # rot_mat = fromTranslationRotation(translation, rotation)
    # print rot_mat
    # R = Matrix([[cos(angle), 0, sin(angle)],
	# 	[0, 1, 0],
	# 	[-sin(angle), 0, cos(angle)]])
    # trace = R[0,0] + R[1,1] + R[2,2];
    # if( trace > 0 ):
    #   s = 0.5 / sqrt(trace+ 1.0);
    #   w = 0.25 / s;
    #   x = ( R[2,1] - R[1,2] ) * s;
    #   y = ( R[0,2] - R[2,0] ) * s;
    #   z = ( R[1,0] - R[0,1] ) * s;
    # else:
    #   if ( R[0,0] > R[1,1] and R[0,0] > R[2,2] ):
	# s = 2.0 * sqrt( 1.0 + R[0,0] - R[1,1] - R[2,2]);
	# w = (R[2,1] - R[1,2] ) / s;
	# x = 0.25 * s;
	# y = (R[0,1] + R[1,0] ) / s;
	# z = (R[0,2] + R[2,0] ) / s;
    #   elif (R[1,1] > R[2,2]):
	# s = 2.0 * sqrt( 1.0 + R[1,1] - R[0,0] - R[2,2]);
	# w = (R[0,2] - R[2,0] ) / s;
	# x = (R[0,1] + R[1,0] ) / s;
	# y = 0.25 * s;
	# z = (R[1,2] + R[2,1] ) / s;
    #   else:
	# s = 2.0 * sqrt( 1.0 + R[2,2] - R[0,0] - R[1,1] );
	# w = (R[1,0] - R[0,1] ) / s;
	# x = (R[0,2] + R[2,0] ) / s;
	# y = (R[1,2] + R[2,1] ) / s;
	# z = 0.25 * s;
    while (not rospy.is_shutdown()):
      goal = tf.TransformBroadcaster()
      goal.sendTransform((0.94338, -2.40965, 0.42202),
			(quat[0], quat[1], quat[2], quat[3]),
			rospy.Time.now(),
			self.active_frame,
			self.ref_frame)
      rate.sleep()



if __name__ == '__main__':
    try:
        goal_broadcast()
    except rospy.ROSInterruptException: pass
