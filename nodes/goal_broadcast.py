#!/usr/bin/env python
# This node is used to broadcast a goal frame for the tip of the robot

import roslib; roslib.load_manifest('kcl_stiffness')
import rospy
from geometry_msgs.msg import Pose
import tf


class goal_broadcast(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('goal_broadcast')
    # Get the topic name to which the node will subscribe from ROS parameter
    topic = rospy.get_param('~topic')
    # Get the name of the reference frame for the Baxter's target from ROS parameter
    self.ref_frame = rospy.get_param('~frame')
    # Define the ROS Subscriber used to get the Baxter's target
    goal_sub = rospy.Subscriber(topic, Pose, self.get_goal)

    # Spin and wait for the data to be received
    rospy.spin()

  # This function is used to receive the target data
  def get_goal(self,msg):
    # Initialize the ROS TF Broadcaster
    goal = tf.TransformBroadcaster()
    # Broadcast the TF frame 'goal_compi' whose poses are received from the topic with respect to reference frame
    goal.sendTransform((msg.position.x,msg.position.y,msg.position.z),
		      (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
		      rospy.Time.now(),
		      "goal_compi",
		      self.ref_frame)

if __name__ == '__main__':
    try:
        goal_broadcast()
    except rospy.ROSInterruptException: pass
