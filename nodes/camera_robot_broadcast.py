#!/usr/bin/env python
# Created by King's College London and Queen Mary University of London, 2017.
# This node is used to nitialize the Kinect's position and orientation with respect to the robot's base
import roslib; roslib.load_manifest('kcl_stiff')
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform
import tf
import math

class goal_broadcast(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('camera_baxter_broadcast')
    # Define the reference frame which is the base of the robot in this case
    self.ref_frame = "base"
    # Define the frame to be transformed which is the frame of Kinect Camera in this case
    self.active_frame = "openni_link"

    # Transformed degree to radian
    deg_to_rad = math.pi/180.0

    # Read the position of the Camera with respect to the robot from camera callibration stage done by Tekniker
    x_camera = 0.94338
    y_camera = -2.40965
    z_camera = 0.42202

    # Read the orientation (Euler angle) of the Camera with respect to the robot from camera callibration stage done by Tekniker
    yaw_camera = 2.54857
    pitch_camera = -0.02827
    roll_camera = -0.01238

    # Transform the Euler angle into quaternion
    quat = tf.transformations.quaternion_from_euler(roll_camera, pitch_camera, yaw_camera)
    # Set the rate to be 10 Hz
    rate = rospy.Rate(10.0)

    while (not rospy.is_shutdown()):
      # Initialize the TF broadcaster
      goal = tf.TransformBroadcaster()
      # Transform the camera frame with respect to the base frame of the robot with the callibration value
      goal.sendTransform((x_camera, y_camera, z_camera),
			(quat[0], quat[1], quat[2], quat[3]),
			rospy.Time.now(),
			self.active_frame,
			self.ref_frame)

      # Sleep until 1/10 ss
      rate.sleep()



if __name__ == '__main__':
    try:
        goal_broadcast()
    except rospy.ROSInterruptException: pass
