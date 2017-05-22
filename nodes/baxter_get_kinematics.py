#!/usr/bin/python
# This node is used to get the kinematics information (Jacobian, forward kinematics, etc.) of the Baxter
import rospy
import numpy as np
import PyKDL
from baxter_pykdl import baxter_kinematics
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

class cart_control(object):
  def __init__(self):
    # Initialize the ROS Node
    rospy.init_node('baxter_get_kinematics')
    # Initialize the ROS Publisher
    # Used to publish the robot's jacobian of the tip
    pub = rospy.Publisher('/fourbythree_topics/stiffness/baxter/jacobian', Float64MultiArray, queue_size=10)
    # Used to publish the the tip's position of the robot
    pose_pub = rospy.Publisher('/fourbythree_topics/stiffness/baxter/pose/current', Pose, queue_size=10)
    # Used to publish the Euler angle of the tip
    euler_pub = rospy.Publisher('/fourbythree_topics/stiffness/baxter/angle/current', Float64MultiArray, queue_size=10)

    # Define the kinematics of the Baxter's left hand
    self.kin_gripper = baxter_kinematics('left','hand')
    # Define the number of joints for Baxter
    num_joint = 7

    # Define the Pose data
    pose_sent = Pose()
    # Define the Float array data used to send Jacobian matrix
    Jacobi_sent = Float64MultiArray()
    # Define the Float array data used to send the euler angle
    self.euler = Float64MultiArray()

    while not rospy.is_shutdown():
      # Get the Jacobian
      JGripper = self.kin_gripper.jacobian()
      # Get the current tip position
      pose = self.kin_gripper.forward_position_kinematics()
      # Define the quaternion type of data
      quat_ros = Quaternion()
      # Get the quaternion's components
      quat_ros.x = pose[3]
      quat_ros.y = pose[4]
      quat_ros.z = pose[5]
      quat_ros.w = pose[6]
      # Transform the quaternion into Euler angles
      self.quat_to_angle(quat_ros)

      # Empty the array
      Jacobi_sent.data = []
      for i in range(0,6):
    	for j in range(0,num_joint):
          # Put the component of the Jacobian matrix into the array
    	  Jacobi_sent.data.append(JGripper[i,j])

      # Put the pose into the Pose() data type to be sent
      pose_sent.position.x = pose[0]
      pose_sent.position.y = pose[1]
      pose_sent.position.z = pose[2]
      pose_sent.orientation.x = pose[3];
      pose_sent.orientation.y = pose[4];
      pose_sent.orientation.z = pose[5];
      pose_sent.orientation.w = pose[6];

      # Publish the data
      pub.publish(Jacobi_sent)
      pose_pub.publish(pose_sent)
      euler_pub.publish(self.euler)


  # This function is used to transform quaternion into Euler angle array
  def quat_to_angle(self, quat):
    # Transform quaternion into Euler angle
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    # Put the Euler angle into array
    self.euler.data = []
    self.euler.data.append(rot.GetEulerZYX()[0])
    self.euler.data.append(rot.GetEulerZYX()[1])
    self.euler.data.append(rot.GetEulerZYX()[2])

if __name__ == "__main__":
    try:
        cart_control()
    except rospy.ROSInterruptException: pass
