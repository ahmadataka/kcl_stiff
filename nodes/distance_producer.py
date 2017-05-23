#!/usr/bin/env python
# This code is used to produce distance information between points on the robot and points on the human body
import roslib
roslib.load_manifest('kcl_stiff')
import rospy
import tf
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    # Initialize the ROS Node
    rospy.init_node('distance_producer')
    # Get the human index parameter
    index = rospy.get_param('~ind')

    # Define the PoseArray message
    pose_frame = PoseArray()
    # Define the Pose Message for reading the frame of robot and human body
    pose_dummy = Pose()
    # Define the PoseArray message used to publish the human-robot distance
    pose_sent = PoseArray()
    # Define the listener used to listen to the robot and human's frame
    listener = tf.TransformListener()

    # Define the ROS Publisher used to send the robot-human distance
    tip_pose = rospy.Publisher('/fourbythree_topics/stiffness/human_robot_distance', PoseArray, queue_size = 10)

    # Define the base frame
    base_frame = 'torso'
    # Initialize empty array used to save the name of the robot's frame
    frame_robot = []
    # Push back the name of the robot's frame
    frame_robot.append('left_hand_camera')
    frame_robot.append('left_lower_forearm')
    frame_robot.append('left_wrist')
    # Enter the value for the number of robot's frame used
    number_robot = 3

    # Initialize empty array used to save the name of the human's frame
    frame_human = []
    # Push back the name of the human's frame
    frame_human.append('left_elbow'+'_'+str(index))
    frame_human.append('left_shoulder'+'_'+str(index))
    frame_human.append('left_hand'+'_'+str(index))
    # Enter the value for the number of human's frame used
    number_human = 3

    # Set the rate of the code to be 40 Hz
    rate = rospy.Rate(40.0)

    while not rospy.is_shutdown():
        # Initialize the PoseArray variables
    	pose_frame.poses = []
    	pose_sent.poses = []

        # Listen to the frames on the robot body
        for i in range(0,number_robot):
            # Initialize Pose variable
            pose_dummy = Pose()
            # Try the listener
            try:
                (trans,rot) = listener.lookupTransform(base_frame, frame_robot[i], rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # Put the values
            pose_dummy.position.x = trans[0]
            pose_dummy.position.y = trans[1]
            pose_dummy.position.z = trans[2]
            # Put the Pose into the PoseArray
            pose_frame.poses.append(pose_dummy)

        # Listen to the frames on the human body
    	for i in range(0,number_human):
          # Try the listener
          try:
    	      (trans,rot) = listener.lookupTransform(base_frame, frame_human[i], rospy.Time(0))
    	  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    	      continue
    	  # Initialize Pose variable
    	  pose_dummy = Pose()
          # Put the values
    	  pose_dummy.position.x = trans[0]
    	  pose_dummy.position.y = trans[1]
    	  pose_dummy.position.z = trans[2]
          # Put the Pose into the PoseArray
    	  pose_frame.poses.append(pose_dummy)

        # Check the length of the Pose array
    	if(len(pose_frame.poses) == number_robot+number_human):
          # Execute when the length of Pose array equals the total number of frame on the robot and human body
          for i in range(0,number_robot):
    	    for j in range(0,number_human):
              # Initialize the Pose variable
              pose_dummy = Pose()
              # Find the distance vector for every possible frame pairs on the robot and human body
    	      pose_dummy.position.x = pose_frame.poses[i].position.x - pose_frame.poses[number_robot+j].position.x
    	      pose_dummy.position.y = pose_frame.poses[i].position.y - pose_frame.poses[number_robot+j].position.y
    	      pose_dummy.position.z = pose_frame.poses[i].position.z - pose_frame.poses[number_robot+j].position.z
    	      # Put the distance information as a Pose Array format
    	      pose_sent.poses.append(pose_dummy)
          # Publish the distance vector array
    	  tip_pose.publish(pose_sent)

        # Sleep until 1/40 Hz
        rate.sleep()
