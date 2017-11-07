Dependencies:
- numpy python library
- libjsoncpp C++ library
- fourbythree_msgs ROS package (provided by Tekniker)
- OpenNI ROS package (provided by Tekniker)
- OpenNI Tracker ROS package (provided by Tekniker)

PREPARE KINECT
1. Make the Kinect is on and connected to the computer
2. Open a new terminal and run the Kinect:
roslaunch openni_launch openni.launch camera:=openni
3. Open New Terminal and run the human skeleton tracker:
roslaunch openni_tracker openni_skeleton_tracker.launch
4. The human needs to stand up in front of the camera. Wait for the frame in R-Viz to be available. Check the human index and remember it for the next section. For example, if the left elbow for example is detected as ‘left_elbow_2’, then the human index is ‘2’. Make sure to check which arm is assigned as “left” in the R-Viz. It is possible for your right arm to be assigned as “left”. Only “left” arm can be used both in the Stiffness Adjustment and Ergonomics part.

STIFFNESS ADJUSTMENT
1. Open New Terminal and run the stiffness adjustment:
roslaunch kcl_stiff launch_stiffnesss.launch index:=1
The argument 'index' is assigned according to the human index in R-Viz.
2. If you want to see the stiffness, subscribe to this topic in the new terminal:
/fourbythree_topics/stiffness/stiffness_vector/
