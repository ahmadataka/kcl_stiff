/* This node is used to record data for experiment */

#include <ros/ros.h>
#include <iostream>
#include <string>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "baxter_core_msgs/JointCommand.h"
#include "kcl_stiff/record_data.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "baxter_core_msgs/SEAJointState.h"
#include <armadillo>
#include <math.h>
using namespace std;
using namespace arma;

// Define the Global Variables
// Used to send the recorded data
kcl_stiff::record_data data_sent;
// Define the robot's pose, target vector, and tip force estimate
geometry_msgs::Vector3 pose_vector, target_vector, force;
// Define the robot's pose and target as Pose() data type
geometry_msgs::Pose pose, target;
// Used to get the joint torque of the robot
sensor_msgs::JointState joint_torque;
// Used to get the Jacobian and gravity matrix of the robot
mat JFull, Gr;
// Used as flags to check whether the data has been received
int flag_jacobi, flag_torque, flag_gravity;
// Used to get the torque command as a control signal
baxter_core_msgs::JointCommand torque_c;

// Used to get the joint torque
void get_torque(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Get the joint torque
  joint_torque = *msg;
  // Set the array numbers of the data
  unsigned char bound;
  bound = 17;
  // Initialize the torque data
  double torque_data[7] = {0, 0, 0, 0, 0, 0, 0};
  // Clear the torque class array used to send the torque values
  data_sent.torque.command.clear();
  // Get the torque for each joint
  for(unsigned char i=0; i<bound; i++)
  {
      if(joint_torque.name[i] == "left_s0") torque_data[0] = joint_torque.effort[i];
      else if(joint_torque.name[i] == "left_s1") torque_data[1] = joint_torque.effort[i];
      else if(joint_torque.name[i] == "left_e0") torque_data[2] = joint_torque.effort[i];
      else if(joint_torque.name[i] == "left_e1") torque_data[3] = joint_torque.effort[i];
      else if(joint_torque.name[i] == "left_w0") torque_data[4] = joint_torque.effort[i];
      else if(joint_torque.name[i] == "left_w1") torque_data[5] = joint_torque.effort[i];
      else if(joint_torque.name[i] == "left_w2") torque_data[6] = joint_torque.effort[i];
  }
  // Put the torque values into array
  for(unsigned char i =0; i<7; i++)
  {
    data_sent.torque.command.push_back(torque_data[i]);
  }
  // Set the flag
  flag_torque = 1;
}

// This function is used to get the control signal
void get_control_torque(const baxter_core_msgs::JointCommand::ConstPtr& msg)
{
  torque_c = *msg;
}

// This function is used to get the stiffness vector
void get_stiff(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  data_sent.stiffness_vector = *msg;
}

// This function is used to get the distance vector between robot and human
void get_dist(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  data_sent.distance_vector = *msg;
}

// This function is used to get the pose of the robot's tip
void get_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
  // Get the tip's pose
  pose = *msg;
  // Put the tip's position in a Vector3 message format
  pose_vector.x = pose.position.x; pose_vector.y = pose.position.y; pose_vector.z = pose.position.z;
}

// This function is used to get the target for the robot's tip
void get_target(const geometry_msgs::Pose::ConstPtr& msg)
{
  // Get the tip's target
  target = *msg;
  // Put the tip's target in a Vector3 message format
  target_vector.x = target.position.x; target_vector.y = target.position.y; target_vector.z = target.position.z;
}

// This funtion is used to get the Jacobian
void get_jacob(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  // Get the Jacobian
  std_msgs::Float64MultiArray jacob;
  jacob = *msg;
  // Put the Jacobian into matrix
  JFull << jacob.data[0] << jacob.data[1] << jacob.data[2] << jacob.data[3] << jacob.data[4] << jacob.data[5] << jacob.data[6] << endr
	<< jacob.data[7] << jacob.data[8] << jacob.data[9] << jacob.data[10] << jacob.data[11] << jacob.data[12] << jacob.data[13] << endr
	<< jacob.data[14] << jacob.data[15] << jacob.data[16] << jacob.data[17] << jacob.data[18] << jacob.data[19] << jacob.data[20] << endr;
  // Set the flag
  flag_jacobi = 1;
}

// This function is used to get the gravity matrix term
void get_gravity(const baxter_core_msgs::SEAJointState::ConstPtr& msg)
{
  // Get the gravity
  baxter_core_msgs::SEAJointState variable;
  variable = *msg;
  // Put the gravity terms into matrix
  Gr << variable.gravity_model_effort[0] << endr
     << variable.gravity_model_effort[1] << endr
     << variable.gravity_model_effort[2] << endr
     << variable.gravity_model_effort[3] << endr
     << variable.gravity_model_effort[4] << endr
     << variable.gravity_model_effort[5] << endr
     << variable.gravity_model_effort[6] << endr;
  // Set the flag
  flag_gravity = 1;
}

int main(int _argc, char **_argv)
{

  // Initialize the ROS Node
  ros::init(_argc, _argv, "record");
  // Initilize the local vector and matrix variables
  mat tip_force, torque_mat, Jac_trans, Jac_pinv;
  // Create ROS node and init
  ros::NodeHandle n;
  // Set the rate to be 40 Hz
  ros::Rate loop_rate(40);

  // Define the ROS Subscriber
  // Used to get the robot's joint state
  ros::Subscriber torque_sub = n.subscribe("/robot/joint_states", 10, get_torque);
  // Used to get the control signal
  ros::Subscriber torque_c_sub = n.subscribe("/robot/limb/left/joint_command", 10, get_control_torque);
  // Used to get the stiffness vector
  ros::Subscriber stiff_sub = n.subscribe("/fourbythree_topics/stiffness/stiffness_vector", 10, get_stiff);
  // Used to get the closest distance vector between human and robot
  ros::Subscriber dist_sub = n.subscribe("/fourbythree_topics/stiffness/closest_distance/vector", 10, get_dist);
  // Used to get the Baxter's current pose
  ros::Subscriber pose_sub = n.subscribe("/fourbythree_topics/stiffness/baxter/pose/current", 10, get_pose);
  // Used to get the Baxter's desired pose
  ros::Subscriber target_sub = n.subscribe("/fourbythree_topics/stiffness/baxter/pose/desired", 10, get_target);
  // Used to get the Baxter's jacobian
  ros::Subscriber jacob_sub = n.subscribe("/fourbythree_topics/stiffness/baxter/jacobian", 10, get_jacob);
  // Used to get the gravity compensation term
  ros::Subscriber sub_gravity = n.subscribe("/robot/limb/left/gravity_compensation_torques", 10, get_gravity);

  // Define the ROS Publisher
  // Used to publish the record data
  ros::Publisher recorded = n.advertise<kcl_stiff::record_data>("/fourbythree_topics/stiffness/record_data",10);
  // Used to publish the error flag of the robot's tip
  ros::Publisher flag_pub = n.advertise<std_msgs::Float64>("/fourbythree_topics/stiffness/error_flag",10);

  // Initialize the pose, target, and force
  pose.position.x = 0; pose.position.y = 0; pose.position.z = 0;
  target.position.x = 0; target.position.y = 0; target.position.z = 0;
  force.x =0; force.y = 0; force.z =0;

  // Define the variable used to send an error
  std_msgs::Float64 error_sent;

  while (ros::ok())
  {
    // Clear the force class array
    data_sent.force.clear();

    // Check whether the Jacobi and torque have been received
    if(flag_jacobi == 1 and flag_torque==1)
    {
      // Calculate the error between current pose and desired pose
      double error = pow((pow(pose.position.x-target.position.x,2)+pow(pose.position.y-target.position.y,2)+pow(pose.position.z-target.position.z,2)),0.5);
      // If error is big, set the error flag, otherwise reset the error flag
      if(error>0.20) error_sent.data = 1;
      else error_sent.data = 0;
    }

    // Check whether the Jacobi, torque, and gravity term have been received
    if(flag_jacobi == 1 and flag_torque==1 and flag_gravity==1)
    {
      // Get the external torque from the difference between real torque and superposition of control signal plus gravity term
      torque_mat << data_sent.torque.command[0] - torque_c.command[0] - Gr(0,0) << endr
                 << data_sent.torque.command[1] - torque_c.command[1] - Gr(1,0) << endr
                 << data_sent.torque.command[2] - torque_c.command[2] - Gr(2,0) << endr
                 << data_sent.torque.command[3] - torque_c.command[3] - Gr(3,0) << endr
                 << data_sent.torque.command[4] - torque_c.command[4] - Gr(4,0) << endr
                 << data_sent.torque.command[5] - torque_c.command[5] - Gr(5,0) << endr
                 << data_sent.torque.command[6] - torque_c.command[6] - Gr(6,0) << endr;

      // Calculate the external force at the tip
      Jac_trans = (JFull.t());
      Jac_pinv = pinv(Jac_trans);
      tip_force = Jac_pinv*torque_mat;

      // Put the force into Vector3 message
      force.x = tip_force(0,0); force.y = tip_force(1,0); force.z = tip_force(2,0);
    }

    // Put the pose, target, and force into vector3 array to be published
    data_sent.force.push_back(pose_vector);
    data_sent.force.push_back(target_vector);
    data_sent.force.push_back(force);

    // Record the current time
    data_sent.now = ros::Time::now();
    // Publish the recorded data and error flag
    recorded.publish(data_sent);
    flag_pub.publish(error_sent);

    // Spin and sleep until 1/40 Hz
    ros::spinOnce();
    loop_rate.sleep();
  }
}
