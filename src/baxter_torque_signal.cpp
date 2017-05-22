/*This node is used to do torque control for Baxter*/

#include <ros/ros.h>
#include <iostream>
#include <string>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Joy.h"
#include "baxter_core_msgs/JointCommand.h"
#include "baxter_core_msgs/SEAJointState.h"
#include <armadillo>
#include <math.h>

// using std::string;
using namespace std;
using namespace arma;

// Define the global variables
// Used to get the joint angle of the robot
sensor_msgs::JointState joint_angle;
// Get the input from keyboard or Joystick
sensor_msgs::Joy joy_msg;
// Flags are used to check whether some of the subscribed data have been received
unsigned char flag_jacobi, flag_euler, flag_target, flag_move;
// Matrix variables used to get the value of Jacobians
mat Jv, Jw, JFull;
// Vector variables used to get the current pose and desired pose for static and moving cases
mat pose,  pose_d, pose_d_tf;
// Vector variables used as current Euler angles and desired Euler angles
mat angle, angle_d;
// Vector variables used as the rate of joint angles and the angular velocity
mat qdot_vec, angledot_vec;
// Used to get the position change from keyboard/Joystick
mat joy_mat;
// Used as a proportional constant for position and rotation control
geometry_msgs::Vector3 KP, KP_rot;
// Used to get the ROS parameters
int real_baxter, controller, joy_node, KP0;
// Used to get the target for the Baxter's tip
geometry_msgs::Pose ee_target;

double qdot[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0};

// This function is used to get the joint angle and joint angle speed
void get_joint(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Get the joint angle
  joint_angle = *msg;
  // Initialize the joint angle speed
  qdot_vec << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr;

  // Set the number of arrays in the message according to the use of either simulation or real Baxter
  unsigned char bound;
  if(real_baxter == 0) bound = 19;
  else bound = 17;

  // For each corresponding joint, get the joint angle speed
  for(unsigned char i=0; i<bound; i++)
  {
      if(joint_angle.name[i] == "left_s0") qdot_vec(0,0) = joint_angle.velocity[i];
      else if(joint_angle.name[i] == "left_s1") qdot_vec(1,0) = joint_angle.velocity[i];
      else if(joint_angle.name[i] == "left_e0") qdot_vec(2,0) = joint_angle.velocity[i];
      else if(joint_angle.name[i] == "left_e1") qdot_vec(3,0) = joint_angle.velocity[i];
      else if(joint_angle.name[i] == "left_w0") qdot_vec(4,0) = joint_angle.velocity[i];
      else if(joint_angle.name[i] == "left_w1") qdot_vec(5,0) = joint_angle.velocity[i];
      else if(joint_angle.name[i] == "left_w2") qdot_vec(6,0) = joint_angle.velocity[i];
  }
}

// This function is used to get the input from keyboard/Joystick
void get_joy(const sensor_msgs::Joy::ConstPtr& msg)
{
  // Define the scaling factor
  float ds = 0.05;
  // Read the keyboard/Joystick data
  joy_msg = *msg;
  // Change the data into positional changes
  joy_mat << joy_msg.axes[0]*ds << endr
	  << joy_msg.axes[1]*ds << endr
	  << joy_msg.axes[3]*ds << endr;

  // Check whether the joystick is activated. If so, update the target.
  if(joy_node == 1) pose_d_tf = pose_d_tf + joy_mat;

  // Check the keyboard 'x' or 'z' to update the modes: either static or moving
  if(joy_msg.axes[2]==1)
  {
    flag_move = 1;
  }
  else if(joy_msg.axes[2]==-1)
  {
    flag_move = 0;
  }
}

// This function is used to get the stiffness vector
void get_stiff(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  std_msgs::Float64MultiArray stiff_data;
  // Get the stiffness vector
  stiff_data = *msg;
  // Put the stiffness data into a Vector3 data
  KP.x = stiff_data.data[0];
  KP.y = stiff_data.data[1];
  KP.z = stiff_data.data[2];
}

// This function is used to get the Jacobian matrix
void get_jacob(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  std_msgs::Float64MultiArray jacob;
  // Get the Jacobian data
  jacob = *msg;
  // Put the Jacobian into matrix
  JFull << jacob.data[0] << jacob.data[1] << jacob.data[2] << jacob.data[3] << jacob.data[4] << jacob.data[5] << jacob.data[6] << endr
	<< jacob.data[7] << jacob.data[8] << jacob.data[9] << jacob.data[10] << jacob.data[11] << jacob.data[12] << jacob.data[13] << endr
	<< jacob.data[14] << jacob.data[15] << jacob.data[16] << jacob.data[17] << jacob.data[18] << jacob.data[19] << jacob.data[20] << endr
	<< jacob.data[21] << jacob.data[22] << jacob.data[23] << jacob.data[24] << jacob.data[25] << jacob.data[26] << jacob.data[27] << endr
	<< jacob.data[28] << jacob.data[29] << jacob.data[30] << jacob.data[31] << jacob.data[32] << jacob.data[33] << jacob.data[34] << endr
	<< jacob.data[35] << jacob.data[36] << jacob.data[37] << jacob.data[38] << jacob.data[39] << jacob.data[40] << jacob.data[41] << endr;
  // Set the flag
  flag_jacobi = 1;
}

// This function is used to get the pose of the tip
void get_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
  geometry_msgs::Pose pose_var;
  // Get the pose
  pose_var = *msg;
  // Put the pose into vector
  pose << pose_var.position.x << endr
       << pose_var.position.y << endr
       << pose_var.position.z << endr;
}

// This function is used to get the Euler angle of the tip
void get_euler(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  std_msgs::Float64MultiArray euler_var;
  // Get the Euler angle
  euler_var = *msg;
  // Put the angles into vectors
  angle << euler_var.data[0] << endr
	<< euler_var.data[1] << endr
	<< euler_var.data[2] << endr;
  // Set the flag
  flag_euler = 1;
}

// This function is used to get the Baxter's target
void get_target(const geometry_msgs::Pose::ConstPtr& msg)
{
  // Get the target and set the flag
  ee_target = *msg;
  flag_target = 1;
}

// This function is used to transform the Vector3 ROS Data into Armadillo-styled Matrix
mat Vector3toArma(geometry_msgs::Vector3 vect)
{
  mat T_arma;
  T_arma << vect.x << 0.0 << 0.0 << endr
	 << 0.0 << vect.y << 0.0 << endr
	 << 0.0 << 0.0 << vect.z << endr;
  return T_arma;
}

// This function is used to build the Matrix to transform the Euler angle rate into Angular Velocity
mat AngletoMatOme(mat ang)
{
  mat T_arma;
  T_arma << -sin(ang[0,1]) << cos(ang[0,1])*sin(ang[0,2]) << cos(ang[0,1])*cos(ang[0,2]) << endr
	 << 0.0 << cos(ang[0,2]) << -sin(ang[0,1]) << endr
	 << 1.0 << 0.0 << 0.0 << endr;
  return T_arma;
}

// This function is used to build the rotation Matrix from Euler angle
mat AngletoR(mat ang)
{
  mat T_arma;
  T_arma << cos(ang[0,0])*cos(ang[0,1]) << cos(ang[0,0])*sin(ang[0,1])*sin(ang[0,2])-sin(ang[0,0])*cos(ang[0,2]) << cos(ang[0,0])*sin(ang[0,1])*cos(ang[0,2])+sin(ang[0,0])*sin(ang[0,2]) << endr
	 << sin(ang[0,0])*cos(ang[0,1]) << sin(ang[0,0])*sin(ang[0,1])*sin(ang[0,2])+cos(ang[0,0])*cos(ang[0,2]) << sin(ang[0,0])*sin(ang[0,1])*cos(ang[0,2])-cos(ang[0,0])*sin(ang[0,2]) << endr
	 << -sin(ang[0,1]) << cos(ang[0,1])*sin(ang[0,2]) << cos(ang[0,1])*cos(ang[0,2]) << endr;

  return T_arma;
}

// This function is used to transform rotation matrix to quaternion
mat RtoQuat(mat T_matrix)
{
  mat T_arma;

  double w, x, y, z;

  float trace = T_matrix(0,0) + T_matrix(1,1) + T_matrix(2,2); // I removed + 1.0f; see discussion with Ethan
  if( trace > 0 ) {// I changed M_EPSILON to 0
    float s = 0.5f / sqrtf(trace+ 1.0f);
    w = 0.25f / s;
    x = ( T_matrix(2,1) - T_matrix(1,2) ) * s;
    y = ( T_matrix(0,2) - T_matrix(2,0) ) * s;
    z = ( T_matrix(1,0) - T_matrix(0,1) ) * s;
  } else {
    if ( T_matrix(0,0) > T_matrix(1,1) && T_matrix(0,0) > T_matrix(2,2) ) {
      float s = 2.0f * sqrtf( 1.0f + T_matrix(0,0) - T_matrix(1,1) - T_matrix(2,2));
      w = (T_matrix(2,1) - T_matrix(1,2) ) / s;
      x = 0.25f * s;
      y = (T_matrix(0,1) + T_matrix(1,0) ) / s;
      z = (T_matrix(0,2) + T_matrix(2,0) ) / s;
    } else if (T_matrix(1,1) > T_matrix(2,2)) {
      float s = 2.0f * sqrtf( 1.0f + T_matrix(1,1) - T_matrix(0,0) - T_matrix(2,2));
      w = (T_matrix(0,2) - T_matrix(2,0) ) / s;
      x = (T_matrix(0,1) + T_matrix(1,0) ) / s;
      y = 0.25f * s;
      z = (T_matrix(1,2) + T_matrix(2,1) ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + T_matrix(2,2) - T_matrix(0,0) - T_matrix(1,1) );
      w = (T_matrix(1,0) - T_matrix(0,1) ) / s;
      x = (T_matrix(0,2) + T_matrix(2,0) ) / s;
      y = (T_matrix(1,2) + T_matrix(2,1) ) / s;
      z = 0.25f * s;
    }
  }

  // Build the quaternion vector
  T_arma << x << endr
	 << y << endr
	 << z << endr
	 << w << endr;
  return T_arma;

}

// The main function
int main(int _argc, char **_argv)
{
  // Initialize the ROS Node
  ros::init(_argc, _argv, "torque_control");

  // Create ROS node and init
  ros::NodeHandle n;

  // Get the ROS Parameters
  n.param("real_baxter", real_baxter, real_baxter);
  n.param("controller", controller, controller);
  n.param("joy_node", joy_node, joy_node);
  n.param("stiffness_value", KP0, KP0);

  // Set the rate to be 1000 Hz
  ros::Rate loop_rate(1000);
  // Define the torque data to be sent to the robot
  baxter_core_msgs::JointCommand torque_sent;

  // Define the ROS Subscriber
  // Used to get the robot's joint state
  ros::Subscriber joint_sub = n.subscribe("/robot/joint_states", 10, get_joint);
  // Used to get the robot's current tip position
  ros::Subscriber pose_sub = n.subscribe("/baxter/pose/current", 10, get_pose);
  // Used to get the robot's Jacobian of the tip
  ros::Subscriber jacob_sub = n.subscribe("/baxter/jacobian", 10, get_jacob);
  // Used to get the robot's current Euler angle of the tip
  ros::Subscriber angle_sub = n.subscribe("/baxter/angle/current", 10, get_euler);
  // Used to get the stiffness vector
  ros::Subscriber stiff_sub = n.subscribe("/stiffness_vector", 10, get_stiff);
  // Used to get the input from keyboard/Joystick
  ros::Subscriber goal_sub = n.subscribe("/joy", 10, get_joy);
  // Used to get the Baxter's target for the tip
  ros::Subscriber target_sub = n.subscribe("/baxter_target", 10, get_target);

  // Define the ROS Publisher
  // Used to send the torque signal
  ros::Publisher joint_command = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",10);
  // Used to publish the desired pose
  ros::Publisher pub_pose_desired = n.advertise<geometry_msgs::Pose>("/baxter/pose/desired",10);
  // Used to publish the twist of the Baxter's tip
  ros::Publisher pub_twist = n.advertise<geometry_msgs::Vector3>("/agent/ee/twist",10);

  // Define the variable used to sent the updated desired pose
  geometry_msgs::Pose pose_d_sent;
  // Define the variable used to send the twist of the Baxter's tip
  geometry_msgs::Vector3 twist_sent;

  // Reset the flags
  flag_jacobi = 0; flag_euler = 0;

  // Define the matrix and vector variables
  mat Torque_cart, xdot_vec, KP_mat, KP_rot_mat, force_task, torque_task, F_task, Matrix_Omega, quat_d, R_d;

  // Initialize the joint angle speed
  qdot_vec << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr
	   << 0.0 << endr;

  // Initialize the positional stiffness
  KP.x = (float)(KP0); KP.y = (float)(KP0); KP.z = (float)(KP0);
  // Set the rotational stiffness
  KP_rot.x = 1.0; KP_rot.y = 1.0; KP_rot.z = 1.0;
  // Set the derivative term
  float KD = 1.0;
  // Define the number of joints
  unsigned char num_joint = 7;
  // Reset the flag for initial assignment of target
  unsigned char flag_init = 0;
  // Initialize the desired position
  pose_d_tf.zeros(3,1);

  while (ros::ok())
  {
    // Get the ROS Parameter showing whether the keyboard/Joystick status is used
    n.param("joy_node", joy_node, joy_node);

    // Check whether the Jacobian and the Euler angle have been received
    if(flag_jacobi ==1 && flag_euler == 1)
    {
      // If the robot is in moving mode, update the desired target
      if(flag_move == 1) pose_d = pose_d_tf;

      // Check whether this is the first time the loop is run
      if(flag_init==0)
      {
        // If so, set the desired pose as the current pose
        pose_d_tf = pose;
        pose_d = pose;
        // Set the desired angle
    	  angle_d << 0.5*M_PI << endr
    		        << 0 << endr
    		        << M_PI << endr;
        // Set the flag
    	  flag_init = 1;
      }
      // Check whether the keyboard/Joystick is disabled and target has been received
      if(flag_target == 1 && joy_node == 0)
      {
        // In that case, get the desired position from the target
	       pose_d_tf << ee_target.position.x << endr
	                 << ee_target.position.y << endr
	                 << ee_target.position.z << endr;
      }

      // Get the translational velocity Jacobian
      Jv = JFull;
      Jv.shed_rows(3,5);
      // Get the rotational velocity Jacobian
      Jw = JFull;
      Jw.shed_rows(0,2);

      // Calculate the desired rotational matrix and quaternion
      R_d = AngletoR(angle_d);
      quat_d = RtoQuat(R_d);

      // Calculate the robot's twist via Jacobian
      xdot_vec = Jv*qdot_vec;
      angledot_vec = Jw*qdot_vec;

      // Transform the stiffness into diagonal matrix
      KP_mat = Vector3toArma(KP);
      KP_rot_mat = Vector3toArma(KP_rot);

      // Calculate the matrix used to transform Euler angle rate into angular velocity
      Matrix_Omega = AngletoMatOme(angle);

      // Calculate the task-space attractive translational force
      force_task = KP_mat*(pose - pose_d);

      // Calculate the task-space rotational torque according to the type of controller
      if(controller==1) torque_task = Matrix_Omega*KP_rot_mat*(angle - angle_d);
      else if(controller==2) torque_task = zeros(3,1);
      // Combine the translational and rotational force-torque
      F_task = join_cols(force_task, torque_task);

      // According to the type of controller, calculate the joint torque via transpose of Jacobian
      // Controller 0: Only position control. Rotation is free.
      // Controller 1: Both position and rotation control.
      // Controller 2: Position control combined with zero rotation.
      // Else: Rotation control only
      if(controller == 0) Torque_cart = Jv.t()*(-KP_mat*(pose - pose_d) )- Jv.t()*KD*xdot_vec;
      else if(controller == 1 || controller == 2) Torque_cart = -JFull.t()*F_task - KD*qdot_vec;
      else
      {
	       Torque_cart = - (Jw.t()*(Matrix_Omega*KP_rot_mat*(angle - angle_d) + KD*angledot_vec));
      }

      // Put the desired pose into Pose() data to be sent
      pose_d_sent.position.x = pose_d_tf[0,0];
      pose_d_sent.position.y = pose_d_tf[0,1];
      pose_d_sent.position.z = pose_d_tf[0,2];
      pose_d_sent.orientation.x = quat_d[0,0];
      pose_d_sent.orientation.y = quat_d[0,1];
      pose_d_sent.orientation.z = quat_d[0,2];
      pose_d_sent.orientation.w = quat_d[0,3];

      // Set the torque mode control
      torque_sent.mode = 3;
      // Clear the names and command of the array
      torque_sent.names.clear();
      torque_sent.command.clear();
      // Put the names of the joint into array
      torque_sent.names.push_back("left_s0");
      torque_sent.names.push_back("left_s1");
      torque_sent.names.push_back("left_e0");
      torque_sent.names.push_back("left_e1");
      torque_sent.names.push_back("left_w0");
      torque_sent.names.push_back("left_w1");
      torque_sent.names.push_back("left_w2");
      // Put the value of the torque into the array
      for(unsigned char i=0; i<num_joint; i++)
      {
        torque_sent.command.push_back(Torque_cart[0,i]);
      }

      // Put the twist into Vector3 data to be sent
      twist_sent.x = xdot_vec(0,0);
      twist_sent.y = xdot_vec(1,0);
      twist_sent.z = xdot_vec(2,0);

      // Publish the torque, desired pose, and robot's twist
      joint_command.publish(torque_sent);
      pub_pose_desired.publish(pose_d_sent);
      pub_twist.publish(twist_sent);
    }

    // Spin and sleep until 1/1000 Hz
    ros::spinOnce();
    loop_rate.sleep();
  }
}
