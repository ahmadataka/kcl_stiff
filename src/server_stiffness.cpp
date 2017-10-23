// This node is a server node used to retrieve the stiffness data from a ROS Action. Not used in the real system. Used only for testing and troubleshooting.
// Created by King's College London and Queen Mary University of London, 2017.

// Include all the needed libraries. The non-standard library needed by the code is fourbythree_msgs. See Readme file for download instructions.
#include <ros/ros.h>
#include "fourbythree_msgs/ExecuteInstructionAction.h"
#include <actionlib/server/simple_action_server.h>
#include <iostream>

// Use the namespace
using namespace std;

// Initialize the server simple action using the fourbythree_msgs data type
typedef actionlib::SimpleActionServer<fourbythree_msgs::ExecuteInstructionAction> Server;

// This function is used when the ROS Action from the client is received
void execute(const fourbythree_msgs::ExecuteInstructionGoalConstPtr& goal, Server* as)
{
  // Set the flag to be succeeded so that the client can continue sending data
  as->setSucceeded();
  // Initialize the data to read stiffness value
  fourbythree_msgs::ExecuteInstructionGoal testing;
  // Read the stiffness data from ROS Action
  testing = *goal;
  // Print the value to the screen
  cout << testing.parameters << endl;
}

// This is the main function
int main(int argc, char** argv)
{
  // Initialize ROS Node
  ros::init(argc, argv, "server_stiffness");
  ros::NodeHandle n;

  // Initialize the server variable under the name 'execute_sm_instruction'
  Server server(n, "execute_sm_instruction", boost::bind(&execute, _1, &server), false);
  // Start the server and let the client knows that the server is ready
  server.start();
  // Spin the loop
  ros::spin();
  return 0;
}
