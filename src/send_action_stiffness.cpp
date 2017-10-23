// This node is a client node used to send the stiffness to the robot as a ROS Action
// Created by King's College London and Queen Mary University of London, 2017.

// Include all the needed libraries. The non-standard library needed by the code is the libjsoncpp for JSON Array definition and fourbythree_msgs. See Readme file for download instructions.
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <actionlib/client/simple_action_client.h>
#include "fourbythree_msgs/ExecuteInstructionAction.h"
#include <string>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <sstream>

// Initialize the stiffness array
std_msgs::Float64MultiArray stiffness;
// Initialize the client simple action using the fourbythree_msgs data type
typedef actionlib::SimpleActionClient<fourbythree_msgs::ExecuteInstructionAction> Client;
// Initialize variable 'goal' to send stiffness value to the server
fourbythree_msgs::ExecuteInstructionGoal goal;
// Initialize the stiffness variable in the form of JSON String
Json::Value stiff_str;

// This function is used to get the stiffness vector
void get_stiff(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  stiffness = *msg;
}

// This is the main function
int main(int argc, char** argv)
{
  // Initialize ROS Node
  ros::init(argc, argv, "send_action_stiffness");
  ros::NodeHandle n;

  // Used to get the stiffness vector
  ros::Subscriber stiff_sub = n.subscribe("/fourbythree_topics/stiffness/stiffness_vector", 10, get_stiff);
  // Reset the flag variable
  unsigned char flag_action = 0;

  // Clear the stiffness array
  stiffness.data.clear();
  // Initialize stiffness array with zero value
  for(unsigned char i=0; i<3; i++)
  {
    stiffness.data.push_back(0);
  }

  // Define the type of data as 'STIFFNESS' to be sent to the server
  goal.type = "STIFFNESS";
  // Initialize the client variable under the name 'execute_sm_instruction'
  Client client("execute_sm_instruction", true);
  // Wait until Server under the name 'execute_sm_instruction' is ready
  client.waitForServer();

  // Check whether ROS is running and the flag_action is zero
  while(flag_action == 0 and ros::ok())
  {
    // Set the flag
    flag_action = 1;

    // Clear the stiffness JSON array variable
    stiff_str.clear();
    // Add the stiffness value into JSON array
    for(unsigned char i=0; i<3; i++)
    {
      stiff_str.append(stiffness.data[i]);
    }

    // Define the string stream variable used to change the JSON array into string format
    std::stringstream ss;
    // Transformed the JSON Array into string format
    ss << stiff_str;
    // Add the string in JSON format to goal parameter to be sent
    goal.parameters = ss.str();

    // Send the stiffness values written as string in JSON format to server
    client.sendGoal(goal);

    // Wait for response from server
    client.waitForResult();
    // Check whether the data transfer to server is succeeded
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      // If so, reset the flag. Otherwise, the while loop will not be running.
      flag_action = 0;
    }
    // Spin the loop once
    ros::spinOnce();
  }
  return 0;
}
