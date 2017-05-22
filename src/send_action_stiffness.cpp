#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <actionlib/client/simple_action_client.h>
#include "fourbythree_msgs/ExecuteInstructionAction.h"
#include <sstream>

std_msgs::Float64MultiArray stiffness;
typedef actionlib::SimpleActionClient<fourbythree_msgs::ExecuteInstructionAction> Client;

// This function is used to get the stiffness vector
void get_stiff(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  stiffness = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_action_stiffness");
  ros::NodeHandle n;
  Client client("execute_sm_instruction", true); // true -> don't need ros::spin()
  // Used to get the stiffness vector
  ros::Subscriber stiff_sub = n.subscribe("/fourbythree_topics/stiffness/stiffness_vector", 10, get_stiff);

  client.waitForServer();

  fourbythree_msgs::ExecuteInstructionGoal goal;
  goal.type = "STIFFNESS";
  // std::ostringstream stiff_string;
  // stiff_string << "[" << stiffness.data[0] << ", " << stiffness.data[1] << ", " << stiffness.data[2] << "]";
  // goal.parameters = stiff_string.str();
  goal.parameters = "GOAL";
  // sprintf(goal.parameters,"[%f, %f, %f]", stiffness.data[0], stiffness.data[1], stiffness.data[2]);
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
