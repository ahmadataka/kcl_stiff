
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <actionlib/client/simple_action_client.h>
#include "fourbythree_msgs/ExecuteInstructionAction.h"
#include <string>
#include <iostream>

std_msgs::Float64MultiArray stiffness;
typedef actionlib::SimpleActionClient<fourbythree_msgs::ExecuteInstructionAction> Client;
std::string opening, commas, closing, stiff_spring, final_stiff;
fourbythree_msgs::ExecuteInstructionGoal goal;
// This function is used to get the stiffness vector
void get_stiff(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  stiffness = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_action_stiffness");
  ros::NodeHandle n;

  // Used to get the stiffness vector
  ros::Subscriber stiff_sub = n.subscribe("/fourbythree_topics/stiffness/stiffness_vector", 10, get_stiff);
  unsigned char flag_action = 0;
  stiffness.data.clear();
  for(unsigned char i=0; i<3; i++)
  {
    stiffness.data.push_back(0);
  }
  opening = "[";
  closing = "]";
  final_stiff = "";
  commas = ", ";

  goal.type = "STIFFNESS";
  Client client("execute_sm_instruction", true);
  client.waitForServer();

  while(flag_action == 0 and ros::ok())
  {

    flag_action = 1;

    final_stiff = "";
    for(unsigned char i=0; i<3; i++)
    {
      stiff_spring = boost::lexical_cast<std::string>(stiffness.data[i]);

      final_stiff = final_stiff + stiff_spring;
      if(i!=2) final_stiff = final_stiff + commas;
    }
    final_stiff = opening + final_stiff + closing;

    goal.parameters = final_stiff;

    client.sendGoal(goal);

    client.waitForResult();
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      flag_action = 0;
    }

    ros::spinOnce();
  }
  return 0;
}
