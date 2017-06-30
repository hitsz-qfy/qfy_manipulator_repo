#include "ros/ros.h"
#include "std_srvs/Trigger.h"

bool trigger_service(std_srvs::Trigger::Request  &req,
         std_srvs::Trigger::Response &res)
{
  res.success = true;
  res.message = "hello";
  ROS_INFO("The response is: %d", (int)res.success);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_manipulator_trigger");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("move_manipulator_trigger", trigger_service);
  ROS_INFO("Ready to trigger");
  ros::spin();

  return 0;
}