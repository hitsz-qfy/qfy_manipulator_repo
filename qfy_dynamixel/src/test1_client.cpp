/*
* @Author: qfyhaha
* @Date:   2017-04-18 16:43:15
* @Last Modified by:   qfyhaha
* @Last Modified time: 2017-04-18 19:59:13
*/

#include <iostream>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("move_manipulator_trigger");
	//ros::Duration(1.0).sleep();
	std_srvs::Trigger trigger;
	if (client.call(trigger))
  	{
   		//ROS_INFO("The Result is :%d", (int)trigger.response.success);
   		ROS_INFO("Call success");
 	}
  	else
  	{
    	ROS_ERROR("Failed to call service move_manipulator_trigger");
    	//return 1;
 	}

    return 0;
}