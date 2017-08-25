#include <iostream>
#include "ros/ros.h"
#include <termios.h>
#include "std_msgs/Float64.h"

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "keyboard");
	ros::NodeHandle nh;
    ros::Publisher pub_keyboard = nh.advertise<std_msgs::Float64>("/time_to_grasp", 100);
    std_msgs::Float64 time_to_grasp;
    time_to_grasp.data = 0.;
    ROS_INFO_ONCE("Press a to grasp, q to quit!!");
	while (ros::ok())
    {
		int c = getch();   // call your non-blocking input function
  		if (c == 'a')
        {
            ROS_WARN("WARNNING: Time to grasp!!!");
            time_to_grasp.data = 1.0;
        }
        if (c == 'b')
        {
            ROS_INFO("Flag clear!!!");
            time_to_grasp.data = 0.0;
        }
        if (c == 'q')
            ros::shutdown();
        pub_keyboard.publish(time_to_grasp);
	}
}
