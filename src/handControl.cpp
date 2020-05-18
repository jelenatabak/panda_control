#include "ros/ros.h"
#include "std_msgs/Int32.h"

void readyCallback(const std_msgs::Int32::ConstPtr& msg)
{
  // sad zovi funkciju za impedantno upravljanje
  ROS_INFO("I heard: [%d]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "handControl");
  ros::NodeHandle n;

  // ovdje otvoriti hvataljku

  ros::Subscriber sub = n.subscribe("/grasp_ready", 1000, readyCallback);
  ros::spin();
  return 0;
}
