#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "Test Run Behavior Tree");
  ros::NodeHandle n;

  ROS_INFO_STREAM("HELLO WORLD");

  return 0;
}
