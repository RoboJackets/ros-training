#include <ros/ros.h>

#include <week_slam/slam_node.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week_slam");

  SlamNode slam_node;
  ros::spin();
}
