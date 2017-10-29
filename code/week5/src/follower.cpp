#include <ros/ros.h>
#include <hal_msgs/speed_pair.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

ros::Publisher motor_pub;
std::string topic_name;

void linkNameCallback(const std_msgs::String::ConstPtr& msg)
{
  topic_name = msg->data;
}

/*
void originCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  geometry_msgs::Point position;
  if (tf_listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(3.0)))
  {
    tf_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
  }
}
*/

int main(int argc, char** argv) {
  ros::init(argc, argv, "follower_2");

  ros::NodeHandle nh;

  motor_pub = nh.advertise<hal_msgs::speed_pair>("/motors", 1);

  ros::Subscriber link_sub = nh.subscribe("/link", 1, linkNameCallback);

  ros::Rate loop_rate(10);

  while(ros::ok()) {
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    if (tf_listener.waitForTransform("/base_link", "/ball", ros::Time(0), ros::Duration(3.0)))
    {
      tf_listener.lookupTransform("/base_link", "/ball", ros::Time(0), transform);
      double roll, pitch, yaw;
      tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
      ROS_INFO_STREAM(transform.getOrigin().getX());
      ROS_INFO_STREAM(roll << " " << pitch << " " << yaw);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
