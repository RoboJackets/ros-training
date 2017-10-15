#include <ros/ros.h>
#include "std_msgs/String.h"
#include <hal_msgs/speed_pair.h>

ros::Publisher motor_pub;

double desired_speed;

void driveCallback(const std_msgs::String::ConstPtr& msg)
{
  hal_msgs::speed_pair speed;
  speed.right_velocity = desired_speed;
  speed.left_velocity = desired_speed;
  if(msg->data == "backward") {
    speed.right_velocity = -desired_speed;
    speed.left_velocity = -desired_speed;
  } else if(msg->data == "right") {
    speed.right_velocity = -desired_speed;
  } else if(msg->data == "left") {
    speed.left_velocity = -desired_speed;
  }
  motor_pub.publish(speed);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "driver");

  ros::NodeHandle nh;

  nh.getParam("desired_speed", desired_speed);

  ros::Subscriber sub = nh.subscribe("drive", 1, driveCallback);

  motor_pub = nh.advertise<hal_msgs::speed_pair>("/motors", 1);

  ros::spin();
}
