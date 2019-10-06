#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher g_velocity_pub;
ros::Publisher g_error_pub;

geometry_msgs::PoseStamped g_kyle_pose;
ros::Time g_last_time;

double g_kp;

void kylePoseCallback(const geometry_msgs::PoseStamped& msg)
{
  g_kyle_pose = msg;
}

void oswinPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  if (g_last_time.sec == 0)
  {
    g_last_time = ros::Time::now();
    return;
  }

  double error = g_kyle_pose.pose.position.x - msg.pose.position.x;
  ros::Duration delta_t = msg.header.stamp - g_last_time;

  double proportional = error;
  double control = g_kp * proportional;

  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = control;
  g_velocity_pub.publish(twist_msg);

  g_last_time = msg.header.stamp;

  std_msgs::Float64 error_msg;
  error_msg.data = error;
  g_error_pub.publish(error_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week3");

  ros::NodeHandle nh;
  ros::NodeHandle pnh{"~"};

  pnh.getParam("kp", g_kp);

  g_velocity_pub = nh.advertise<geometry_msgs::Twist>("oswin/velocity", 1);
  g_error_pub = nh.advertise<std_msgs::Float64>("error", 1);

  ros::Subscriber kyle_sub = nh.subscribe("kyle/ground_truth", 1, kylePoseCallback);
  ros::Subscriber oswin_sub = nh.subscribe("oswin/ground_truth", 1, oswinPoseCallback);
  ros::spin();
}
