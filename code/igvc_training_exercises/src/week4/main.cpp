#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// struct to represent Position
struct Position
{
  double x;
  double y;
};

// struct to represent a Pose: a Position + heading
struct Pose
{
  Position position;
  double heading;
};

// struct to represent a twist, ie. a linear / angular velocity
struct Twist
{
  double linear;
  double angular;
};

// struct to represent the State
struct State
{
  Pose pose;
  Twist twist;
};

State g_state; // Current state

ros::Time g_last_time; // Last time we received a message, to calculate dt
ros::Publisher g_odom_pub; // Publisher for odom

/**
 * Callback for imu message
 * @param msg message containing imu
 */
void imuCallback(const sensor_msgs::Imu& msg)
{
  // tf Transform Broadcaster to broadcast tf transforms. We make it static here to make sure that
  // it happens after the first node handle
  static tf::TransformBroadcaster g_broadcaster;

  // Calculate the delta_t from current message to the last message
  ros::Duration delta_t = msg.header.stamp - g_last_time;

  // Set the twist to be equal to the result from the gyroscope
  g_state.twist.angular = msg.angular_velocity.z;

  // The linear velocity is the integral of acceleration
  g_state.twist.linear += msg.linear_acceleration.x * delta_t.toSec();
  // The heading is the integral of angular velocity
  g_state.pose.heading += g_state.twist.angular * delta_t.toSec();

  // The x and y of the position is just the integral of velocity multiplied by cos / sin
  g_state.pose.position.x += cos(g_state.pose.heading) * g_state.twist.linear * delta_t.toSec();
  g_state.pose.position.y += sin(g_state.pose.heading) * g_state.twist.linear * delta_t.toSec();

  g_last_time = msg.header.stamp; // update the last timestamp

  // Fill in the information for the nav_msgs::Odometry msg
  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.frame_id = "odom";
  odometry_msg.header.stamp = msg.header.stamp;
  odometry_msg.pose.pose.position.x = g_state.pose.position.x;
  odometry_msg.pose.pose.position.y = g_state.pose.position.y;
  // We use tf::createQuaternionMsgFromYaw to convert from a heading to a quaternion message
  odometry_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(g_state.pose.heading);
  odometry_msg.twist.twist.linear.x = g_state.twist.linear;
  odometry_msg.twist.twist.angular.z = g_state.twist.angular;
  // And publish it
  g_odom_pub.publish(odometry_msg);

  // Fill in the information for the tf::Transform variable
  tf::Transform transform;
  transform.setOrigin({g_state.pose.position.x, g_state.pose.position.y, 0.0});
  // Use tf::createQuaternionFromYaw to convert from a heading to a tf quaternion
  transform.setRotation(tf::createQuaternionFromYaw(g_state.pose.heading));
  tf::StampedTransform stamped_transform{transform, msg.header.stamp, "odom", "oswin"};
  // So that we can broadcast it for other nodes to use
  g_broadcaster.sendTransform(stamped_transform);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week_4");

  ros::NodeHandle nh;

  // Subscribe to oswin/imu and publish to oswin/odometry
  ros::Subscriber imu_sub = nh.subscribe("oswin/imu", 1, imuCallback);
  g_odom_pub = nh.advertise<nav_msgs::Odometry>("oswin/odometry", 1);

  // Set the last_time to now
  g_last_time = ros::Time::now();

  // Don't forget to spin to let ros handle calling the callbacks
  ros::spin();
}
