#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher g_velocity_pub; // Publisher for velocity
ros::Publisher g_error_pub; // Publisher for error

geometry_msgs::PoseStamped g_kyle_pose; // Pose of top turtle
ros::Time g_last_time; // Last time callback was called (to calculate delta t)

double g_kp; // Kp coefficient for Proportional controller

/**
 * Callback for Kyle (top turtle). Saves the position of the top turtle into the global
 * variable g_kyle_pose
 * @param msg message containing Kyle's pose
 */
void kylePoseCallback(geometry_msgs::PoseStamped msg)
{
  g_kyle_pose = msg;
}

/**
 * Callback for Oswin (bottom turtle). Calculates the error in x between the two turtles,
 * and then uses a Proportional Controller to calculate a control to publish
 * @param msg message containing Oswin's pose
 */
void oswinPoseCallback(geometry_msgs::PoseStamped msg)
{
  if (g_last_time.sec == 0)
  {
    g_last_time = ros::Time::now();
    return;
  }

  // Calculate error in x between top and bottom turtles
  double error = g_kyle_pose.pose.position.x - msg.pose.position.x;

  ros::Duration delta_t = msg.header.stamp - g_last_time;

  double proportional = error;
  // For a proportional controller:
  // u(t) = Kp * e(t)
  double control = g_kp * proportional;

  // Create geometry_msgs::Twist message and fill it in to publish the control
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = control;
  g_velocity_pub.publish(twist_msg);

  g_last_time = msg.header.stamp;

  // Create a std_msgs::Float64 message to be able to graph the error in
  // rqt_plot
  std_msgs::Float64 error_msg;
  error_msg.data = error;
  g_error_pub.publish(error_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week3");

  // For creating Publisher / Subscribers
  ros::NodeHandle nh;
  // For getting parameters that are between the <node> tags in the launch file
  ros::NodeHandle pnh{"~"};

  // Create parameter for kp
  pnh.getParam("kp", g_kp);

  // Advertise "/oswin/velocity" to control the bottom turtle and "error" for visualization
  g_velocity_pub = nh.advertise<geometry_msgs::Twist>("oswin/velocity", 1);
  g_error_pub = nh.advertise<std_msgs::Float64>("error", 1);

  // Subscriber to both ground truth topics to get their positions
  ros::Subscriber kyle_sub = nh.subscribe("kyle/ground_truth", 1, kylePoseCallback);
  ros::Subscriber oswin_sub = nh.subscribe("oswin/ground_truth", 1, oswinPoseCallback);

  // Don't forget to call ros::spin() to let ros do things behind the scenes and call your callback
  // functions when it receives a new message!
  ros::spin();
}
