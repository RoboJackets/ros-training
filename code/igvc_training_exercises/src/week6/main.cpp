#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

geometry_msgs::PoseStamped g_kyle_pose;
ros::Publisher g_map_pub;
ros::Publisher g_path_pub;
ros::Publisher g_velocity_pub;
nav_msgs::OccupancyGrid g_map;

constexpr int8_t OCCUPIED_THRESHOLD = 50;
constexpr double GOAL_THRESHOLD = 0.2;

void updateMap(const pcl::PointCloud<pcl::PointXYZ>& pointcloud)
{
  // 1: Convert to odom frame, since data is in "oswin" frame
  static tf::TransformListener transform_listener;
  ros::Time stamp = pcl_conversions::fromPCL(pointcloud.header.stamp);
  if (!transform_listener.waitForTransform("odom", "oswin", stamp, ros::Duration(5)))
  {
    ROS_ERROR_STREAM("Failed to find transform from odom to oswin.");
    return;
  }
  tf::StampedTransform transform_to_odom;
  transform_listener.lookupTransform("odom", "oswin", stamp, transform_to_odom);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl_ros::transformPointCloud(pointcloud, transformed_pointcloud, transform_to_odom);

  for (const auto& point : transformed_pointcloud)
  {
    int map_x = static_cast<int>(std::round((point.x - g_map.info.origin.position.x) / g_map.info.resolution));
    int map_y = static_cast<int>(std::round((point.y - g_map.info.origin.position.y) / g_map.info.resolution));
    int index = map_y * g_map.info.width + map_x;

    if (g_map.data[index] < 100)
    {
      g_map.data[index]++;
    }
  }
}

void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>& pointcloud)
{
  g_map.header.stamp = pcl_conversions::fromPCL(pointcloud.header.stamp);
  updateMap(pointcloud);
  g_map_pub.publish(g_map);
}

double getCost(const geometry_msgs::Pose& pose)
{
  int map_x = static_cast<int>(std::round((pose.position.x - g_map.info.origin.position.x) / g_map.info.resolution));
  int map_y = static_cast<int>(std::round((pose.position.y - g_map.info.origin.position.y) / g_map.info.resolution));
  int index = map_y * g_map.info.width + map_x;

  if (g_map.data[index] >= OCCUPIED_THRESHOLD)
  {
    return std::numeric_limits<double>::infinity();
  }

  auto dx = g_kyle_pose.pose.position.x - pose.position.x;
  auto dy = g_kyle_pose.pose.position.y - pose.position.y;
  double distance = std::hypot(dx, dy);
  if (distance <= GOAL_THRESHOLD)
  {
    return -std::numeric_limits<double>::infinity();
  }
  return distance;
}

void oswinPoseCallback(const geometry_msgs::PoseStamped& pose)
{
  auto dx = g_kyle_pose.pose.position.x - pose.pose.position.x;
  auto dy = g_kyle_pose.pose.position.y - pose.pose.position.y;
  double distance_to_goal = std::hypot(dx, dy);
  if (distance_to_goal <= GOAL_THRESHOLD)
  {
    geometry_msgs::Twist twist_message{};
    g_velocity_pub.publish(twist_message);
    return;
  }

  int num_primitives = 15;

  double angles_start = -1.0;
  double angles_end = 1.0;
  double angle_increment = (angles_end - angles_start) / num_primitives;

  double timestep = 0.01;
  int num_steps = 1000;

  std::vector<double> costs(num_primitives * num_primitives, 0.0);
  std::vector<nav_msgs::Path> paths(num_primitives * num_primitives);

  constexpr double linear_velocity = 0.5;

  std::vector<std::vector<double>> controls;

  for (int i = 0; i < num_primitives; i++)
  {
    for (int j = 0; j < num_primitives; j++)
    {
      std::vector<double> control{angles_start + i * angle_increment, angles_start + j * angle_increment};
      controls.emplace_back(control);
    }
  }

  // Get costs for all paths
  for (int i = 0; i < num_primitives * num_primitives; i++)
  {
    geometry_msgs::PoseStamped current_pose = pose;
    for (int t = 0; t < num_steps; t++)
    {
      double angular_velocity;
      if (t < num_steps / 2)
      {
        angular_velocity = controls[i][0];
      }
      else
      {
        angular_velocity = controls[i][1];
      }

      current_pose.pose.position.x += linear_velocity * cos(tf::getYaw(current_pose.pose.orientation)) * timestep;
      current_pose.pose.position.y += linear_velocity * sin(tf::getYaw(current_pose.pose.orientation)) * timestep;
      double new_yaw = tf::getYaw(current_pose.pose.orientation) + angular_velocity * timestep;
      current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(new_yaw);

      double cost = getCost(current_pose.pose);
      if (cost == -std::numeric_limits<double>::infinity())
      {
        break;
      }
      costs[i] += cost;
      paths[i].poses.emplace_back(current_pose);
    }
  }

  auto best_path = std::min_element(costs.begin(), costs.end());
  ROS_INFO_STREAM("Best cost: " << *best_path);
  int best_index = std::distance(costs.begin(), best_path);


  geometry_msgs::Twist twist_message;
  twist_message.linear.x = linear_velocity;
  twist_message.angular.z = controls[best_index][0];

  g_velocity_pub.publish(twist_message);

  nav_msgs::Path best_path_viz = paths[best_index];
  best_path_viz.header.frame_id = "odom";
  best_path_viz.header.stamp = ros::Time::now();
  g_path_pub.publish(best_path_viz);
}

void kylePoseCallback(const geometry_msgs::PoseStamped& pose)
{
  g_kyle_pose = pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week5");

  ros::NodeHandle nh;

  g_velocity_pub = nh.advertise<geometry_msgs::Twist>("oswin/velocity", 1);
  g_path_pub = nh.advertise<nav_msgs::Path>("oswin/path", 1);
  g_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("oswin/map", 1);
  ros::Subscriber pointcloud_sub = nh.subscribe("oswin/pointcloud", 1, &pointcloudCallback);
  ros::Subscriber oswin_pose_sub = nh.subscribe("oswin/ground_truth", 1, &oswinPoseCallback);
  ros::Subscriber kyle_pose_sub = nh.subscribe("kyle/ground_truth", 1, &kylePoseCallback);

  g_map.info.width = 200;
  g_map.info.height = 200;
  g_map.info.origin.position.x = -10;
  g_map.info.origin.position.y = -10;
  g_map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
  g_map.info.resolution = 0.1;
  g_map.header.frame_id = "odom";

  int total_cells = g_map.info.width * g_map.info.height;

  g_map.data = std::vector<int8_t>(total_cells, 0);

  ros::spin();
}
