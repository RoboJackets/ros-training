#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

ros::Publisher g_map_pub;
nav_msgs::OccupancyGrid g_map;

void updateMap(const pcl::PointCloud<pcl::PointXYZ>& pointcloud)
{
  // Create a static tf::TransformListener so that we only create it once, and it's after ros::NodeHandle
  static tf::TransformListener transform_listener;

  // Convert the timestamp from PCL to ros::Time, since PCL uses uint64
  ros::Time stamp = pcl_conversions::fromPCL(pointcloud.header.stamp);

  // Lookup the transform to convert the pointcloud from oswin to odom, at the time of the message
  // if it is valid, otherwise get the latest transform
  if (!transform_listener.waitForTransform("odom", "oswin", stamp, ros::Duration(5)))
  {
    ROS_ERROR_STREAM("Failed to find transform from odom to oswin.");
    return;
  }
  tf::StampedTransform transform_to_odom;
  transform_listener.lookupTransform("odom", "oswin", stamp, transform_to_odom);

  // Transform the pointcloud from the lidar frame to the odom frame using the transform form just now
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl_ros::transformPointCloud(pointcloud, transformed_pointcloud, transform_to_odom);

  // Find the coordinate in the grid from each point in the pointcloud
  for (const auto& point : transformed_pointcloud)
  {
    int map_x = static_cast<int>(std::round((point.x - g_map.info.origin.position.x) / g_map.info.resolution));
    int map_y = static_cast<int>(std::round((point.y - g_map.info.origin.position.y) / g_map.info.resolution));

    // Convert the indices from 2D index to 1D index
    int index = map_y * g_map.info.width + map_x;

    // If it's less than 100, increment it
    if (g_map.data[index] < 100)
    {
      g_map.data[index]++;
    }
  }
}

/**
 * The callback for the pointcloud subscriber
 * @param pointcloud
 */
void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZ>& pointcloud)
{
  g_map.header.stamp = pcl_conversions::fromPCL(pointcloud.header.stamp);
  // We break out the "updateMap" functionality into another method for clarity
  updateMap(pointcloud);
  // Publish the updated map
  g_map_pub.publish(g_map);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week5");

  ros::NodeHandle nh;

  // Advertise to oswin/map and subscriber to oswin/pointcloud
  g_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("oswin/map", 1);
  ros::Subscriber pointcloud_sub = nh.subscribe("oswin/pointcloud", 1, &pointcloudCallback);

  // Set information about the map, ie. width, height etc
  g_map.info.width = 200;
  g_map.info.height = 200;
  g_map.info.origin.position.x = -10;
  g_map.info.origin.position.y = -10;
  g_map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
  g_map.info.resolution = 0.1;
  g_map.header.frame_id = "odom"; // Make sure to set the frame_id

  int total_cells = g_map.info.width * g_map.info.height;

  // Instantiate the g_map.data to hvae total_cells number of cells
  g_map.data = std::vector<int8_t>(total_cells, 0);

  // Don't forget to ros::spin() to let ros call callbacks for you
  ros::spin();
}
