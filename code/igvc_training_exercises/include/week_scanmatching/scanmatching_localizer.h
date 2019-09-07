#ifndef SRC_SCANMATCHING_LOCALIZER_H
#define SRC_SCANMATCHING_LOCALIZER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/LinearMath/Transform.h>
#include <week_scanmatching/icp/icp.h>

class ScanmatchingLocalizer
{
 public:
  ScanmatchingLocalizer();

 private:
  void lidarCallback(const pcl::PointCloud<pcl::PointXYZ>& scan);
  tf::Transform getTransformFromScan(const pcl::PointCloud<pcl::PointXYZ>& scan);
  void publishPose();

  ros::NodeHandle nh_;

  pcl::PointCloud<pcl::PointXYZ> previous_scan_ = {};
  ros::Subscriber lidar_sub_;
  ros::Publisher pose_pub_;

  ros::Publisher input_pub_;
  ros::Publisher target_pub_;
  ros::Publisher transformed_input_pub_;


  tf::Transform pose_;
  scanmatcher::ICP icp_;
};

#endif  // SRC_SCANMATCHING_LOCALIZER_H
