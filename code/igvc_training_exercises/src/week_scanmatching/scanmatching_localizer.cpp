#include <tf_conversions/tf_eigen.h>
#include <week_scanmatching/scanmatching_localizer.h>

#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

ScanmatchingLocalizer::ScanmatchingLocalizer() : pose_{ tf::Transform::getIdentity() }
{
  lidar_sub_ = nh_.subscribe("oswin/pointcloud", 1, &ScanmatchingLocalizer::lidarCallback, this);

  pose_pub_ = nh_.advertise<nav_msgs::Odometry>("oswin/scanmatched_pose", 1);
  input_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("oswin/input_pc", 1);
  target_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("oswin/target_pc", 1);
  transformed_input_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("oswin/transformed_input_pc", 1);
}

void ScanmatchingLocalizer::lidarCallback(const pcl::PointCloud<pcl::PointXYZ>& scan)
{
  if (!previous_scan_.empty())
  {
    auto transform = getTransformFromScan(scan);
    tf::Transform pose_rotation = tf::Transform::getIdentity();
    pose_rotation.setRotation(pose_.getRotation());

    pose_.setOrigin(pose_.getOrigin() + pose_rotation * transform.getOrigin());
    pose_.setRotation(transform.getRotation() * pose_.getRotation());
    publishPose();

    input_pub_.publish(scan);
    previous_scan_.header.frame_id = "oswin";
    previous_scan_.header.stamp = scan.header.stamp;
    target_pub_.publish(previous_scan_);
    pcl::PointCloud<pcl::PointXYZ> transformed_input;
    pcl_ros::transformPointCloud(scan, transformed_input, transform);
    transformed_input.header.frame_id = "oswin";
    transformed_input_pub_.publish(transformed_input);
  }
  previous_scan_ = scan;
}

tf::Transform ScanmatchingLocalizer::getTransformFromScan(const pcl::PointCloud<pcl::PointXYZ>& scan)
{
  auto [transform, error] = icp_.scanmatch(scan, previous_scan_);
  tf::Transform tf_transform;
  tf::poseEigenToTF(transform, tf_transform);

  return tf_transform;
}

void ScanmatchingLocalizer::publishPose()
{
  nav_msgs::Odometry msg{};
  msg.header.frame_id = "odom";
  msg.header.stamp = ros::Time::now();

  tf::poseTFToMsg(pose_, msg.pose.pose);
  pose_pub_.publish(msg);
}
