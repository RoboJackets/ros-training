#ifndef SRC_LANDMARK_REGISTRATION_H
#define SRC_LANDMARK_REGISTRATION_H

#include <vector>

#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <week_slam/barrel_ransac.h>
#include <tf/LinearMath/Transform.h>

struct Landmark
{
  int id;
  Barrel barrel;
};

class LandmarkRegistration
{
 public:
  struct Options
  {
    double new_landmark_threshold;
  };

  LandmarkRegistration(const Options& options, const BarrelRansac::Options& barrel_ransac_options);
  std::vector<Landmark> getLandmarks(const pcl::PointCloud<pcl::PointXYZ>& scan, const tf::Transform& transform_to_odom);
  std::vector<Landmark> landmarks_;
  void updateLandmark(int id, const Eigen::Vector2d& location);
 private:
  int findClosestLandmark(const Barrel& barrel) const;
  int registerLandmark(const Barrel& barrel);

  Options options_;
  BarrelRansac barrel_ransac_;
};

#endif //SRC_LANDMARK_REGISTRATION_H
