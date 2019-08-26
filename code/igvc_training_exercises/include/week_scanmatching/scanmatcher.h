#ifndef SRC_SCANMATCHER_H
#define SRC_SCANMATCHER_H

#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>

namespace scanmatcher
{
class ScanMatcher
{
public:
  struct Result
  {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    double error = 0.0;
  };

  [[nodiscard]] virtual Result scanmatch(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& target,
                           const Eigen::Isometry3d& prediction) = 0;

  [[nodiscard]] virtual Result scanmatch(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& target)
  {
    return scanmatch(input, target, Eigen::Isometry3d::Identity());
  }
};
}  // namespace scanmatcher
#endif  // SRC_SCANMATCHER_H
