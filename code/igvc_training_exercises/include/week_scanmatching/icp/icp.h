#ifndef SRC_ICP_H
#define SRC_ICP_H

#include <week_scanmatching/scanmatcher.h>

namespace scanmatcher
{
using Result = ScanMatcher::Result;

class ICP : public ScanMatcher
{
public:
  [[nodiscard]] Result scanmatch(const pcl::PointCloud<pcl::PointXYZ>& input,
                                 const pcl::PointCloud<pcl::PointXYZ>& target,
                                 const Eigen::Isometry3d& prediction) override;

  [[nodiscard]] Result scanmatch(const pcl::PointCloud<pcl::PointXYZ>& input,
                                 const pcl::PointCloud<pcl::PointXYZ>& target) override
  {
    return scanmatch(input, target, Eigen::Isometry3d::Identity());
  }

private:
  [[nodiscard]] Eigen::Isometry3d getRigidTransform(const Eigen::VectorXd& weights, const Eigen::Matrix3Xd& input,
                                                    const Eigen::Matrix3Xd& target) const;
};
}  // namespace scanmatcher

#endif  // SRC_ICP_H
