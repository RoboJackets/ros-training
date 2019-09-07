#include <ros/ros.h>

#include <week_scanmatching/icp/icp.h>
#include <week_scanmatching/scanmatching_localizer.h>

void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Affine3d& transformation)
{
  for (auto& point : cloud)
  {
    point.getVector3fMap() = transformation.cast<float>() * point.getVector3fMap();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "week_scanmatching");

  ScanmatchingLocalizer localizer;

  ros::spin();
  std::exit(0);
}
