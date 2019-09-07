#ifndef SRC_MAPPER_H
#define SRC_MAPPER_H

#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>

class Mapper
{
 public:
  void setInfo(int width, int height, float resolution);
  void addScan(const pcl::PointCloud<pcl::PointXYZ>& scan);
  int toIndex(double x, double y) const;

  nav_msgs::OccupancyGrid occupancy_grid_{};
 private:
  bool inMap(const pcl::PointXYZ& point) const;
};

#endif //SRC_MAPPER_H
