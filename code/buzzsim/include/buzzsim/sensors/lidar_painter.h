#ifndef SRC_LIDAR_PAINTER_H
#define SRC_LIDAR_PAINTER_H

#include <QPainter>
#include <pcl_ros/point_cloud.h>
#include <buzzsim/sensors/lidar.h>

namespace turtle
{
class LidarPainter
{
 public:
  struct Options
  {
    int visualization_ratio;
  };

  LidarPainter(const Options& options, const Lidar::Options& lidar_options);
  void paint(QPainter& painter, const pcl::PointCloud<pcl::PointXYZ>& pointcloud, const motion::Pose& robot_pose, int width, int height) const;

 private:
  int discretize(double angle, double angular_resolution) const;
  double undiscretize(int discretized_angle, double angular_resolution) const;
  QLineF toQLineF(const motion::Position& start, const motion::Position& end, int width, int height) const;
  std::pair<double, double> toPolar(const pcl::PointXYZ& point) const;

  Options options_;
  Lidar::Options lidar_options_;
};
}

#endif //SRC_LIDAR_PAINTER_H
