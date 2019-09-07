#include <buzzsim/sensors/lidar_painter.h>
#include <QLineF>
#include <QVector>

namespace turtle
{
LidarPainter::LidarPainter(const Options& options, const Lidar::Options& lidar_options)
  : options_{ options }, lidar_options_{ lidar_options }
{
}

void LidarPainter::paint(QPainter& painter, const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                         const motion::Pose& robot_pose, int width, int height) const
{
  QPen pen;
  pen.setColor({234, 97, 228});
  painter.setPen(pen);
  std::unordered_map<int, double> angles;

  const int start_angle = discretize(-lidar_options_.angle_width / 2.0, lidar_options_.angular_resolution);
  const int end_angle = discretize(lidar_options_.angle_width / 2.0, lidar_options_.angular_resolution);

  for (const auto& point : pointcloud)
  {
    auto [angle, distance] = toPolar(point);
    int discretized_angle = discretize(angle, lidar_options_.angular_resolution);
    angles.insert(std::make_pair(discretized_angle, distance));
  }

  QVector<QLineF> lines;
  lines.reserve((end_angle - start_angle) / options_.visualization_ratio + 1);

  for (int angle = start_angle; angle <= end_angle; angle += options_.visualization_ratio)
  {
    double range;
    // Out of range, show a line that goes max range.
    if (angles.find(angle) == angles.end())
    {
      range = lidar_options_.range;
    }
    else
    {
      range = angles.find(angle)->second;
    }

    double world_angle = undiscretize(angle, lidar_options_.angular_resolution) + robot_pose.orientation;
    double x = robot_pose.position.x +
               range * cos(world_angle);
    double y = robot_pose.position.y +
               range * sin(world_angle);
    motion::Position endpoint{ x, y };
    lines.push_back(toQLineF(robot_pose.position, endpoint, width, height));
  }

  painter.drawLines(lines);
}

std::pair<double, double> LidarPainter::toPolar(const pcl::PointXYZ& point) const
{
  double angle = std::atan2(point.y, point.x);
  double distance = std::hypot(point.x, point.y);

  return std::make_pair(angle, distance);
}

int LidarPainter::discretize(double angle, double angular_resolution) const
{
  double coeff = 1 / angular_resolution;
  return static_cast<int>(std::round(coeff * angle));
}

double LidarPainter::undiscretize(int discretized_angle, double angular_resolution) const
{
  return discretized_angle * angular_resolution;
}

QLineF LidarPainter::toQLineF(const motion::Position& start, const motion::Position& end, int width, int height) const
{
  // x is front, y is left.
  QPointF qpoint_end = end.toQPointF(width, height);
  QPointF qpoint_start = start.toQPointF(width, height);
  return { qpoint_start, qpoint_end };
}
}  // namespace turtle
