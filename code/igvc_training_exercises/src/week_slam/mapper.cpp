#include <week_slam/mapper.h>

void Mapper::setInfo(int width, int height, float resolution)
{
  occupancy_grid_.info.resolution = resolution;
  occupancy_grid_.info.width = width;
  occupancy_grid_.info.height = height;

  occupancy_grid_.info.origin.position.x = -(width * resolution) / 2.0;
  occupancy_grid_.info.origin.position.y = -(height * resolution) / 2.0;
  occupancy_grid_.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
  occupancy_grid_.header.frame_id = "odom";
  occupancy_grid_.data = std::vector<int8_t>(width * height, 0);
}

void Mapper::addScan(const pcl::PointCloud<pcl::PointXYZ>& scan)
{
  ros::Time stamp = pcl_conversions::fromPCL(scan.header.stamp);
  occupancy_grid_.header.stamp = stamp;

  for (const auto& point : scan)
  {
    if (inMap(point))
    {
      int index = toIndex(point.x, point.y);
      if (occupancy_grid_.data[index] < 100)
      {
        occupancy_grid_.data[index]++;
      }
    }
  }
}

int Mapper::toIndex(double x, double y) const
{
  int map_x = static_cast<int>(std::round((x - occupancy_grid_.info.origin.position.x) / occupancy_grid_.info.resolution));
  int map_y = static_cast<int>(std::round((y - occupancy_grid_.info.origin.position.y) / occupancy_grid_.info.resolution));
  int index = map_y * static_cast<int>(occupancy_grid_.info.width) + map_x;
  return index;
}

bool Mapper::inMap(const pcl::PointXYZ& point) const
{
  double width = occupancy_grid_.info.width * occupancy_grid_.info.resolution;
  double height = occupancy_grid_.info.height * occupancy_grid_.info.resolution;
  bool x_good = -width / 2 < point.x && point.x < width / 2;
  bool y_good = -height / 2 < point.y && point.y < height / 2;
  return x_good && y_good;
}
