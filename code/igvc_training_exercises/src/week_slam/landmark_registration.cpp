#include <week_slam/landmark_registration.h>

LandmarkRegistration::LandmarkRegistration(const Options& options, const BarrelRansac::Options& barrel_ransac_options)
  : options_{ options }, barrel_ransac_{ barrel_ransac_options }
{
}

std::vector<Landmark> LandmarkRegistration::getLandmarks(const pcl::PointCloud<pcl::PointXYZ>& scan, const tf::Transform& transform_to_odom)
{
  auto result = barrel_ransac_.execute(scan);

  std::vector<Landmark> landmarks;
  landmarks.reserve(result.size());
  for (const auto& barrel : result)
  {
    auto transformed_barrel_tf = transform_to_odom * tf::Vector3{barrel(0), barrel(1), 0.0};
    auto transformed_barrel = Eigen::Vector3d{transformed_barrel_tf.x(), transformed_barrel_tf.y(), barrel(2)};

    int closest_index = findClosestLandmark(transformed_barrel);
    if (closest_index == -1)
    {
      closest_index = registerLandmark(transformed_barrel);
    }
    Landmark closest_landmark{closest_index, barrel};
    landmarks.emplace_back(closest_landmark);
  }

  return landmarks;
}

int LandmarkRegistration::findClosestLandmark(const Barrel& barrel) const
{
  double min_distance = std::numeric_limits<double>::max();
  int min_index = 0;
  for (int i = 0; i < static_cast<int>(landmarks_.size()); i++)
  {
    double distance = (landmarks_[i].barrel.head<2>() - barrel.head<2>()).norm();
    if (distance < min_distance)
    {
      min_distance = distance;
      min_index = i;
    }
  }

  if (min_distance < options_.new_landmark_threshold)
  {
    return min_index;
  }

  return -1;
}

int LandmarkRegistration::registerLandmark(const Barrel& barrel)
{
  int id = landmarks_.size();
  Landmark new_landmark{ id, barrel };
  landmarks_.emplace_back(new_landmark);
  return id;
}

void LandmarkRegistration::updateLandmark(int id, const Eigen::Vector2d& location)
{
  landmarks_.at(id).barrel.head<2>() = location;
}
