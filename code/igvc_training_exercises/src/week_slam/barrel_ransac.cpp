#include <Eigen/StdVector>

#include <week_slam/g2o/edge/edge_point_on_circle.h>
#include <week_slam/barrel_ransac.h>

void BarrelRansac::test()
{
  double radius = 0.5;
  int num_points = 10;

  std::vector<Eigen::Vector2d> points;
  points.reserve(3 * num_points);

  std::uniform_real_distribution<> distribution(0, 2 * M_PI);
  std::normal_distribution<> rad_dist(radius, 0.05);

  {
    Eigen::Vector2d center{ 1.0, 2.0 };
    for (int i = 0; i < num_points; i++)
    {
      double r = rad_dist(mt_);
      double angle = distribution(mt_);

      double x = center.x() + r * cos(angle);
      double y = center.y() + r * sin(angle);

      points.emplace_back(x, y);
    }
  }

  {
    Eigen::Vector2d center{ 5.0, 4.0 };
    for (int i = 0; i < num_points; i++)
    {
      double r = rad_dist(mt_);
      double angle = distribution(mt_);

      double x = center.x() + r * cos(angle);
      double y = center.y() + r * sin(angle);

      points.emplace_back(x, y);
    }
  }

  {
    Eigen::Vector2d center{ 6.2, 4.0 };
    for (int i = 0; i < num_points; i++)
    {
      double r = rad_dist(mt_);
      double angle = distribution(mt_);

      double x = center.x() + r * cos(angle);
      double y = center.y() + r * sin(angle);

      points.emplace_back(x, y);
    }
  }
  //  auto res = executeOnce(points);
  //  ROS_INFO_STREAM("Barrel: " << res.barrel);
  //  for (const auto& outlier : res.outliers)
  //  {
  //    ROS_INFO_STREAM("Outliers: " << outlier);
  //  }

  pcl::PointCloud<pcl::PointXYZ> scan;
  scan.points.reserve(num_points);
  for (const auto& vector : points)
  {
    pcl::PointXYZ p{ static_cast<float>(vector(0)), static_cast<float>(vector(1)), 0.0 };
    scan.points.emplace_back(p);
  }

  auto before = ros::Time::now();
  auto result = execute(scan);
  auto after = ros::Time::now();
  ROS_INFO_STREAM("Time taken: " << (after - before).toSec() << " seconds.");

  for (const auto& barrel : result)
  {
    ROS_INFO_STREAM(barrel);
  }
}

BarrelRansac::BarrelRansac(const Options& options) : options_{ options }, mt_(std::random_device()())
{
  auto block_solver = std::make_unique<BlockSolver>(std::make_unique<LinearSolver>());
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
  optimizer_.setAlgorithm(solver);
}

std::vector<Barrel> BarrelRansac::execute(const pcl::PointCloud<pcl::PointXYZ>& scan)
{
  std::vector<Eigen::Vector2d> eigen_scan;
  std::transform(scan.begin(), scan.end(), std::back_inserter(eigen_scan), [](const pcl::PointXYZ& point) {
    return Eigen::Vector2d{ point.x, point.y };
  });

  return executeSequentially(std::move(eigen_scan));
}

std::vector<Barrel> BarrelRansac::executeSequentially(std::vector<Eigen::Vector2d> scan)
{
  std::vector<Barrel> barrels;
  barrels.reserve(options_.max_barrels);

  while (static_cast<int>(barrels.size()) < options_.max_barrels &&
         static_cast<int>(scan.size()) > options_.min_scan_points)
  {
    //    ROS_INFO_STREAM("Starting loop. scan.size() == " << scan.size());
    auto result = executeOnce(scan);

//    ROS_INFO_STREAM("result.error: " << result.error);
    if (result.error > options_.max_error)
    {
      ROS_INFO_STREAM("Breaking due to error");
      break;
    }

    barrels.emplace_back(result.barrel);
    scan = std::move(result.outliers);
  }
  //  ROS_INFO_STREAM("barrels.size(): " << barrels.size() << ", scan.size(): " << scan.size());
  return barrels;
}

BarrelRansac::RANSACResult BarrelRansac::executeOnce(const std::vector<Eigen::Vector2d>& scan)
{
  //  ROS_INFO_STREAM("Starting <fn executeOnce>");
  std::vector<Eigen::Vector2d> copy = scan;
  int num_points = scan.size();

  Barrel best_barrel;
  std::vector<Eigen::Vector2d> best_outliers;
  double best_err = std::numeric_limits<double>::max();

  for (int i = 0; i < options_.iterations; i++)
  {
    std::vector<Eigen::Vector2d> inliers;
    inliers.reserve(scan.size());
    std::vector<Eigen::Vector2d> outliers;
    outliers.reserve(scan.size());

    std::shuffle(copy.begin(), copy.end(), mt_);
    inliers.emplace_back(copy[0]);
    inliers.emplace_back(copy[1]);
    inliers.emplace_back(copy[2]);

    Barrel barrel = fitToBarrel(copy[0], copy[1], copy[2]);

    // Found barrel must have radius between min and max radius
    if (barrel(2) < options_.min_barrel_radius || barrel(2) > options_.max_barrel_radius)
    {
      continue;
    }

    // Points must be in forward facing half of the circle
    if (!inForwardHalf(barrel, copy[0], copy[1], copy[2]))
    {
      continue;
    }

    for (int j = 3; j < static_cast<int>(copy.size()); j++)
    {
      if (residual(barrel, copy[j]) < options_.threshold)
      {
        inliers.emplace_back(copy[j]);
      }
      else
      {
        outliers.emplace_back(copy[j]);
      }
    }

    //    barrel = fitToBarrel(barrel, inliers);
    double error = residual(barrel, inliers);
    if (error < best_err)
    {
      best_barrel = barrel;
      best_outliers = std::move(outliers);
      best_err = error;
    }
  }

  //  ROS_INFO_STREAM("done with <fn executeOnce>");
//  ROS_INFO_STREAM("Scan size: " << scan.size() << ", inliers: " << scan.size() - best_outliers.size());
  return { best_barrel, std::move(best_outliers), best_err };
}

Barrel BarrelRansac::fitToBarrel(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3) const
{
  auto x1 = p1(0);
  auto y1 = p1(1);
  auto x2 = p2(0);
  auto y2 = p2(1);
  auto x3 = p3(0);
  auto y3 = p3(1);

  auto a = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;

  auto b = (x1 * x1 + y1 * y1) * (y3 - y2) + (x2 * x2 + y2 * y2) * (y1 - y3) + (x3 * x3 + y3 * y3) * (y2 - y1);

  auto c = (x1 * x1 + y1 * y1) * (x2 - x3) + (x2 * x2 + y2 * y2) * (x3 - x1) + (x3 * x3 + y3 * y3) * (x1 - x2);

  auto x = -b / (2 * a);
  auto y = -c / (2 * a);
  auto r = std::hypot(x - x1, y - y1);

  return { x, y, r };
}

Barrel BarrelRansac::fitToBarrel(const Barrel& estimate, const std::vector<Eigen::Vector2d>& inliers)
{
  auto* circle = new VertexCircle();
  circle->setId(0);
  circle->setEstimate(estimate);
  optimizer_.addVertex(circle);

  for (const auto& inlier : inliers)
  {
    auto* edge = new EdgePointOnCircle;
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    edge->setVertex(0, circle);
    edge->setMeasurement(inlier);
    optimizer_.addEdge(edge);
  }

  optimizer_.initializeOptimization();
  optimizer_.optimize(100);

  Barrel output = circle->estimate();
  optimizer_.clear();

  return output;
}

double BarrelRansac::residual(const Barrel& barrel, const Eigen::Vector2d& point) const
{
  double d = (barrel.head<2>() - point).norm() - barrel(2);
  return d * d;
}

double BarrelRansac::residual(const Barrel& barrel, const std::vector<Eigen::Vector2d>& points) const
{
  double acc = 0.0;
  for (const auto& point : points)
  {
    acc += residual(barrel, point);
  }
  return acc / points.size();
}

bool BarrelRansac::inForwardHalf(const Barrel& barrel, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                 const Eigen::Vector2d& p3) const
{
  double barrel_theta = atan2(barrel(1), barrel(0));

  for (const auto& p : std::vector<Eigen::Vector2d>{ p1, p2, p3 })
  {
    double theta = atan2(p(1) - barrel(1), p(0) - barrel(0));
    double rotated_theta = BarrelRansac::normalizeAngle(theta - barrel_theta - (M_PI / 2));

    if (rotated_theta < 0)
    {
      return false;
    }
  }

  return true;
}

double BarrelRansac::normalizeAngle(double angle)
{
  return -M_PI + fmod(2 * M_PI + fmod(angle + M_PI, 2 * M_PI), 2 * M_PI);
}
