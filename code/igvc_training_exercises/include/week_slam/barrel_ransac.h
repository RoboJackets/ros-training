#ifndef SRC_BARREL_RANSAC_H
#define SRC_BARREL_RANSAC_H

#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
#include <random>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
using Barrel = Eigen::Vector3d;

class BarrelRansac
{
public:
  struct Options
  {
    int iterations;
    double threshold;
    int min_scan_points;
    int max_barrels;
    double max_error;
    double min_barrel_radius;
    double max_barrel_radius;
  };

  struct RANSACResult
  {
    Barrel barrel;
    std::vector<Eigen::Vector2d> outliers;
    double error;
  };

  void test();
  BarrelRansac(const Options& options);
  std::vector<Barrel> execute(const pcl::PointCloud<pcl::PointXYZ>& scan);
  std::vector<Barrel> executeSequentially(std::vector<Eigen::Vector2d> scan);
  RANSACResult executeOnce(const std::vector<Eigen::Vector2d>& scan);
  [[nodiscard]] double residual(const Barrel& barrel, const Eigen::Vector2d& point) const;
  [[nodiscard]] double residual(const Barrel& barrel, const std::vector<Eigen::Vector2d>& points) const;

private:
  using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>>;
  using LinearSolver = g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType>;

  [[nodiscard]] Barrel fitToBarrel(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3) const;
  [[nodiscard]] Barrel fitToBarrel(const Barrel& estimate, const std::vector<Eigen::Vector2d>& inliers);
  [[nodiscard]] bool inForwardHalf(const Barrel& barrel, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3) const;

  /**
   * Normalizes angle between -pi and pi
   * @param angle
   * @return
   */
  [[nodiscard]] static double normalizeAngle(double angle);

  Options options_;
  std::mt19937 mt_;
  g2o::SparseOptimizer optimizer_ = {};
};

#endif  // SRC_BARREL_RANSAC_H
