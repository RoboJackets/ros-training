#ifndef SRC_SLAM_NODE_H
#define SRC_SLAM_NODE_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <unordered_map>

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <week_slam/g2o/vertex/vertex_robot_state.h>
#include <week_slam/landmark_registration.h>
#include <week_slam/mapper.h>
#include <week_slam/slam_node.h>
#include <week_slam/slam_node_params.h>

class SlamNode
{
public:
  SlamNode();

private:
  using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
  using SlamLinearSolver = g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType>;

  void setupG2o();
  void setupGraph();

  void poseCallback(const geometry_msgs::PoseStamped& msg);
  void imuCallback(const sensor_msgs::Imu& msg);
  void lidarCallback(const pcl::PointCloud<pcl::PointXYZ>& msg);
  void publishLandmarks(const std::vector<Landmark>& landmarks);

  g2o::VertexRobotState* addVertex(const g2o::RobotState& state);
  void addLandmarkEdges(const std::vector<Landmark>& landmarks);
  void addLandmarkEdge(int vertex_id, const Landmark& landmark);
  int addLandmarkVertex(const Landmark& landmark);
  void updateLandmarkLocations();

  void addIMUEdge(const sensor_msgs::Imu& msg);

  void optimizeGraph();
  void publishLatestPose();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber lidar_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher map_pub_;
  ros::Publisher landmark_pub_;

  SlamNodeParams params_;
  LandmarkRegistration landmark_registration_;
  Mapper mapper_{};
  geometry_msgs::PoseStamped pose_stamped_{};

  std::unordered_map<int, int> landmark_map_{};
  g2o::SparseOptimizer optimizer_;
  ros::Time starting_time_{};

  int latest_pose_vertex_id_ = 0;
  int latest_vertex_id_ = 0;
};

#endif  // SRC_SLAM_NODE_H
