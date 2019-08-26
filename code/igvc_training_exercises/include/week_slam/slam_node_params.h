#ifndef SRC_SLAM_NODE_PARAMS_H
#define SRC_SLAM_NODE_PARAMS_H

#include <ros/ros.h>
#include <week_slam/landmark_registration.h>

struct SlamNodeParams
{
  SlamNodeParams() = default;
  SlamNodeParams(ros::NodeHandle& nh);
  LandmarkRegistration::Options landmark_registration_options_;
  BarrelRansac::Options barrel_ransac_options_;
};

#endif //SRC_SLAM_NODE_PARAMS_H
