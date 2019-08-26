#include <week_slam/slam_node_params.h>

SlamNodeParams::SlamNodeParams(ros::NodeHandle& nh)
{
  nh.getParam("landmark_registration/new_landmark_threshold", landmark_registration_options_.new_landmark_threshold);

  nh.getParam("barrel_ransac/threshold", barrel_ransac_options_.threshold);
  nh.getParam("barrel_ransac/iterations", barrel_ransac_options_.iterations);
  nh.getParam("barrel_ransac/max_barrels", barrel_ransac_options_.max_barrels);
  nh.getParam("barrel_ransac/max_error", barrel_ransac_options_.max_error);
  nh.getParam("barrel_ransac/max_barrel_radius", barrel_ransac_options_.max_barrel_radius);
  nh.getParam("barrel_ransac/min_barrel_radius", barrel_ransac_options_.min_barrel_radius);
  nh.getParam("barrel_ransac/min_scan_points", barrel_ransac_options_.min_scan_points);
}


