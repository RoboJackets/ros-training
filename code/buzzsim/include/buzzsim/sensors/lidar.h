#ifndef SRC_LIDAR_H
#define SRC_LIDAR_H

#include <vector>

#include <pcl_ros/point_cloud.h>

#include <buzzsim/motion.h>
#include <buzzsim/obstacle.h>
#include <buzzsim/sensors/cgal_types.h>

namespace turtle
{
class Lidar
{
public:
  struct Options
  {
    double angle_width;
    double range;
    double angular_resolution;
  };

  Lidar(const Options& options);

  [[nodiscard]] pcl::PointCloud<pcl::PointXYZ> simulate(const motion::Pose& pose,
                                                        const std::vector<Obstacle>& obstacles) const;

private:
  /**
   * Computes the visibility polygon from the passed in position, with the passed in polygon with holes
   * @param position
   * @param polygon
   * @return
   */
  [[nodiscard]] Polygon_2 computeVisibilityPolygon(const motion::Position& position,
                                                   const Polygon_with_holes_2& polygon) const;

  /**
   * Returns a polygon with holes representing a bounded environment of the robot, with holes representing obstacles
   * @param position
   * @param obstacles
   * @return
   */
  [[nodiscard]] Polygon_with_holes_2 getPolygonWithHoles(const motion::Position& position,
                                                         const std::vector<Obstacle>& obstacles) const;

  /**
   * Calculates the lidar hits from the passed in pose and visibility polygon
   * @param pose
   * @param visibility_polygon
   * @return
   */
  [[nodiscard]] pcl::PointCloud<pcl::PointXYZ> calculateLidarHits(const motion::Pose& pose,
                                                                  const Polygon_2& visibility_polygon) const;

  /**
   * Returns the closest intersection between the scan_segment and visibility_polygon
   * @param scan_segment
   * @param visibility_polygon
   * @return The closest intersecting point
   */
  [[nodiscard]] Point_2 getClosestIntersection(const Segment_2& scan_segment,
                                               const Polygon_2& visibility_polygon) const;

  /**
   * Sets closest_point and closest_distance to the new_point and new squared_distance respectively if the new_point is
   * closer to the comparison_point
   *
   * @param closest_point
   * @param closest_distance
   * @param new_point
   * @param comparison_point
   */
  void setClosestIfCloser(Point_2* closest_point, double* closest_distance, const Point_2& new_point,
                          const Point_2& comparison_point) const;

  Options options_;
};
}  // namespace turtle

#endif  // SRC_LIDAR_H
