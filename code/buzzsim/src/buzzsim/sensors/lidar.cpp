#include <buzzsim/sensors/cgal_types.h>
#include <buzzsim/sensors/lidar.h>
#include <buzzsim/sensors/print_utils.h>

namespace turtle
{
Lidar::Lidar(const Options& options) : options_{ options }
{
}

pcl::PointCloud<pcl::PointXYZ> Lidar::simulate(const motion::Pose& pose, const std::vector<Obstacle>& obstacles) const
{
  auto polygon = getPolygonWithHoles(pose.position, obstacles);
  auto visibility_polygon = computeVisibilityPolygon(pose.position, polygon);
  return calculateLidarHits(pose, visibility_polygon);
}

pcl::PointCloud<pcl::PointXYZ> Lidar::calculateLidarHits(const motion::Pose& pose,
                                                         const Polygon_2& visibility_polygon) const
{
  double start_angle = pose.orientation - options_.angle_width / 2;
  double end_angle = pose.orientation + options_.angle_width / 2;

  pcl::PointCloud<pcl::PointXYZ> scan;
  scan.points.reserve(std::round((end_angle - start_angle) / options_.angular_resolution) + 2);
  Point_2 pose_point{ pose.position.x, pose.position.y };

  for (double angle = start_angle; angle <= end_angle; angle += options_.angular_resolution)
  {
    double endpoint_x = pose.position.x + options_.range * cos(angle);
    double endpoint_y = pose.position.y + options_.range * sin(angle);

    Point_2 scan_endpoint{ endpoint_x, endpoint_y };
    Segment_2 scan_segment{ pose_point, scan_endpoint };

    Point_2 closest_intersection = getClosestIntersection(scan_segment, visibility_polygon);
    double squared_distance = CGAL::to_double(CGAL::squared_distance(pose_point, closest_intersection));

    //    ROS_INFO_STREAM("Angle: " << angle << ", distance: " << std::sqrt(squared_distance));
    if (squared_distance != 0 && squared_distance <= options_.range * options_.range)
    {
      // Transform back to local frame
      double x = CGAL::to_double(closest_intersection.x() - pose_point.x());
      double y = CGAL::to_double(closest_intersection.y() - pose_point.y());
      double rotated_x = cos(pose.orientation) * x + sin(pose.orientation) * y;
      double rotated_y = -sin(pose.orientation) * x + cos(pose.orientation) * y;
      pcl::PointXYZ pcl_point{ static_cast<float>(rotated_x), static_cast<float>(rotated_y), 0.0 };
      scan.points.emplace_back(pcl_point);
    }
  }

  return scan;
}

Polygon_2 Lidar::computeVisibilityPolygon(const motion::Position& position, const Polygon_with_holes_2& polygon) const
{
  Arrangement_2 env;
  std::vector<Segment_2> segments;

  for (auto it = polygon.outer_boundary().edges_begin(); it != polygon.outer_boundary().edges_end(); it++)
  {
    segments.emplace_back(*it);
  }

  for (auto it = polygon.holes_begin(); it != polygon.holes_end(); it++)
  {
    for (auto edge_it = it->edges_begin(); edge_it != it->edges_end(); edge_it++)
    {
      segments.emplace_back(*edge_it);
    }
  }

  CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

  // Find the face of the query point
  Point_2 q{ position.x, position.y };
  Arrangement_2::Face_const_handle* face;
  CGAL::Arr_naive_point_location<Arrangement_2> pl(env);
  CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(q);

  // The query point locates in the interior of the a face
  face = boost::get<Arrangement_2::Face_const_handle>(&obj);

  // Compute regularized visibility area
  Arrangement_2 regular_output;
  TEV regular_visibility(env);

  Face_handle face_handle = regular_visibility.compute_visibility(q, *face, regular_output);

  Polygon_2 visibility_polygon;
  auto circulator = face_handle->outer_ccb();
  visibility_polygon.push_back(circulator->source()->point());
  while (++circulator != face_handle->outer_ccb())
  {
    visibility_polygon.push_back(circulator->source()->point());
  }

  return visibility_polygon;
}

Polygon_with_holes_2 Lidar::getPolygonWithHoles(const motion::Position& position,
                                                const std::vector<Obstacle>& obstacles) const
{
  auto range = options_.range * 2;  // Should be sqrt(2) but I don't want to deal with rounding errors and stuff...

  Point_2 top{ position.x + range, position.y };
  Point_2 bottom{ position.x - range, position.y };
  Point_2 left{ position.x, position.y + range };
  Point_2 right{ position.x, position.y - range };

  Polygon_2 lidar_range_rect;
  lidar_range_rect.push_back(top);
  lidar_range_rect.push_back(left);
  lidar_range_rect.push_back(bottom);
  lidar_range_rect.push_back(right);

  Polygon_set_2 result;
  for (const auto& obstacle : obstacles)
  {
    Polygon_2 polygon_obstacle;
    for (const auto& point : obstacle.points)
    {
      Point_2 p{ point.x, point.y };
      polygon_obstacle.push_back(p);
    }
    // Flip it if it's the wrong orientation
    if (polygon_obstacle.is_clockwise_oriented())
    {
      polygon_obstacle.reverse_orientation();
    }

    result.insert(polygon_obstacle);
  }

  result.complement();
  result.intersection(lidar_range_rect);

  std::list<Polygon_with_holes_2> result_list;
  result.polygons_with_holes(std::back_inserter(result_list));

  return result_list.front();
}

Point_2 Lidar::getClosestIntersection(const Segment_2& scan_segment, const Polygon_2& visibility_polygon) const
{
  // value for no matches = pos
  auto pos = scan_segment.source();
  Point_2 closest_point = pos;
  double closest_distance = 0.0;

  for (auto edge_it = visibility_polygon.edges_begin(); edge_it != visibility_polygon.edges_end(); edge_it++)
  {
    auto result = CGAL::intersection(scan_segment, *edge_it);

    if (result)
    {
      if (auto seg = boost::get<Segment_2>(&*result))
      {
        auto source = seg->source();
        auto target = seg->target();

        setClosestIfCloser(&closest_point, &closest_distance, source, pos);
        setClosestIfCloser(&closest_point, &closest_distance, target, pos);
      }
      else if (auto point = boost::get<Point_2>(&*result))
      {
        setClosestIfCloser(&closest_point, &closest_distance, *point, pos);
      }
    }
  }
  return closest_point;
}

void Lidar::setClosestIfCloser(Point_2* closest_point, double* closest_distance, const Point_2& new_point,
                               const Point_2& comparison_point) const
{
  double new_distance = CGAL::to_double(CGAL::squared_distance(comparison_point, new_point));
  if (*closest_distance == 0.0 || new_distance < *closest_distance)
  {
    *closest_point = new_point;
    *closest_distance = new_distance;
  }
}
}  // namespace turtle
