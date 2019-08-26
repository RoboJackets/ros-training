#include <buzzsim/obstacle_parser.h>

Obstacle Polygon::toObstacle() const
{
  return { points };
}

Obstacle Barrel::toObstacle() const
{
  constexpr int num_points = 64;
  constexpr double d_theta = (2 * M_PI) / num_points;

  std::vector<motion::Position> points;
  for (int i = 0; i < num_points; i++)
  {
    double theta = i * d_theta;

    double x = center.x + radius * sin(theta);
    double y = center.y + radius * cos(theta);

    motion::Position point{ x, y };
    points.emplace_back(point);
  }

  return { points };
}

namespace YAML
{
bool convert<Polygon>::decode(const Node& node, Polygon& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  if (!node["points"].IsDefined() || !node["points"].IsSequence())
  {
    return false;
  }

  for (const auto& point_node : node["points"])
  {
    if (!point_node.IsSequence())
    {
      return false;
    }
    motion::Position point{};

    if (point_node.size() != 2 || !point_node[0].IsScalar() || !point_node[1].IsScalar())
    {
      return false;
    }
    point.x = point_node[0].as<double>();
    point.y = point_node[1].as<double>();

    rhs.points.emplace_back(point);
  }

  return true;
}

bool convert<Barrel>::decode(const Node& node, Barrel& rhs)
{
  if (!node.IsMap())
  {
    return false;
  }

  auto center_node = node["center"];
  if (!center_node || !center_node.IsSequence())
  {
    return false;
  }

  if (center_node.size() != 2 || !center_node[0].IsScalar() || !center_node[1].IsScalar())
  {
    return false;
  }

  rhs.center.x = center_node[0].as<double>();
  rhs.center.y = center_node[1].as<double>();

  return true;
}

bool convert<Obstacle>::decode(const Node& node, Obstacle& rhs)
{
  if (node.IsMap())
  {
    if (node["type"].IsDefined())
    {
      std::string type = node["type"].as<std::string>();
      if (type == "barrel")
      {
        rhs = node.as<Barrel>().toObstacle();
        return true;
      }
      else if (type == "polygon")
      {
        rhs = node.as<Polygon>().toObstacle();
        return true;
      }
    }
  }

  return false;
}
}  // namespace YAML
