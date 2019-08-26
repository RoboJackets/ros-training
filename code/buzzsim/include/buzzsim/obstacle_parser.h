#ifndef SRC_OBSTACLE_PARSER_H
#define SRC_OBSTACLE_PARSER_H

#include <yaml-cpp/yaml.h>

#include <buzzsim/motion.h>
#include <buzzsim/obstacle.h>

struct Polygon
{
  std::vector<motion::Position> points;

  [[nodiscard]] Obstacle toObstacle() const;
};

struct Barrel
{
  motion::Position center;
  constexpr static double radius = 0.5;

  [[nodiscard]] Obstacle toObstacle() const;
};

namespace YAML
{
template <>
struct convert<Polygon>
{
  static bool decode(const Node& node, Polygon& rhs);
};

template <>
struct convert<Barrel>
{
  static bool decode(const Node& node, Barrel& rhs);
};

template <>
struct convert<Obstacle>
{
  static bool decode(const Node& node, Obstacle& rhs);
};
}  // namespace YAML

#endif  // SRC_OBSTACLE_PARSER_H
