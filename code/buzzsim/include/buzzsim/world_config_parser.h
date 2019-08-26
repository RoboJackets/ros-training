#ifndef SRC_WORLD_CONFIG_PARSER_H
#define SRC_WORLD_CONFIG_PARSER_H

#include <vector>

#include <QImage>

#include <yaml-cpp/yaml.h>

#include <buzzsim/motion.h>
#include <buzzsim/obstacle.h>
#include <buzzsim/turtle.h>

class WorldConfigParser
{
public:
  struct SpawnOptions
  {
    std::vector<Obstacle> obstacles;
    std::vector<turtle::Turtle::Options> turtles;
  };

  void setImages(std::vector<QImage>&& images);

  SpawnOptions parseConfig(const std::string& path, const std::string& world_name) const;

  SpawnOptions parseWorld(const YAML::Node& world) const;

  /**
   * Returns default settings for spawn options.
   *
   * Spawns a turtle named "Oswin" at (0,0) with default motion limits.
   * @return Default settings for spawn options
   */
  SpawnOptions getSimpleOptions() const;

  /**
   * Returns default settings for motion limits.
   *
   * Acceleration of ±5 and velocity of ±2.
   * @return Default settings for motion limits.
   */
  motion::Limits getSimpleLimits() const;

private:
  std::vector<QImage> images_;
};

std::ostream& operator<<(std::ostream& os, const WorldConfigParser::SpawnOptions& options);

namespace YAML
{
template <>
struct convert<motion::Position>
{
  static bool decode(const Node& node, motion::Position& rhs);
};

template <>
struct convert<motion::Pose>
{
  static bool decode(const Node& node, motion::Pose& rhs);
};

template <>
struct convert<motion::Twist>
{
  static bool decode(const Node& node, motion::Twist& rhs);
};

template <>
struct convert<motion::State>
{
  static bool decode(const Node& node, motion::State& rhs);
};

template <>
struct convert<motion::AccelerationLimits>
{
  static bool decode(const Node& node, motion::AccelerationLimits& rhs);
};

template <>
struct convert<motion::TwistLimits>
{
  static bool decode(const Node& node, motion::TwistLimits& rhs);
};

template <>
struct convert<motion::Limits>
{
  static bool decode(const Node& node, motion::Limits& rhs);
};

template <>
struct convert<turtle::Turtle::Options>
{
  static bool decode(const Node& node, turtle::Turtle::Options& rhs);
};

template <>
struct convert<turtle::Turtle::PublishOptions>
{
  static bool decode(const Node& node, turtle::Turtle::PublishOptions& rhs);
};

template <>
struct convert<turtle::Turtle::ImuStdDevs>
{
  static bool decode(const Node& node, turtle::Turtle::ImuStdDevs& rhs);
};

template <>
struct convert<turtle::Lidar::Options>
{
  static bool decode(const Node& node, turtle::Lidar::Options& rhs);
};

template <>
struct convert<turtle::LidarPainter::Options>
{
  static bool decode(const Node& node, turtle::LidarPainter::Options& rhs);
};
}  // namespace YAML

#endif  // SRC_WORLD_CONFIG_PARSER_H
