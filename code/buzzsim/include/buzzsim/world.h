#ifndef SRC_WORLD_H
#define SRC_WORLD_H

#include <QPainter>

#include <buzzsim/world_config_parser.h>
#include <buzzsim/turtle.h>

class World
{
 public:
  using SpawnOptions = WorldConfigParser::SpawnOptions;

  World();
  World(std::vector<QImage>&& images);

  /**
   * Initializes the world with the options passed.
   * @param options Options used to initialize the world.
   */
  void init(const SpawnOptions& options);

  /**
   * Initializes the world with the yml file passed in the config_path parameter, if any.
   * Otherwise initializes with getSimpleOptions().
   */
  void init();

  void update();

  void paint(QPainter* painter);

 private:
  WorldConfigParser parser_;

  std::vector<std::unique_ptr<turtle::Turtle>> turtles_;
  std::vector<Obstacle> obstacles_;

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
};

#endif //SRC_WORLD_H
