#include <buzzsim/world.h>

World::World() : pnh{ "~" }
{
}

World::World(std::vector<QImage>&& images) : pnh{ "~" }
{
  parser_.setImages(std::move(images));
}

void World::init()
{
  std::string path;
  std::string world_name;

  pnh.getParam("config_path", path);
  pnh.getParam("world_name", world_name);

  if (!path.empty() && !world_name.empty())
  {
    init(parser_.parseConfig(path, world_name));
  }
  else
  {
    init(parser_.getSimpleOptions());
  }
}

void World::init(const WorldConfigParser::SpawnOptions& options)
{
  turtles_.clear();
  obstacles_ = options.obstacles;

  for (const auto& option : options.turtles)
  {
    turtles_.emplace_back(std::make_unique<turtle::Turtle>(option, &obstacles_));
  }
}

void World::update()
{
  for (auto& turtle : turtles_)
  {
    turtle->updatePose();
  }
}

void World::paint(QPainter* painter)
{
  for (auto& turtle : turtles_)
  {
    turtle->paint(*painter);
  }
}
