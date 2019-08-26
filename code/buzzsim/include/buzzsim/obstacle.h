#ifndef SRC_OBSTACLE_H
#define SRC_OBSTACLE_H

#include <buzzsim/motion.h>

struct Obstacle
{
  std::vector<motion::Position> points;
};

std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle);

#endif  // SRC_OBSTACLE_H
