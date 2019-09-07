#include <buzzsim/obstacle.h>

std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle)
{
  os << "Obstacle(";
  for (const auto& point : obstacle.points)
  {
    os << point << " ";
  }
  os << ")";
  return os;
}
