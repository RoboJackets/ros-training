#include <buzzsim/sensors/print_utils.h>

std::ostream& operator<<(std::ostream& os, const Polygon_2& polygon)
{
  os << "[ " << polygon.size() << " vertices:";
  for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); it++)
  {
    os << " (" << *it << ")";
  }
  os << " ]";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Polygon_with_holes_2& polygon)
{
  if (!polygon.is_unbounded())
  {
    os << "{ Outer boundary =";
    os << polygon.outer_boundary() << std::endl;
  }
  else
  {
    os << "{ Unbounded polygon." << std::endl;
  }

  size_t k = 1;
  os << "\t" << polygon.number_of_holes() << " holes: " << std::endl;
  for (auto it = polygon.holes_begin(); it != polygon.holes_end(); it++)
  {
    os << "\tHole " << k << " = " << *it << std::endl;
  }
  os << " }";

  return os;
}
