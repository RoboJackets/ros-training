#ifndef SRC_PRINT_UTILS_H
#define SRC_PRINT_UTILS_H

#include <buzzsim/sensors/cgal_types.h>

std::ostream& operator<<(std::ostream& os, const Polygon_2& polygon);

std::ostream& operator<<(std::ostream& os, const Polygon_with_holes_2& polygon);

#endif //SRC_PRINT_UTILS_H
