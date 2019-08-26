#ifndef SRC_CGAL_TYPES_H
#define SRC_CGAL_TYPES_H

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_polygon_visibility_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Rotational_sweep_visibility_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<Kernel>;
using Polygon_set_2 = CGAL::Polygon_set_2<Kernel>;
using Traits_2 = CGAL::Arr_segment_traits_2<Kernel>;
using Arrangement_2 = CGAL::Arrangement_2<Traits_2>;
using Face_handle = Arrangement_2 ::Face_handle;
using Edge_const_iterator = Arrangement_2::Edge_const_iterator;
using Ccb_halfedge_circulator = Arrangement_2::Ccb_halfedge_circulator;
using TEV = CGAL::Triangular_expansion_visibility_2<Arrangement_2>;
using RSV = CGAL::Rotational_sweep_visibility_2<Arrangement_2>;

#endif //SRC_CGAL_TYPES_H
