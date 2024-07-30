#pragma once

#include <variant>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <boost/optional.hpp>

using CGAL_Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using CGAL_Point_3 = CGAL_Kernel::Point_3;
using CGAL_Segment_3 = CGAL_Kernel::Segment_3;
using CGAL_Triangle_3 = CGAL_Kernel::Triangle_3;


glm::vec3 cgal2glm(CGAL_Point_3 p) {
    return glm::vec3(
        CGAL::to_double<CGAL_Kernel::FT>(p.x()), 
        CGAL::to_double<CGAL_Kernel::FT>(p.y()), 
        CGAL::to_double<CGAL_Kernel::FT>(p.z()));
}