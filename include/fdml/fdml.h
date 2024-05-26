#pragma once

// #include "kdtree.h"
#include "random.h"
// #include "geometry.h"

// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

// using K = CGAL::Exact_predicates_exact_constructions_kernel;
using K = CGAL::Simple_cartesian<double>;

using FT = K::FT;
using Ray = K::Ray_3;
using Line = K::Line_3;
using Point = K::Point_3;
using Vector = K::Vector_3;
using Triangle = K::Triangle_3;
using AABBTree = CGAL::AABB_tree<CGAL::AABB_traits<K, CGAL::AABB_triangle_primitive<K, std::list<Triangle>::iterator>>>;

namespace fdml {
    struct R3xS1 {
        Point position;
        FT orientation;

        R3xS1() : position(Point(0, 0, 0)), orientation(0) {}
        R3xS1(Point position, FT orientation) : position(position), orientation(orientation) {}

        R3xS1 operator*(const R3xS1& other) {
            FT newOrientation = orientation + other.orientation;
            FT x = position.x() + other.position.x() * cos(orientation) - other.position.y() * sin(orientation);
            FT y = position.y() + other.position.x() * sin(orientation) + other.position.y() * cos(orientation);
            FT z = position.z() + other.position.z();
            Point newPosition(x, y, z);
            return R3xS1(newPosition, newOrientation);
        }

        double measureDistance(AABBTree& room) {
            Point down(this->position.x(), this->position.y(), this->position.z() - 1.0);
            Ray ray(this->position, down);
            auto result = room.first_intersection(ray);
            if (!result.has_value()) { return -1; }
            const Point* p = boost::get<Point>(&(result->first));
            return sqrt(CGAL::squared_distance(this->position, *p));
        }
    };
    using OdometrySequence = std::vector<R3xS1>;
    using MeasurementSequence = std::vector<double>;

    
}