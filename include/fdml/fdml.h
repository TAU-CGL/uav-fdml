#pragma once

// #include "kdtree.h"
#include "random.h"
// #include "geometry.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

using K = CGAL::Simple_cartesian<double>;

using FT = K::FT;
using Ray = K::Ray_3;
using Line = K::Line_3;
using Point = K::Point_3;
using Triangle = K::Triangle_3;
using AABBTree = CGAL::AABB_tree<CGAL::AABB_traits<K, CGAL::AABB_triangle_primitive<K, std::list<Triangle>::iterator>>>;

struct R3xS1 {
    Point position;
    FT orientation;

    R3xS1() : position(Point(0, 0, 0)), orientation(0) {}
    R3xS1(Point position, FT orientation) : position(position), orientation(orientation) {}

    R3xS1 operator*(const R3xS1& other) {
        FT newOrientation = orientation + other.orientation;
        FT x = position.x() + cos(orientation) * other.position.x() - sin(orientation) * other.position.y();
        Point newPosition;

        return R3xS1(newPosition, newOrientation);
    }
};