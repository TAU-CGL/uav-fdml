#pragma once

#include <ctime>

#include <omp.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

using K = CGAL::Simple_cartesian<double>;
using FT = K::FT;
using Ray = K::Ray_3;
using Line = K::Line_3;
using Point = K::Point_3;
using Vector = K::Vector_3;
using Box = K::Iso_cuboid_3;
using Triangle = K::Triangle_3;
using AABBTree = CGAL::AABB_tree<CGAL::AABB_traits<K, CGAL::AABB_triangle_primitive<K, std::list<Triangle>::iterator>>>;

#define MIN(a, b) ((a) <= (b) ? (a) : (b))
#define MAX(a, b) ((a) >= (b) ? (a) : (b))
#define INFTY 1e10
#ifndef M_PI
#define M_PI 3.14159265358979323846 // Fix for windows
#endif
#define TIMEOUT 30

namespace fdml {
    struct R3xS2 {
        Point position;
        Point direction;

        R3xS2() : position(Point(0,0,0)), direction(Point(1,0,0)) {}
        R3xS2(Point position, Point direction) : position(position), direction(direction) {}
        
        double measureDistance(AABBTree& room) {
            Point target(position.x() + direction.x(), position.y() + direction.y(), position.z() + direction.z()); 
            Ray ray(this->position, target);
            auto result = room.first_intersection(ray);
            if (!result.has_value()) { return -1; }
            const Point* p = boost::get<Point>(&(result->first));
            return sqrt(CGAL::squared_distance(this->position, *p));
        }
    };

    struct R3xS1 {
        Point position;
        FT orientation;

        R3xS1() : position(Point(0, 0, 0)), orientation(0) {}
        R3xS1(Point position, FT orientation) : position(position), orientation(orientation) {}

        R3xS2 operator*(const R3xS2& other) {
            FT px = position.x() + cos(orientation) * other.position.x() - sin(orientation) * other.position.y();
            FT py = position.y() + sin(orientation) * other.position.x() + cos(orientation) * other.position.y();
            FT pz = position.z() + other.position.z();
            FT dx = cos(orientation) * other.direction.x() - sin(orientation) * other.direction.y();
            FT dy = sin(orientation) * other.direction.x() + cos(orientation) * other.direction.y();
            return R3xS2(Point(px, py, pz), Point(dx, dy, other.direction.z()));
        }
    };
    using OdometrySequence = std::vector<R3xS2>;
    using MeasurementSequence = std::vector<double>;

    struct ErrorBounds {
        FT errorOdometryX, errorOdometryY, errorOdometryZ, errorOdometryR;
        FT errorDistance;

        ErrorBounds() : errorOdometryX(0), errorOdometryY(0), errorOdometryZ(0), errorOdometryR(0), errorDistance(0) {}
    };

    struct R3xS1_Voxel {
    public:
        Point bottomLeftPosition, topRightPosition;
        FT bottomLeftRotation, topRightRotation;

        static std::vector<R3xS1_Voxel> splitVoxels(std::vector<R3xS1_Voxel>& in) {
            std::vector<R3xS1_Voxel> out;
            for (auto& v : in) v.split(out);
            return out;
        }
        void split(std::vector<R3xS1_Voxel>& out) {
            std::vector<R3xS1_Voxel> temp, temp2;
            this->splitSingleAxis(temp, SplitAxis::SPLIT_X);
            for (auto& v : temp) v.splitSingleAxis(temp2, SplitAxis::SPLIT_Y); temp.clear();
            for (auto& v : temp2) v.splitSingleAxis(temp, SplitAxis::SPLIT_Z); temp2.clear();
            for (auto& v: temp) v.splitSingleAxis(out, SplitAxis::SPLIT_R);
        }

        FT volume() {
            return (topRightPosition.x() - bottomLeftPosition.x()) * 
                (topRightPosition.y() - bottomLeftPosition.y()) * 
                (topRightPosition.z() - bottomLeftPosition.z()) * 
                (topRightRotation - bottomLeftRotation);
        }

        FT volumeXYT() {
            return (topRightPosition.x() - bottomLeftPosition.x()) * 
                (topRightPosition.y() - bottomLeftPosition.y()) * 
                (topRightRotation - bottomLeftRotation);
        }


        inline Point gamma_gd(FT theta, R3xS2 g, FT d) {
            // gamma_gd(theta) = R_theta * (g_p + d * g_r)
            FT cost = cos(theta), sint = sin(theta);
            FT x = cost * (g.position.x() + d * g.direction.x()) - sint * (g.position.y() + d * g.direction.y());
            FT y = sint * (g.position.x() + d * g.direction.x()) + cost * (g.position.y() + d * g.direction.y());
            FT z = g.position.z() + d * g.direction.z();
            return Point(x, y, z);
        }

        R3xS1_Voxel forwardOdometry(R3xS2 g, FT d, FT errorBound) {
            FT alpha1 = g.position.x() + d * g.direction.x();
            FT alpha2 = g.position.y() + d * g.direction.y();
            FT alpha3 = g.position.z() + d * g.direction.z();

            FT beta1 = atan(-alpha2 / alpha1);
            FT beta2 = atan(alpha1 / alpha2);

            std::vector<FT> thetas;
            thetas.push_back(this->bottomLeftRotation);
            thetas.push_back(this->topRightRotation);
            for (int k = -3; k < 3; k++) {
                if (beta1 + k * M_PI <= topRightRotation && beta1 + k * M_PI >= bottomLeftRotation) thetas.push_back(beta1 + k * M_PI);
                if (beta2 + k * M_PI <= topRightRotation && beta2 + k * M_PI >= bottomLeftRotation) thetas.push_back(beta2 + k * M_PI);
            }

            FT minX = INFINITY, maxX = -INFINITY;
            FT minY = INFINITY, maxY = -INFINITY;
            for (FT theta : thetas) {
                Point tmp = gamma_gd(theta, g, d);
                minX = MIN(minX, tmp.x()); maxX = MAX(maxX, tmp.x());
                minY = MIN(minY, tmp.y()); maxY = MAX(maxY, tmp.y());
            }
            minX += bottomLeftPosition.x(); maxX += topRightPosition.x();
            minY += bottomLeftPosition.y(); maxY += topRightPosition.y();
            FT minZ = bottomLeftPosition.z() + alpha3;
            FT maxZ = topRightPosition.z() + alpha3;

            R3xS1_Voxel v;
            v.bottomLeftPosition = Point(minX - errorBound, minY - errorBound, minZ - errorBound);
            v.topRightPosition = Point(maxX + errorBound, maxY + errorBound, maxZ + errorBound);
            v.bottomLeftRotation = v.topRightRotation = 0;
            return v;
        }

        bool predicate(R3xS2 g_tilde, FT measurement, AABBTree& env, FT errorBound, int iteration) {
            R3xS1_Voxel v = forwardOdometry(g_tilde, measurement, errorBound);
            Box query(v.bottomLeftPosition, v.topRightPosition);
            return env.do_intersect(query);
        }

        bool contains(R3xS1 q) {
            return q.position.x() >= bottomLeftPosition.x() && q.position.x() <= topRightPosition.x() &&
                q.position.y() >= bottomLeftPosition.y() && q.position.y() <= topRightPosition.y() &&
                q.position.z() >= bottomLeftPosition.z() && q.position.z() <= topRightPosition.z() &&
                q.orientation >= bottomLeftRotation && q.orientation <= topRightRotation;
        }

        void expand(int times) {
            FT dx = times * (topRightPosition.x() - bottomLeftPosition.x());
            FT dy = times * (topRightPosition.y() - bottomLeftPosition.y());
            FT dz = times * (topRightPosition.z() - bottomLeftPosition.z());
            FT dr = times * (topRightRotation - bottomLeftRotation);
            bottomLeftPosition = Point(bottomLeftPosition.x() - dx, bottomLeftPosition.y() - dy, bottomLeftPosition.z() - dz);
            topRightPosition = Point(topRightPosition.x() + dx, topRightPosition.y() + dy, topRightPosition.z() + dz);
            bottomLeftRotation -= dr;
            topRightRotation += dr;
        }

        bool areNeighbors(R3xS1_Voxel& other) {
            return (topRightPosition.x() == other.bottomLeftPosition.x() || bottomLeftPosition.x() == other.topRightPosition.x()) &&
                (topRightPosition.y() == other.bottomLeftPosition.y() || bottomLeftPosition.y() == other.topRightPosition.y()) &&
                (topRightPosition.z() == other.bottomLeftPosition.z() || bottomLeftPosition.z() == other.topRightPosition.z()) &&
                (topRightRotation == other.bottomLeftRotation || bottomLeftRotation == other.topRightRotation);
        }

    private:
        enum class SplitAxis { SPLIT_X, SPLIT_Y, SPLIT_Z, SPLIT_R };
        void splitSingleAxis(std::vector<R3xS1_Voxel>& out, SplitAxis axis) {
            R3xS1_Voxel left = *this, right = *this;
            switch(axis) {
                case SplitAxis::SPLIT_X:
                    left.topRightPosition = Point((bottomLeftPosition.x() + topRightPosition.x()) / 2, topRightPosition.y(), topRightPosition.z());
                    right.bottomLeftPosition = Point((bottomLeftPosition.x() + topRightPosition.x()) / 2, bottomLeftPosition.y(), bottomLeftPosition.z());
                    break;
                case SplitAxis::SPLIT_Y:
                    left.topRightPosition = Point(topRightPosition.x(), (bottomLeftPosition.y() + topRightPosition.y()) / 2, topRightPosition.z());
                    right.bottomLeftPosition = Point(bottomLeftPosition.x(), (bottomLeftPosition.y() + topRightPosition.y()) / 2, bottomLeftPosition.z());
                    break;
                case SplitAxis::SPLIT_Z:
                    left.topRightPosition = Point(topRightPosition.x(), topRightPosition.y(), (bottomLeftPosition.z() + topRightPosition.z()) / 2);
                    right.bottomLeftPosition = Point(bottomLeftPosition.x(), bottomLeftPosition.y(), (bottomLeftPosition.z() + topRightPosition.z()) / 2);
                    break;
                case SplitAxis::SPLIT_R:
                    left.topRightRotation = (bottomLeftRotation + topRightRotation) / 2;
                    right.bottomLeftRotation = (bottomLeftRotation + topRightRotation) / 2;
                    break;
            }
            out.push_back(left);
            out.push_back(right);
        }
        
    };
    using VoxelCloud = std::vector<R3xS1_Voxel>;
    
    static VoxelCloud localize(AABBTree& env, OdometrySequence& odometrySequence, MeasurementSequence& measurementSequence, R3xS1_Voxel& boundingBox, int recursionDepth, FT errorBound = 0) {
        omp_set_num_threads(omp_get_max_threads());

        // Get squence of aggregated odometries
        // OdometrySequence tildeOdometries;
        // for (auto g : odometrySequence) {
        //     if (tildeOdometries.empty()) tildeOdometries.push_back(g);
        //     else tildeOdometries.push_back(g * tildeOdometries.back());
        // }

        VoxelCloud voxels, localization;
        R3xS1_Voxel bb1 = boundingBox, bb2 = boundingBox;
        bb1.bottomLeftRotation = 0;
        bb1.topRightRotation = M_PI;
        bb2.bottomLeftRotation = M_PI;
        bb2.topRightRotation = 2 * M_PI;
        voxels.push_back(bb1);
        voxels.push_back(bb2);

        std::chrono::steady_clock::time_point begin, curr;    
        std::chrono::duration<double, std::milli> __duration;
        begin = std::chrono::steady_clock::now();

        for (int i = 0; i < recursionDepth; i++) {
            localization.clear();
            #pragma omp parallel for
            for (auto v : voxels) {
                bool flag = true;
                for (int j = 0; j < odometrySequence.size(); j++) {
                    if (measurementSequence[j] < 0) continue;
                    if (!v.predicate(odometrySequence[j], measurementSequence[j], env, errorBound, j)) {
                        flag = false;
                        break;
                    }
                }
                if (flag) {
                    #pragma omp critical
                    localization.push_back(v);
                }
            }
            voxels.clear();
            for (auto v : localization) v.split(voxels);

            curr = std::chrono::steady_clock::now();
            __duration = curr - begin;
            if (__duration.count() > TIMEOUT * 1000) {
                return VoxelCloud();
            }
        }

        // VoxelCloud clean;
        // for (auto v : localization) {
        //     if (!env.do_intersect(Box(v.bottomLeftPosition, v.topRightPosition))) clean.push_back(v);
        // }
        // localization = clean;

        return localization;
    }
    
}