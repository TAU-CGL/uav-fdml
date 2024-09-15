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
    struct R3xS1 {
        Point position;
        FT orientation;

        R3xS1() : position(Point(0, 0, 0)), orientation(0) {}
        R3xS1(Point position, FT orientation) : position(position), orientation(orientation) {}

        R3xS1 operator*(const R3xS1& other) {
            FT newOrientation = orientation + other.orientation;
            FT x = other.position.x() + position.x() * cos(other.orientation) - position.y() * sin(other.orientation);
            FT y = other.position.y() + position.x() * sin(other.orientation) + position.y() * cos(other.orientation);
            FT z = other.position.z() + position.z();
            Point newPosition(x, y, z);
            return R3xS1(newPosition, newOrientation);
        }

        R3xS1 operator/(const R3xS1& other) {
            FT newOrientation = orientation - other.orientation;
            FT x = cos(other.orientation) * (position.x() - other.position.x()) + sin(other.orientation) * (position.y() - other.position.y());
            FT y = -sin(other.orientation) * (position.x() - other.position.x()) + cos(other.orientation) * (position.y() - other.position.y());
            FT z = position.z() - other.position.z();
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

        inline void _calcBoundX(FT blR, FT trR, FT gx, FT gy, FT& minX, FT& maxX) {
            std::vector<FT> thetas;
            thetas.push_back(blR);
            thetas.push_back(trR);
            for (int k = -3; k <= 3; k++) {
                FT tmp = atan(-gy / gx) + k * M_PI;
                if (tmp >= blR && tmp <= trR) thetas.push_back(tmp);
            }
            minX = INFTY, maxX = -INFTY;
            for (auto& theta : thetas) {
                FT x = gx * cos(theta) - gy * sin(theta);
                minX = MIN(minX, x);
                maxX = MAX(maxX, x);
            }
        }

        inline void _calcBoundY(FT blR, FT trR, FT gx, FT gy, FT& minY, FT& maxY) {
            std::vector<FT> thetas;
            thetas.push_back(blR);
            thetas.push_back(trR);
            for (int k = -4; k <= 4; k++) {
                FT tmp = atan(gx / gy) + k * M_PI;
                if (tmp >= blR && tmp <= trR) thetas.push_back(tmp);
            }
            minY = INFTY, maxY = -INFTY;
            for (auto& theta : thetas) {
                FT y = gx * sin(theta) + gy * cos(theta);
                minY = MIN(minY, y);
                maxY = MAX(maxY, y);
            }
        }

        // Apply the g_tilde offset to the voxel as described in the paper
        R3xS1_Voxel forwardOdometry(R3xS1 g_tilde, FT measurement, ErrorBounds errorBounds, int iteration) {
            // Find X axis bounds
            FT minX = INFTY, maxX = -INFTY;
            FT tmp1, tmp2;
            _calcBoundX(bottomLeftRotation, topRightRotation, 
                g_tilde.position.x() - errorBounds.errorOdometryX * iteration, 
                g_tilde.position.y() - errorBounds.errorOdometryY * iteration, tmp1, tmp2); 
                minX = MIN(minX, tmp1); maxX = MAX(maxX, tmp2);
            _calcBoundX(bottomLeftRotation, topRightRotation, 
                g_tilde.position.x() - errorBounds.errorOdometryX * iteration, 
                g_tilde.position.y() + errorBounds.errorOdometryY * iteration, tmp1, tmp2); 
                minX = MIN(minX, tmp1); maxX = MAX(maxX, tmp2);
            _calcBoundX(bottomLeftRotation, topRightRotation, 
                g_tilde.position.x() + errorBounds.errorOdometryX * iteration, 
                g_tilde.position.y() - errorBounds.errorOdometryY * iteration, tmp1, tmp2); 
                minX = MIN(minX, tmp1); maxX = MAX(maxX, tmp2);
            _calcBoundX(bottomLeftRotation, topRightRotation, 
                g_tilde.position.x() + errorBounds.errorOdometryX * iteration, 
                g_tilde.position.y() + errorBounds.errorOdometryY * iteration, tmp1, tmp2); 
                minX = MIN(minX, tmp1); maxX = MAX(maxX, tmp2);
            minX += bottomLeftPosition.x();
            maxX += topRightPosition.x();

            // Find Y axis bounds
            FT minY = INFTY, maxY = -INFTY;
            _calcBoundY(bottomLeftRotation, topRightRotation, 
                g_tilde.position.x() - errorBounds.errorOdometryX * iteration, 
                g_tilde.position.y() - errorBounds.errorOdometryY * iteration, tmp1, tmp2); 
                minY = MIN(minY, tmp1); maxY = MAX(maxY, tmp2);
            _calcBoundY(bottomLeftRotation, topRightRotation, 
                g_tilde.position.x() - errorBounds.errorOdometryX * iteration, 
                g_tilde.position.y() + errorBounds.errorOdometryY * iteration, tmp1, tmp2); 
                minY = MIN(minY, tmp1); maxY = MAX(maxY, tmp2);
            _calcBoundY(bottomLeftRotation, topRightRotation, 
                g_tilde.position.x() + errorBounds.errorOdometryX * iteration, 
                g_tilde.position.y() - errorBounds.errorOdometryY * iteration, tmp1, tmp2); 
                minY = MIN(minY, tmp1); maxY = MAX(maxY, tmp2);
            _calcBoundY(bottomLeftRotation, topRightRotation, 
                g_tilde.position.x() + errorBounds.errorOdometryX * iteration, 
                g_tilde.position.y() + errorBounds.errorOdometryY * iteration, tmp1, tmp2); 
                minY = MIN(minY, tmp1); maxY = MAX(maxY, tmp2);
            minY += bottomLeftPosition.y();
            maxY += topRightPosition.y();

            // Find Z axis bounds
            FT minZ = bottomLeftPosition.z() + g_tilde.position.z() - measurement - errorBounds.errorDistance - errorBounds.errorOdometryZ * iteration;
            FT maxZ = topRightPosition.z() + g_tilde.position.z() - measurement + errorBounds.errorDistance + errorBounds.errorOdometryZ * iteration;

            R3xS1_Voxel v;
            v.bottomLeftPosition = Point(minX, minY, minZ);
            v.topRightPosition = Point(maxX, maxY, maxZ);
            v.bottomLeftRotation = v.topRightRotation = 0;
            return v;
        }

        bool predicate(R3xS1 g_tilde, FT measurement, AABBTree& env, ErrorBounds errorBounds, int iteration) {
            R3xS1_Voxel v = forwardOdometry(g_tilde, measurement, errorBounds, iteration);
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
    
    static VoxelCloud localize(AABBTree& env, OdometrySequence& odometrySequence, MeasurementSequence& measurementSequence, R3xS1_Voxel& boundingBox, int recursionDepth, ErrorBounds errorBounds = ErrorBounds()) {
        omp_set_num_threads(omp_get_max_threads());

        // Get squence of aggregated odometries
        OdometrySequence tildeOdometries;
        for (auto g : odometrySequence) {
            if (tildeOdometries.empty()) tildeOdometries.push_back(g);
            else tildeOdometries.push_back(g * tildeOdometries.back());
        }

        VoxelCloud voxels, localization;
        R3xS1_Voxel bb1 = boundingBox, bb2 = boundingBox;
        bb1.bottomLeftRotation = 0;
        bb1.topRightRotation = M_PI;
        bb2.bottomLeftRotation = M_PI;
        bb2.topRightRotation = 2 * M_PI;
        voxels.push_back(bb1);
        voxels.push_back(bb2);


        // please write me code
        std::chrono::steady_clock::time_point begin, curr;    
        std::chrono::duration<double, std::milli> __duration;
        begin = std::chrono::steady_clock::now();

        for (int i = 0; i < recursionDepth; i++) {
            localization.clear();
            #pragma omp parallel for
            for (auto v : voxels) {
                bool flag = true;
                for (int j = 0; j < tildeOdometries.size(); j++) {
                    if (measurementSequence[j] < 0) continue;
                    if (!v.predicate(tildeOdometries[j], measurementSequence[j], env, errorBounds, j)) {
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

    static std::vector<R3xS1> clusterLocations(VoxelCloud& locations) {
        Point center(0, 0, 0);
        FT rotation = 0;
        for (auto v : locations) {
            center = Point(center.x() + (v.bottomLeftPosition.x() + v.topRightPosition.x()) / 2, center.y() + (v.bottomLeftPosition.y() + v.topRightPosition.y()) / 2, center.z() + (v.bottomLeftPosition.z() + v.topRightPosition.z()) / 2);
            rotation += (v.bottomLeftRotation + v.topRightRotation) / 2;
        }
        if (locations.size() > 0) {
            center = Point(center.x() / locations.size(), center.y() / locations.size(), center.z() / locations.size());
            rotation /= locations.size();
        }
        return {R3xS1(center, rotation)};

        // std::vector<VoxelCloud> clusters;
        // for (auto v : locations) {
        //     bool flag = false;
        //     for (auto cluster : clusters) {
        //         for (auto vc : cluster) {
        //             if (v.areNeighbors(vc)) {
        //                 cluster.push_back(v);
        //                 flag = true;
        //                 break;
        //             }
        //         }
        //     }
        //     if (!flag) clusters.push_back({v});
        // }

        // std::vector<R3xS1> centers;
        // for (auto cluster : clusters) {
        //     Point center(0, 0, 0);
        //     FT rotation = 0;
        //     for (auto v : cluster) {
        //         center = Point(center.x() + (v.bottomLeftPosition.x() + v.topRightPosition.x()) / 2, center.y() + (v.bottomLeftPosition.y() + v.topRightPosition.y()) / 2, center.z() + (v.bottomLeftPosition.z() + v.topRightPosition.z()) / 2);
        //         rotation += (v.bottomLeftRotation + v.topRightRotation) / 2;
        //     }
        //     center = Point(center.x() / cluster.size(), center.y() / cluster.size(), center.z() / cluster.size());
        //     rotation /= cluster.size();
        //     centers.push_back(R3xS1(center, rotation));
        // }
        // return centers;
    }

    static OdometrySequence getGroundTruths(OdometrySequence& odometrySequence, R3xS1 q0) {
        OdometrySequence groundTruths;
        for (auto g : odometrySequence) {
            if (groundTruths.empty()) groundTruths.push_back(g * q0);
            else groundTruths.push_back(g * groundTruths.back());
        }
        return groundTruths;
    }

    static MeasurementSequence getMeasurementSequence(AABBTree& env, OdometrySequence& groundTruths) {
        MeasurementSequence measurements;
        for (auto q : groundTruths) {
            measurements.push_back(q.measureDistance(env));
        }
        return measurements;
    }
    
}