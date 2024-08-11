#pragma once

#include <omp.h>

#include <boost/numeric/interval.hpp>
using Interval = boost::numeric::interval<double, boost::numeric::interval_lib::policies<boost::numeric::interval_lib::save_state<boost::numeric::interval_lib::rounded_transc_std<double>>, boost::numeric::interval_lib::checking_base<double>>>;

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

        // Apply the g_tilde offset to the voxel as described in the paper
        R3xS1_Voxel forwardOdometry(R3xS1 g_tilde, FT measurement) {
            Interval xInterval(bottomLeftPosition.x(), topRightPosition.x());
            Interval yInterval(bottomLeftPosition.y(), topRightPosition.y());
            Interval zInterval(bottomLeftPosition.z(), topRightPosition.z());
            Interval rInterval(bottomLeftRotation, topRightRotation);

            xInterval = xInterval + g_tilde.position.x() * boost::numeric::cos(rInterval) - g_tilde.position.y() * boost::numeric::sin(rInterval);
            yInterval = yInterval + g_tilde.position.x() * boost::numeric::sin(rInterval) + g_tilde.position.y() * boost::numeric::cos(rInterval);
            zInterval = zInterval + g_tilde.position.z() - measurement;
            
            R3xS1_Voxel newVoxel;
            newVoxel.bottomLeftPosition = Point(
                boost::numeric::lower(xInterval), boost::numeric::lower(yInterval), boost::numeric::lower(zInterval)
            );
            newVoxel.topRightPosition = Point(
                boost::numeric::upper(xInterval), boost::numeric::upper(yInterval), boost::numeric::upper(zInterval)
            );
            // newVoxel.bottomLeftRotation = bottomLeftRotation + g_tilde.orientation;
            // newVoxel.topRightRotation = topRightRotation + g_tilde.orientation;
            return newVoxel;
        }

        bool predicate(R3xS1 g_tilde, FT measurement, AABBTree& env) {
            R3xS1_Voxel newVoxel = forwardOdometry(g_tilde, measurement);

            // Hotfix: dialate the voxel a bit, to account for precision errors
            FT diameter = sqrt(
                (topRightPosition.x() - bottomLeftPosition.x()) * (topRightPosition.x() - bottomLeftPosition.x()) +
                (topRightPosition.y() - bottomLeftPosition.y()) * (topRightPosition.y() - bottomLeftPosition.y()) +
                (topRightPosition.z() - bottomLeftPosition.z()) * (topRightPosition.z() - bottomLeftPosition.z())
            );
            diameter = sqrt(diameter);
            FT factor = 0.0;

            newVoxel.bottomLeftPosition = Point(
                newVoxel.bottomLeftPosition.x() - diameter * factor,
                newVoxel.bottomLeftPosition.y() - diameter * factor,
                newVoxel.bottomLeftPosition.z() - diameter * factor
            );
            newVoxel.topRightPosition = Point(
                newVoxel.topRightPosition.x() + diameter * factor,
                newVoxel.topRightPosition.y() + diameter * factor,
                newVoxel.topRightPosition.z() + diameter * factor
            );

            Box query(newVoxel.bottomLeftPosition, newVoxel.topRightPosition);
            return env.do_intersect(query);
        }

        bool contains(R3xS1 q) {
            return q.position.x() >= bottomLeftPosition.x() && q.position.x() <= topRightPosition.x() &&
                q.position.y() >= bottomLeftPosition.y() && q.position.y() <= topRightPosition.y() &&
                q.position.z() >= bottomLeftPosition.z() && q.position.z() <= topRightPosition.z() &&
                q.orientation >= bottomLeftRotation && q.orientation <= topRightRotation;
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
    
    static VoxelCloud localize(AABBTree& env, OdometrySequence& odometrySequence, MeasurementSequence& measurementSequence, R3xS1_Voxel& boundingBox, int recursionDepth) {
        omp_set_num_threads(omp_get_max_threads());

        // Get squence of aggregated odometries
        OdometrySequence tildeOdometries;
        for (auto g : odometrySequence) {
            if (tildeOdometries.empty()) tildeOdometries.push_back(g);
            else tildeOdometries.push_back(g * tildeOdometries.back());
        }

        VoxelCloud voxels, localization;
        voxels.push_back(boundingBox);

        for (int i = 0; i < recursionDepth; i++) {
            localization.clear();
            #pragma omp parallel for
            for (auto v : voxels) {
                bool flag = true;
                for (int j = 0; j < tildeOdometries.size(); j++) {
                    if (measurementSequence[j] < 0) continue;
                    if (!v.predicate(tildeOdometries[j], measurementSequence[j], env)) {
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
        }

        return localization;
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