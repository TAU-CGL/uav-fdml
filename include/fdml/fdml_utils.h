#pragma once

#include <fdml/fdml.h>

#include <set>
#include <ctime>
#include <list>
#include <vector>
#include <memory>

#include <fmt/core.h>

#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/K_neighbor_search.h>

using Kd_tree_Traits = CGAL::Search_traits_3<K>;
using Kd_tree = CGAL::Kd_tree<Kd_tree_Traits>;
using K_neighbor_search = CGAL::K_neighbor_search<Kd_tree_Traits>;

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>

namespace fdml {

    class Random {
    public:
        static double randomDouble() {
            boost::random::uniform_real_distribution<> dist(0.0, 1.0);
            return dist(instance()->rng);
        }

        static double randomGaussian(double sigma2) {
            boost::random::normal_distribution<> dist(0.0, sigma2);
            return dist(instance()->rng);
        }

        static int randomInt() {
            // Like randomDouble but for int
            double d = randomDouble();
            return (int)(d * (double)(0x01 << 30));
        }

        static void seed(int32_t seed = -1) {
            if (seed < 0) seed = std::time(0);
            instance()->rng.seed((uint32_t)seed);
        }

    private:
        static Random* instance() {
            static Random* _i = new Random();
            return _i;
        }

        Random() {}
        boost::mt19937 rng;
    };

    class ExperimentEnv {
    public:
        ExperimentEnv() {}

        void loadTriangles(std::list<Triangle> triangles) {
            m_triangles = triangles;
            m_tree = AABBTree(m_triangles.begin(), m_triangles.end());
            m_tree.accelerate_distance_queries();

            // Compute bounding box
            m_boundingBox.bottomLeftPosition = Point(m_tree.bbox().xmin(), m_tree.bbox().ymin(), m_tree.bbox().zmin());
            m_boundingBox.topRightPosition = Point(m_tree.bbox().xmax(), m_tree.bbox().ymax(), m_tree.bbox().zmax());
            m_boundingBox.bottomLeftRotation = 0.0f;
            m_boundingBox.topRightRotation = 2.f * M_PI;
        }

        R3xS1_Voxel& getBoundingBox() {
            return m_boundingBox;
        }

        AABBTree& getTree() {
            return m_tree;
        }

    public:

    private:
        std::list<Triangle> m_triangles;
        AABBTree m_tree;
        R3xS1_Voxel m_boundingBox;

        Point samplePointInBB() {
            FT x = Random::randomDouble() * (m_boundingBox.topRightPosition.x() - m_boundingBox.bottomLeftPosition.x()) + m_boundingBox.bottomLeftPosition.x();
            FT y = Random::randomDouble() * (m_boundingBox.topRightPosition.y() - m_boundingBox.bottomLeftPosition.y()) + m_boundingBox.bottomLeftPosition.y();
            FT z = Random::randomDouble() * (m_boundingBox.topRightPosition.z() - m_boundingBox.bottomLeftPosition.z()) + m_boundingBox.bottomLeftPosition.z();
            return Point(x, y, z);
        }
    };
}