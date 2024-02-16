#include <fmt/core.h>
#include <gtest/gtest.h>
#include <boost/container/vector.hpp>

using fmt::format, fmt::print;

#include "se3loc/se3loc.h"

constexpr uint8_t DIM = 3;
constexpr int32_t NUM_POINTS = 4096;

TEST(KDTreeTest, TestBuildTree) {
    se3loc::Random::seed(0);
    boost::container::vector<se3loc::Point<DIM, double>> points;
    for (int32_t i = 0; i < NUM_POINTS; ++i) points.push_back(se3loc::Point<DIM, double>::uniform());
    se3loc::KDTree<DIM, double> kdtree;
    kdtree.fit(points);
}

TEST(KDTreeTest, TestNearestNeighbors) {
    se3loc::Random::seed(0);
    boost::container::vector<se3loc::Point<DIM, double>> points;
    for (int32_t i = 0; i < NUM_POINTS; ++i) points.push_back(se3loc::Point<DIM, double>::uniform());
    se3loc::KDTree<DIM, double> kdtree;
    kdtree.fit(points);
    
    boost::container::vector<se3loc::Point<DIM, double>> qs;
    for (int32_t i = 0; i < NUM_POINTS; ++i) qs.push_back(se3loc::Point<DIM, double>::uniform());
    int tmp = 0;
    for (auto q : qs) {
        // Get ground truth
        se3loc::Point<DIM, double> gt;
        double minDist = -1;
        for (auto p : points) {
            double dist = std::sqrt((q - p).squaredNorm());
            if (minDist < 0 || dist < minDist) {
                gt = p;
                minDist = dist;
            }
        }

        // Get nearest neighbor
        se3loc::Point<DIM, double> neighbor = kdtree.nearestNeighbor(q);

        // Check that we get exactly the same point
        for (int j = 0; j < DIM; j++) EXPECT_EQ(neighbor[j], gt[j]);
    }
}