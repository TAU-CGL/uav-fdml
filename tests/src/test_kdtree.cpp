#include <fmt/core.h>
#include <gtest/gtest.h>
#include <boost/container/vector.hpp>

using fmt::format, fmt::print;

#include "se3loc/se3loc.h"

constexpr uint8_t DIM = 2;
constexpr int32_t NUM_POINTS = 16;

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
    // for (auto point : kdtree._points) {
    //     print("{} {}; ", point[0], point[1]);
    // }
    // print("\n");

    boost::container::vector<se3loc::Point<DIM, double>> qs;
    for (int32_t i = 0; i < NUM_POINTS; ++i) qs.push_back(se3loc::Point<DIM, double>::uniform());
    int tmp = 0;
    for (auto q : qs) {
        if (tmp++ != 7) continue; 
        // Get ground truth
        se3loc::Point<DIM, double> gt;
        double minDist = 1e30;
        for (auto p : points) {
            double dist = std::sqrt((q - p).squaredNorm());
            if (dist < minDist) {
                gt = p;
                minDist = dist;
            }
        }

        // Get nearest neighbor
        se3loc::Point<DIM, double> neighbor = kdtree.nearestNeighbor(q);

        double err = abs(std::sqrt((q - gt).squaredNorm()) - std::sqrt((q - neighbor).squaredNorm()));
        EXPECT_LT(err, 1e-4);


    }
}