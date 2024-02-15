#include <fmt/core.h>
#include <gtest/gtest.h>
#include <boost/container/vector.hpp>

using fmt::format, fmt::print;

#include "se3loc/se3loc.h"

constexpr uint32_t NUM_POINTS = 4096 * 2;

TEST(KDTreeTest, TestPointsPresorting) {
    se3loc::Random::seed(0);
    boost::container::vector<se3loc::Point3<double>> points;
    for (uint32_t i = 0; i < NUM_POINTS; ++i) points.push_back(se3loc::Point3<double>::uniform());

    se3loc::KDTreePoints<3, double> kdpoints;
    kdpoints.init(points);

    for (uint32_t i = 0; i < NUM_POINTS - 1; ++i) {
        for (uint8_t j = 0; j < 3; j++) {
            EXPECT_LE(
                points[kdpoints.sorted[j * NUM_POINTS + i]][j],
                points[kdpoints.sorted[j * NUM_POINTS + i + 1]][j]
            );
        }
    }
}

TEST(KDTreeTest, TestPointsMedianValue) {
    se3loc::Random::seed(0);
    boost::container::vector<se3loc::Point3<double>> points;
    for (uint32_t i = 0; i < NUM_POINTS; ++i) points.push_back(se3loc::Point3<double>::uniform());

    se3loc::KDTreePoints<3, double> kdpoints;
    kdpoints.init(points);

    boost::container::vector<uint32_t> indices;
    for (uint32_t idx = 0; idx < points.size(); idx++) indices.push_back(idx);

    for (uint8_t j = 0; j < 3; j++) {
        double median = kdpoints.getMedianValue(j, indices);
        int32_t cntLower = 0, cntGreater = 0;
        for (uint32_t i = 0; i < NUM_POINTS; i++) {
            if (points[i][j] > median) cntGreater++;
            if (points[i][j] < median) cntLower++;
        }
        EXPECT_LE(abs(cntGreater - cntLower), 1) << format(
            "Median is {}, lower:{} greater:{}\n", median, cntLower, cntGreater
        );
    }
}

TEST(KDTreeTest, TestBuildTree) {
    se3loc::Random::seed(0);
    boost::container::vector<se3loc::Point3<double>> points;
    for (uint32_t i = 0; i < NUM_POINTS; ++i) {
        points.push_back(se3loc::Point3<double>::uniform());
        // print("{} {} {}\n", points[i][0], points[i][1], points[i][2]);
    }



    se3loc::KDTree<3, double> kdtree;
    kdtree.fit(points);
}