#include <gtest/gtest.h>
#include <boost/container/vector.hpp>

#include "se3loc/se3loc.h"

constexpr uint32_t NUM_POINTS = 1024;

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