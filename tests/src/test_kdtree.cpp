#include "tests.h"

// constexpr uint8_t DIM = 3;
// constexpr int32_t NUM_POINTS = 4096;

// TEST(KDTreeTest, TestBuildTree) {
//     fdml::Random::seed(0);
//     boost::container::vector<fdml::Point<DIM, double>> points;
//     for (int32_t i = 0; i < NUM_POINTS; ++i) points.push_back(fdml::Point<DIM, double>::uniform());
//     fdml::KDTree<DIM, double> kdtree;
//     kdtree.fit(points);
// }

// TEST(KDTreeTest, TestNearestNeighbors) {
//     fdml::Random::seed(0);
//     boost::container::vector<fdml::Point<DIM, double>> points;
//     for (int32_t i = 0; i < NUM_POINTS; ++i) points.push_back(fdml::Point<DIM, double>::uniform());
//     fdml::KDTree<DIM, double> kdtree;
//     kdtree.fit(points);
    
//     boost::container::vector<fdml::Point<DIM, double>> qs;
//     for (int32_t i = 0; i < NUM_POINTS; ++i) qs.push_back(fdml::Point<DIM, double>::uniform());
//     int tmp = 0;
//     for (auto q : qs) {
//         // Get ground truth
//         fdml::Point<DIM, double> gt;
//         double minDist = -1;
//         for (auto p : points) {
//             double dist = std::sqrt((q - p).squaredNorm());
//             if (minDist < 0 || dist < minDist) {
//                 gt = p;
//                 minDist = dist;
//             }
//         }

//         // Get nearest neighbor
//         fdml::Point<DIM, double> neighbor = kdtree.nearestNeighbor(q);

//         // Check that we get exactly the same point
//         for (int j = 0; j < DIM; j++) EXPECT_EQ(neighbor[j], gt[j]);
//     }
// }

TEST_MAIN()