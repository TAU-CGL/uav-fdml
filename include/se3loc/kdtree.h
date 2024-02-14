#pragma once

#include <boost/container/vector.hpp>
#include <boost/range/algorithm/sort.hpp>

#include "geometry.h"

namespace se3loc {
    /////////////////////////  
    // A single KDTree node
    /////////////////////////
    template <int dim, typename dtype>
    struct KDTreeNode {
        uint8_t cutDim;
        dtype cutValue;
        boost::container::vector<Point<dim, dtype>> points;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Store the points and also sort only once.
    // The array `sorted` is of size dim * points.size(), each points.size() are points sorder by some dimension.
    // Each item in sorted points so some point in `points`.
    // Store in grey points that are already added to KDtree, that is, grey[point_idx] == 1 iff we already added point_idx to the tree.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <int dim, typename dtype>
    struct KDTreePoints {
        boost::container::vector<Point<dim, dtype>> points;
        boost::container::vector<uint32_t> sorted;
        boost::container::vector<uint8_t> grey; // Since this is a boolean array we can further optimize down the size
        
        void init(boost::container::vector<Point<dim, dtype>> points) {
            this->points = points;
            this->sorted = boost::container::vector<uint32_t>(dim * this->points.size());
            this->grey = boost::container::vector<uint8_t>(this->points.size(), 0);

            size_t N = this->points.size();
            for (int i = 0; i < N; i++) for (int j = 0; j < dim; j++) this->sorted[j * N + i] = i; // keep arrays sequential

            for (int j = 0; j < dim; j++) {
                std::sort(
                    &this->sorted[j * N], &this->sorted[j * N] + N,
                    [j, points](const int32_t& a, const int32_t& b) {
                        return points[a].data[j] < points[b].data[j];
                    });
            }
        }
    };

    //////////////////////////////////  
    // The KDTree algorithm & object
    //////////////////////////////////
    template <typename dtype>
    class KDTree {

    };
}