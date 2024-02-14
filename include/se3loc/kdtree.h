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
    // We refer to available point as white (and having value 0) and grey points are appended and having value 1
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <int dim, typename dtype>
    struct KDTreePoints {
        boost::container::vector<Point<dim, dtype>> points;
        boost::container::vector<uint32_t> sorted;
        boost::container::vector<uint8_t> grey; // Since this is a boolean array we can further optimize down the size
        uint32_t greyCnt = 0;
        
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

        // Set the color of a point to grey (instead of white)
        void colorPoint(uint32_t idx) {
            grey[idx] = 1; greyCnt++;
        }

        dtype getMedianValue(uint8_t dimCut) {
            uint32_t cnt = 0, idx = 0;
            while (cnt < (points.size() - greyCnt) / 2) {
                if (!grey[sorted[dimCut * points.size() + idx++]]) cnt++;
            }
            return points[sorted[dimCut * points.size() + idx]][dimCut];
        }
    };

    //////////////////////////////////  
    // The KDTree algorithm & object
    //////////////////////////////////
    template <typename dtype>
    class KDTree {

    };
}