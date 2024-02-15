#pragma once

#include <boost/container/vector.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>

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

        KDTreeNode* left = nullptr;
        KDTreeNode* right = nullptr;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Store the points and also sort only once.
    // The array `sorted` is of size dim * points.size(), each points.size() are points sorder by some dimension.
    // Each item in sorted points so some point in `points`.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <int dim, typename dtype>
    struct KDTreePoints {
        boost::container::vector<Point<dim, dtype>> points;
        boost::container::vector<uint32_t> sorted;
        
        void init(boost::container::vector<Point<dim, dtype>> points) {
            this->points = points;
            this->sorted = boost::container::vector<uint32_t>(dim * this->points.size());

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

        dtype getMedianValue(uint8_t cutDim, boost::container::vector<uint32_t> indices) {
            int cnt = 0;
            for (uint32_t i = 0; i < points.size(); ++i) {
                uint32_t idx = sorted[cutDim * points.size() + i];
                if (!boost::algorithm::any_of_equal(indices, idx)) continue;
                if (cnt++ == indices.size() / 2) return points[idx][cutDim];
            }
            return 0; // Should not get here
        }
    };

    //////////////////////////////////  
    // The KDTree algorithm & object
    //////////////////////////////////
    template <int dim, typename dtype>
    class KDTree {
    public:
        void fit(boost::container::vector<Point<dim, dtype>>& points) {
            _points.init(points);
            boost::container::vector<uint32_t> indices;
            for (uint32_t idx = 0; idx < points.size(); idx++) indices.push_back(idx);
            fitRecurse(indices, 0);
        }

    private:
        KDTreePoints<dim, dtype> _points;
        boost::container::vector<KDTreeNode<dim, dtype>> _nodes;
        KDTreeNode<dim, dtype>* root = nullptr;

        KDTreeNode<dim, dtype>* fitRecurse(boost::container::vector<uint32_t> indices, uint32_t depth) {
            if (!indices.size()) return nullptr;

            uint8_t cutDim = (uint8_t)depth % dim;
            dtype median = _points.getMedianValue(cutDim, indices);

            _nodes.push_back(KDTreeNode<dim, dtype>());
            KDTreeNode<dim, dtype>* node = &_nodes[_nodes.size() - 1];
            node->cutDim = cutDim;
            node->cutValue = median;
            
            boost::container::vector<uint32_t> indicesLower, indicesGreater;
            int lowerCnt = 0, greaterCnt = 0;
            indicesLower.resize(indices.size() / 2 + 1);
            indicesGreater.resize(indices.size() / 2 + 1);
            for (auto idx : indices) {
                if (_points.points[idx][cutDim] == median) node->points.push_back(_points.points[idx]);
                else if (_points.points[idx][cutDim] < median) indicesLower[lowerCnt++] = idx;
                else indicesGreater[greaterCnt++] = idx;
            }
            indicesLower.resize(lowerCnt);
            indicesGreater.resize(greaterCnt);

            node->left = fitRecurse(indicesLower, depth + 1);
            node->right = fitRecurse(indicesGreater, depth + 1);
            return node;
            // return nullptr;
        }
    };
}