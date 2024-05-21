#pragma once

#include <omp.h>

#include <boost/container/vector.hpp>
#include <boost/range/algorithm/sort.hpp>
#include <boost/algorithm/cxx11/any_of.hpp>

#include <fmt/format.h>
using fmt::format;

#include "geometry.h"

namespace fdml {

    template <int dim, typename dtype>
    struct KDTreeNode {
        Point<dim, dtype> point;
        uint8_t cutDim;
        dtype cutValue;

        KDTreeNode* left = nullptr;
        KDTreeNode* right = nullptr;
    };

    //////////////////////////////////  
    // The KDTree algorithm & object
    //////////////////////////////////
    template <int dim, typename dtype>
    class KDTree {
    public:
        void fit(boost::container::vector<Point<dim, dtype>>& points) {
            _points = points;
            _nodes = boost::container::vector<KDTreeNode<dim, dtype>>(_points.size() * 2);
            _tmpSize = 0;
            root = fitRecurse(0, points.size(), 0);
        }

        Point<dim, dtype> nearestNeighbor(Point<dim, dtype> q) {
            Point<dim, dtype> p;
            dtype w = -1;
            nearestNeighborRecurse(q, root, p, w);
            return p;
        }

        boost::container::vector<Point<dim, dtype>> nearestNeighbor_omp(boost::container::vector<Point<dim, dtype>> qs) {
            boost::container::vector<Point<dim, dtype>> neighbors(qs.size());

            #pragma omp parallel for
            for (int32_t idx = 0; idx < qs.size(); ++idx) {
                Point<dim, dtype> neighbor = this->nearestNeighbor(qs[idx]);
                neighbors[idx] = neighbor;
            }
            return neighbors;
        }

        std::string str() const {
            std::string res;
            strRecurse(root, res, 0);
            return res;
        }

    private:
    public:
        boost::container::vector<Point<dim, dtype>> _points;
        boost::container::vector<KDTreeNode<dim, dtype>> _nodes;
        KDTreeNode<dim, dtype>* root = nullptr;
        uint32_t _tmpSize;

        KDTreeNode<dim, dtype>* fitRecurse(int32_t left, int32_t right, int32_t depth) {
            if (right <= left) return nullptr;
            if (left >= _points.size()) return nullptr;

            uint8_t cutDim = (uint8_t)depth % dim;
            sortSubset(cutDim, left, right);

            int32_t middle = left + (right - left) / 2;
            dtype median = _points[middle][cutDim];
            KDTreeNode<dim, dtype>* node = &_nodes[_tmpSize++];
            node->point = _points[middle];
            node->cutDim = cutDim;
            node->cutValue = median;

            node->left = fitRecurse(left, middle, depth + 1);
            node->right = fitRecurse(middle + 1, right, depth + 1);

            return node;
        }

        void strRecurse(KDTreeNode<dim, dtype>* node, std::string& res, int32_t depth) const {
            if (!node) return;
            for (int32_t i = 0; i < depth; ++i) res += "\t";
            res += format("cut@{} val={} {}\n", node->cutDim, node->cutValue, node->point.str());
            strRecurse(node->left, res, depth + 1);
            strRecurse(node->right, res, depth + 1);
        }

        void nearestNeighborRecurse(Point<dim, dtype> q, KDTreeNode<dim, dtype>* node, Point<dim, dtype>& p, dtype& w) {
            if (!node) return;
            
            dtype w_ = std::sqrt((q - node->point).squaredNorm());
            if (w < 0 || w_ < w) {
                w = w_;
                p = node->point;
            }

            bool searchLeft = q[node->cutDim] <= node->cutValue;
            dtype diff = abs(q[node->cutDim] - node->cutValue);
            if (searchLeft) {
                nearestNeighborRecurse(q, node->left, p, w);
                if (diff < w) nearestNeighborRecurse(q, node->right, p, w);
            } else {
                nearestNeighborRecurse(q, node->right, p, w);
                if (diff < w) nearestNeighborRecurse(q, node->left, p, w);
            }
        }

        // ------------------------------------------------------------------------

        // Sort subset from left idx (inclusive) up to right (exclusive)
        void sortSubset(uint8_t cutDim, int32_t left, int32_t right) {
            std::sort(
                _points.begin() + left, _points.begin() + right,
                [cutDim](const Point<dim, dtype>& a, const Point<dim, dtype>& b) {
                    return a.data[cutDim] < b.data[cutDim];
                }
            );
        }
    };
}