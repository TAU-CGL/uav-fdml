#include <fdml/fdml.h>

#include "experiment_options.h"

#define DIM 3

BEGIN_EXPERIMENT("KDTree Nearest Neighbor vs Naive Nearest Neighbor")

    ADD_OPTION(int, num_points, 100000, "number of points to construct kdtree");
    ADD_OPTION(int, num_queries, 1024, "number of query (random) points");

    PARSE_ARGS();

    fdml::Random::seed(-1);
    boost::container::vector<fdml::Point<DIM, double>> points, queries, results;
    for (int32_t i = 0; i < num_points; ++i) points.push_back(fdml::Point<DIM, double>::uniform());
    for (int32_t i = 0; i < num_queries; ++i) queries.push_back(fdml::Point<DIM, double>::uniform());

    fdml::KDTree<DIM, double> kdtree;
    kdtree.fit(points);

    // Start with naive
    START_RUN()
        for (auto q : queries) {
            double minDist = -1;
            fdml::Point<DIM, double> neighbor;
            for (auto p : points) {
                double dist = (p - q).squaredNorm();
                if (minDist < 0 || dist < minDist) {
                    neighbor = p;
                    minDist = dist;
                }
            }
            results.push_back(neighbor);
        }
    END_RUN() // Prints time for naive ( x num_queries)

    results.clear();
    // Start with KDTree NN
    START_RUN()
        for (auto q : queries) {
            fdml::Point<DIM, double> neighbor = kdtree.nearestNeighbor(q);
            results.push_back(neighbor);
        }
    END_RUN() // Prints time for kdtree nn ( x num_queries)
    
END_EXPERIMENT()