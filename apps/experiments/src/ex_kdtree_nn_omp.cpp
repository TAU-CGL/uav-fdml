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

    // Start with serial NN 
    START_RUN()
        for (auto q : queries) {
            fdml::Point<DIM, double> neighbor = kdtree.nearestNeighbor(q);
            results.push_back(neighbor);
        }
    END_RUN() // Prints time for serial nn ( x num_queries)


    // Start with parallel NN 
    results.clear();
    omp_set_num_threads(omp_get_max_threads());
    START_RUN()
        results = kdtree.nearestNeighbor_omp(queries);
    END_RUN() // Prints time for parallel nn ( x num_queries)
    
END_EXPERIMENT()