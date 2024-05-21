#include <fdml/fdml.h>

#include "experiment_options.h"


BEGIN_EXPERIMENT("KDTree construction time experiment")

    ADD_OPTION(int, num_points, 4096, "number of points to construct kdtree");

    PARSE_ARGS();

    START_RUN()
        fdml::Random::seed(-1);
        boost::container::vector<fdml::Point<3, double>> points;
        for (int32_t i = 0; i < num_points; ++i) points.push_back(fdml::Point<3, double>::uniform());

        fdml::KDTree<3, double> kdtree;
        kdtree.fit(points);
    END_RUN()
    
END_EXPERIMENT()