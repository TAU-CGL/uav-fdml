#include <se3loc/se3loc.h>

#include "experiment_options.h"


BEGIN_EXPERIMENT("KDTree construction time experiment")

    ADD_OPTION(int, num_points, 4096, "number of points to construct kdtree");

    PARSE_ARGS();

    START_RUN()
        se3loc::Random::seed(-1);
        boost::container::vector<se3loc::Point<3, double>> points;
        for (int32_t i = 0; i < num_points; ++i) points.push_back(se3loc::Point<3, double>::uniform());

        se3loc::KDTree<3, double> kdtree;
        kdtree.fit(points);
    END_RUN()
    
END_EXPERIMENT()