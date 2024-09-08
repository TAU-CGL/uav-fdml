#include "experiment_options.h"
#include <le3.h>
using namespace le3;

#include <fdml/fdml.h>
#include <fdml/fdml_le3.h>
#include <fdml/fdml_utils.h>

fdml::R3xS1 SampleValidOdometry(AABBTree& tree, fdml::R3xS1 q, fdml::R3xS1_Voxel boundingBox);

const std::string envName = "environment";

BEGIN_EXPERIMENT("Test accuracy (and success rate) of localization in an environment with random valid k")
    ADD_OPTION(int, k, 10, "number of odometries");
    ADD_OPTION(int, depth, 10, "localization recursion depth");
    ADD_OPTION(std::string, environment, "/fdml/scans/240521-141038/240521-141038-scaled.obj", "Path to OBJ file of the scene");
    PARSE_ARGS();

    le3::LE3AssetManager assets;
    fdml::ExperimentEnv env;
    
    LE3Application app(std::make_unique<LE3GameLogic>());
    app.init();
    
    le3::LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");
    assets.addStaticMesh(envName, environment, true);
    FDML_LE3_LoadEnvironment(assets, envName, env);
    
    double accuracy = 0.f;
    int success = 0;

    START_RUN();
        fdml::R3xS1 q0(Point(0, 0, 0), 0), q;
        q0 = SampleValidOdometry(env.getTree(), q0, env.getBoundingBox());
        q = q0;

        fdml::OdometrySequence odometrySequence;
        odometrySequence.push_back(fdml::R3xS1(Point(0, 0, 0), 0));
        for (int i = 0; i < k - 1; i++) {
            fdml::R3xS1 g_i = SampleValidOdometry(env.getTree(), q, env.getBoundingBox());
            q = g_i * q;
            odometrySequence.push_back(g_i);
        }

        fdml::OdometrySequence groundTruths = fdml::getGroundTruths(odometrySequence, q0);
        fdml::MeasurementSequence measurements = fdml::getMeasurementSequence(env.getTree(), groundTruths);
        fdml::VoxelCloud localization = fdml::localize(env.getTree(), odometrySequence, measurements, env.getBoundingBox(), depth);

        for (auto v : localization) {
            if (v.contains(q0)) {
                success++; break;
            }
        }
    
    END_RUN();

    std::cout << "100" << std::endl;
    std::cout << 100.0 * (double)success / (double)__num_experiments << std::endl;
END_EXPERIMENT()

fdml::R3xS1 SampleValidOdometry(AABBTree& tree, fdml::R3xS1 q, fdml::R3xS1_Voxel boundingBox) {
    while (true) {
        FT x = boundingBox.bottomLeftPosition.x() + fdml::Random::randomDouble() * (boundingBox.topRightPosition.x() - boundingBox.bottomLeftPosition.x());
        FT y = boundingBox.bottomLeftPosition.y() + fdml::Random::randomDouble() * (boundingBox.topRightPosition.y() - boundingBox.bottomLeftPosition.y());
        FT z = boundingBox.bottomLeftPosition.z() + fdml::Random::randomDouble() * (boundingBox.topRightPosition.z() - boundingBox.bottomLeftPosition.z());
        FT r = fdml::Random::randomDouble() * 2 * M_PI;
        fdml::R3xS1 g(Point(x, y, z), r);
        fdml::R3xS1 q_ = g * q;
        if (q_.measureDistance(tree) < 0) continue;
        return g;
    }
}