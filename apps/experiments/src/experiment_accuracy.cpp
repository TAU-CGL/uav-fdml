#include "experiment_options.h"
#include <le3.h>
using namespace le3;

#include <fmt/core.h>

#include <fdml/fdml.h>
#include <fdml/fdml_le3.h>
#include <fdml/fdml_utils.h>

const std::string envName = "environment";

BEGIN_EXPERIMENT("Test accuracy (and success rate) of localization in an environment with random valid k")
    ADD_OPTION(int, k, 50, "number of odometries");
    ADD_OPTION(double, delta, 0.1, "maximum box diameter (in 4D) for recursion");
    ADD_OPTION(double, epsilon, 0.02, "distance measurement and odometry error");
    ADD_OPTION(std::string, environment, "/fdml/scans/isprs/cs2.obj", "Path to OBJ file of the scene");
    ADD_OPTION(bool, egt, true, "Enforces good trajectory");
    PARSE_ARGS();

    le3::LE3AssetManager assets;
    fdml::ExperimentEnv env;
    fdml::ExperimentParams params;
    fdml::ExperimentMetrics results;
    params.k = k;
    params.delta = delta;
    params.epsilon = epsilon;
    params.enforceGoodTrajectory = egt;
    
    LE3Application app;
    app.init();
    le3::LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");
    assets.addStaticMesh(envName, environment, true);
    FDML_LE3_LoadEnvironment(assets, envName, env);
    
    START_RUN();
        fmt::print("[iteration {}]\n", __exp_idx);
        fflush(stdout);
        env.runExperiment(params);
        results += env.metrics;
    END_RUN();

    
    fmt::print("numTimeouts: {}\n", results.numTimeouts);
    double numexps = (double)(__num_experiments - results.numTimeouts);
    fmt::print("time: {}\n", results.timeMiliseconds / numexps);
    fmt::print("conservativeSuccess: {}\n", results.conservativeSuccess / numexps);
    fmt::print("errorXYZ: {}\n", results.errorXYZ / numexps);
    fmt::print("errorTheta: {}\n", results.errorTheta / numexps);
    fmt::print("numVoxels: {}\n", results.numVoxels / numexps);
    fmt::print("numClusters: {}\n", results.numClusters / numexps);
    fmt::print("localizationVolume: {}\n", results.localizationVolume / numexps);
    fmt::print("localizationVolumePercentage: {}\n", results.localizationVolumePercentage / numexps);
    fmt::print("localizationVolumeXYT: {}\n", results.localizationVolumeXYT / numexps);
    fmt::print("localizationVolumePercentageXYT: {}\n", results.localizationVolumePercentageXYT / numexps);


END_EXPERIMENT()