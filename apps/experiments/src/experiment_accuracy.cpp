#include "experiment_options.h"
#include <le3.h>
using namespace le3;

#include <fmt/core.h>

#include <fdml/fdml.h>
#include <fdml/fdml_le3.h>
#include <fdml/fdml_utils.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

const std::string envName = "environment";

BEGIN_EXPERIMENT("Test accuracy (and success rate) of localization in an environment with random valid k")
    ADD_OPTION(int, depth, 10, "Subdivision search recursion depth");
    ADD_OPTION(double, epsilon, 0.015, "Distance measurement error bound");
    ADD_OPTION(std::string, environment, "/fdml/scans/labs/lab363_v3.obj", "Path to OBJ file of the scene");
    ADD_OPTION(std::string, measurements, "/fdml/experiments/measurements.json", "Path to JSON file with measurements");
    PARSE_ARGS();

    le3::LE3AssetManager assets;
    fdml::ExperimentEnv env;
    env.createToFCrown(0, 0, 0, Point(0,0,0));
    // fdml::ExperimentParams params;
    // fdml::ExperimentMetrics results;
    // params.k = k;
    // params.delta = delta;
    // params.epsilon = epsilon;
    // params.enforceGoodTrajectory = egt;
    
    LE3Application app;
    app.init();
    le3::LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");
    assets.addStaticMesh(envName, environment, true);
    FDML_LE3_LoadEnvironment(assets, envName, env);

    LE3DatBuffer buffer = LE3GetDatFileSystem().getFileContent(measurements);
    json j = json::parse(buffer.toString());
    
    std::vector<std::vector<double>> measurementSequences;
    std::vector<fdml::R3xS1> groundTruthLocations;

    for (auto m : j) {
        double front = m["front"]; double back = m["back"]; double left = m["left"]; double right = m["right"];
        double x = m["x"]; double y = m["y"]; double z = m["z"]; double yaw = m["yaw"];
        
        std::vector<double> ds;
        ds.push_back(front); ds.push_back(back); ds.push_back(right); ds.push_back(left); ds.push_back(-1); ds.push_back(z);
        measurementSequences.push_back(ds);
        groundTruthLocations.push_back(fdml::R3xS1(Point(y, x, z), yaw));
    }
    

    std::vector<int> numLocations;
    std::vector<double> nearestDistance;
    std::vector<double> nearestOrientation;
    std::vector<double> durations;
    std::vector<double> epsilons;

    fdml::OdometrySequence odometrySequence = env.getToFCrown();
    fdml::VoxelCloud localization;

    for (int idx = 0; idx < measurementSequences.size(); ++idx) {
        fdml::MeasurementSequence measurementSequence = measurementSequences[idx];
        START_RUN();
        localization = 
            fdml::localize(env.getTree(), odometrySequence, measurementSequence, env.getBoundingBox(), depth, epsilon);
        END_RUN();

        numLocations.push_back(localization.size());
        fdml::R3xS1 q = groundTruthLocations[idx];
        double bestDist = -1, bestOrientation = -1;
        for (auto v : localization) {
            Point middle = v.middle().position;
            double orientation = v.middle().orientation;
            double dist = sqrt(CGAL::squared_distance(q.position, middle));
            if (bestDist < 0 || dist < bestDist) {
                bestDist = dist;
                bestOrientation = orientation;
            }
        }
        nearestDistance.push_back(bestDist);
        nearestOrientation.push_back(bestOrientation);
        epsilons.push_back(epsilon);

        durations.push_back(__duration.count() / 1000.0f);
    }


    json results;

    results["numLocations"] = numLocations;
    results["nearestDistance"] = nearestDistance;
    results["nearestOrientation"] = nearestOrientation;
    results["duration"] = durations;
    results["epsilon"] = epsilons;

    fmt::print("{}\n", results.dump(4));

    // START_RUN();
    //     fmt::print("[iteration {}]\n", __exp_idx);
    //     fflush(stdout);
    //     env.runExperiment(params);
    //     results += env.metrics;
    // END_RUN();

    
    // fmt::print("numTimeouts: {}\n", results.numTimeouts);
    // double numexps = (double)(__num_experiments - results.numTimeouts);
    // fmt::print("time: {}\n", results.timeMiliseconds / numexps);
    // fmt::print("conservativeSuccess: {}\n", results.conservativeSuccess / numexps);
    // fmt::print("errorXYZ: {}\n", results.errorXYZ / numexps);
    // fmt::print("errorTheta: {}\n", results.errorTheta / numexps);
    // fmt::print("numVoxels: {}\n", results.numVoxels / numexps);
    // fmt::print("numClusters: {}\n", results.numClusters / numexps);
    // fmt::print("localizationVolume: {}\n", results.localizationVolume / numexps);
    // fmt::print("localizationVolumePercentage: {}\n", results.localizationVolumePercentage / numexps);
    // fmt::print("localizationVolumeXYT: {}\n", results.localizationVolumeXYT / numexps);
    // fmt::print("localizationVolumePercentageXYT: {}\n", results.localizationVolumePercentageXYT / numexps);


END_EXPERIMENT()