#pragma once


#include <chrono>
#include <fmt/core.h>
using fmt::format, fmt::print;

#define GLM_ENABLE_EXPERIMENTAL 1
#include <le3.h>
using namespace le3;

#include <fdml/fdml.h>
#include <fdml/fdml_le3.h>
#include <fdml/fdml_utils.h>

class DemoGUI : public LE3SimpleDemo {
public:   
    void init();
    void renderDebug();
    void update(float deltaTime);
    
protected:
    std::vector<std::string> availableEnvs;
    std::vector<char> availableEnvsStr;
    std::string selectedEnv = "";

    le3::LE3PointCloudPtr pointCloud;

    fdml::ExperimentEnv env;
    fdml::VoxelCloud localization;
    FT errorBound = 0.015;
    bool cluster = true;

    std::vector<std::string> manualDistances;
    std::vector<fdml::R3xS1> groundTruthLocations;
    std::vector<std::vector<double>> measurementSequences;
    int currExpIdx = 0;
    float expIdxFraction = 0.f;
    float speed = 10.0f;
    bool shouldPlay = false;
    bool showAxes = false;
    
    void loadEnvironment(std::string path);
    void runManualExperiment();
    void debugDrawVoxel(fdml::R3xS1_Voxel voxel, glm::vec3 color);
    void debugDrawToFCrown();

    void initDrone();
    void updateDrone();

    void initAvailableEnvs();
    std::string envDisplayName(std::string path);
    std::string envMeshName(std::string path);
};
