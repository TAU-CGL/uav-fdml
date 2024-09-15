#pragma once


#include <chrono>
#include <fmt/core.h>
using fmt::format, fmt::print;

#define GLM_ENABLE_EXPERIMENTAL 1
#include <le3.h>
using namespace le3;

#include "demo.h"
#include <fdml/fdml.h>
#include <fdml/fdml_le3.h>
#include <fdml/fdml_utils.h>

class DemoGUI : public LE3SimpleDemo {
public:   
    void init();
    void renderDebug();
    void update(float deltaTime);

    void addConfigurationMarker(fdml::R3xS1 q);

protected:
    fdml::ExperimentEnv env;
    fdml::ExperimentParams params;
    std::vector<std::string> availableEnvs;
    std::vector<char> availableEnvsStr;
    std::string selectedEnv = "";
    std::string predeterminedPath = "";
    int numExpansions = 1;

    std::vector<fdml::R3xS1> configurations;
    int configurationsHead = 0;
    
    void loadEnvironment(std::string path);
    void runRandomExperiment();
    void debugDrawVoxel(fdml::R3xS1_Voxel voxel, glm::vec3 color);
    void displayRoadmap();

    void initGizmo();
    void updateGizmo();

    void initAvailableEnvs();
    std::string envDisplayName(std::string path);
    std::string envMeshName(std::string path);
    void filterVoxelsNearBoundary(); // We can assume that the UAV has a reasonable clearance from boundary of the environment
};
