#pragma once


#include <chrono>
#include <fmt/core.h>
using fmt::format, fmt::print;

#define GLM_ENABLE_EXPERIMENTAL 1
#include <le3.h>
using namespace le3;

#include "demo.h"
#include <fdml/fdml.h>
#include <fdml/fdml_utils.h>

class DemoGUI : public LE3SimpleDemo {
public:   
    void init();
    void renderDebug();
    void update(float deltaTime);

    void buildAABBTree();
    void addConfiguration(fdml::R3xS1 q);

protected:
    fdml::ExperimentEnv env;
    
    std::vector<fdml::R3xS1> configurations;
    int configurationsHead = 0;
    std::list<Triangle> triangles;

    fdml::OdometrySequence odometrySequence;
    fdml::OdometrySequence tildeOdometries;
    fdml::MeasurementSequence measurements;
    std::vector<fdml::R3xS1_Voxel> localization;
    bool badMeasurement = false;


    void runRandomExperiment();
    void debugDrawVoxel(fdml::R3xS1_Voxel voxel, glm::vec3 color);
    void displayRoadmap();
};
