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

constexpr float DEFAULT_PC_SIZE = 3.f;
constexpr float DEFAULT_PC_OPACITY = 0.2f;

class DemoGUI : public LE3SimpleDemo {
public:   
    void init();
    void renderDebug();
    void update(float deltaTime);
    void updateGUI();
    
protected:

    ////////////////////////////////////////////////

    ////////////////////////
    // Environment panel
    ////////////////////////
    
    // Params
    std::vector<std::string> availableEnvs;
    std::vector<char> availableEnvsStr;
    std::string selectedEnv = "";
    le3::LE3PointCloudPtr pointCloud;

    // Methods
    void panelEnvironment();
    void initAvailableEnvs();
    std::string envDisplayName(std::string path);
    std::string envMeshName(std::string path);
    void loadEnvironment(std::string path);

    ////////////////////////////////////////////////

    ////////////////////////
    // Drone control panel
    ////////////////////////

    // Params
    enum class DroneControlMode { OFFLINE, ONLINE };
    DroneControlMode droneControlMode = DroneControlMode::OFFLINE;

    enum class DroneSensorType { CRAFZYFLIE, ELABORATE_CROWN };
    DroneSensorType droneSensorType = DroneSensorType::CRAFZYFLIE;

    // Methods
    void panelDrone();
    void initDrone();
    void updateDrone();
    void setupDroneSensor();

    ////////////////////////////////////////////////

    ////////////////////////
    // FDML parameters panel
    ////////////////////////

    // Params
    fdml::ExperimentEnv env;
    fdml::VoxelCloud localization;
    FT errorBound = 0.015;
    bool cluster = true;
    std::vector<fdml::R3xS1> trajectoryGroundTruth, trajectoryPredicted;
    int badLocalizations = 0;

    // Methods
    void panelFDMLParams();
    void runLocalization();

    ////////////////////////////////////////////////

    ////////////////////////
    // Online operation panel
    ////////////////////////

    // Params

    // Methods
    void panelOnline();

    ////////////////////////////////////////////////

    ////////////////////////
    // Offline trajectory panel
    ////////////////////////

    // Params
    std::vector<std::string> availableTrajectories;
    std::vector<char> availableTrajectoriesStr;
    std::string selectedTrajectory = "";
    std::vector<std::string> manualDistances;
    std::vector<fdml::R3xS1> groundTruthLocations;
    std::vector<std::vector<double>> measurementSequences;
    int currExpIdx = -1;
    float expIdxFraction = -1.f;
    float speed = 10.0f;
    bool shouldPlay = false;

    // Methods
    void panelOffline();
    void initAvailableTrajectories();
    std::string trajectoryDisplayName(std::string path);
    void loadOfflineData(std::string path);

    ////////////////////////////////////////////////

    ////////////////////////
    // Graphics settings panel
    ////////////////////////

    // Params
    bool showAxes = false;

    // Methods
    void panelGraphics();
    void debugDrawVoxel(fdml::R3xS1_Voxel voxel, glm::vec3 color);
    void debugDrawToFCrown();
    void debugDrawTrajectory(std::vector<fdml::R3xS1> trajectory, glm::vec3 color, bool smooth, int smoothWindow=10);

    ////////////////////////////////////////////////    
};
