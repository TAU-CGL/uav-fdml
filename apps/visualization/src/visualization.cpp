#include "visualization.h"

#ifdef __linux__
#include <GL/glew.h>
#else
#include <gl/glew.h>
#endif

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "misc/cpp/imgui_stdlib.h"

void DemoGUI::init() {
    LE3SimpleDemo::init();

    LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");
    LE3GetSceneManager().getActiveScene()->load("/fdml/demo_scene.lua");
    LE3GetSceneManager().getActiveScene()->getMainCamera()->getTransform().setPosition(glm::vec3(0.f, 0.05f, 0.f));
    LE3GetSceneManager().getActiveScene()->getMainCamera()->setPitchYaw(0.f, -1.57f);
    LE3GetActiveScene()->setCulling(true);

    /// -------------------------------

    initDrone();
    initAvailableEnvs();
    initAvailableTrajectories();
    loadEnvironment(availableEnvs[0]);

    // Load json measurements
    


    fflush(stdout);
}

void DemoGUI::runLocalization() {
    std::chrono::steady_clock::time_point begin, end;    
    std::chrono::duration<double, std::milli> __duration;
    begin = std::chrono::steady_clock::now();

    fdml::OdometrySequence odometrySequence = env.getToFCrown();
    fdml::MeasurementSequence measurementSequence;

    // Split manualDistance by ',' and cast to double
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    std::string manualDistance = manualDistances[currExpIdx];
    while ((pos = manualDistance.find(delimiter)) != std::string::npos) {
        token = manualDistance.substr(0, pos);
        measurementSequence.push_back(std::stod(token));
        manualDistance.erase(0, pos + delimiter.length());
    }
    measurementSequence.push_back(std::stod(manualDistance));

    for (double d : measurementSequence) {
        fmt::print("{},", d);
    }

    localization = fdml::localize(env.getTree(), odometrySequence, measurementSequence, env.getBoundingBox(), 10, errorBound, cluster);

    end = std::chrono::steady_clock::now();
    __duration = end - begin;
    print("FDML method: {} [sec]\n", __duration.count() / 1000.0f);


    if (localization.size() == 0) {
        fmt::print("No localization found\n");
        return;
    }
    for (fdml::R3xS1_Voxel v : localization) {
        fmt::print("{} {} {} {}\n", 
            v.bottomLeftPosition.x(), v.bottomLeftPosition.y(), v.bottomLeftPosition.z(), v.bottomLeftRotation
        );
    }

    fdml::R3xS1 nearestLocation = env.bestPrediction(localization);

    if (trajectoryPredicted.size() == 0)
        trajectoryPredicted.push_back(groundTruthLocations[0]);
    fdml::R3xS1 lastLocation = trajectoryPredicted[trajectoryPredicted.size()-1];

    double delta = fdml::R3xS1::deltaPosition(lastLocation, nearestLocation);
    if (delta > 0.2) {
        trajectoryPredicted.push_back(lastLocation);
        badLocalizations++;
    }
    else {
        trajectoryPredicted.push_back(nearestLocation);
        badLocalizations = 0;
    }

    if (badLocalizations > 10) {
        trajectoryPredicted.push_back(fdml::R3xS1(Point(lastLocation.position.x(), lastLocation.position.y(), 1000), 0));
        trajectoryPredicted.push_back(fdml::R3xS1(Point(groundTruthLocations[currExpIdx].position.x(), groundTruthLocations[currExpIdx].position.y(), 1000), 0));
        trajectoryPredicted.push_back(groundTruthLocations[currExpIdx]);
        badLocalizations = 0;
    }

    

    fdml::R3xS1 actualLocation(groundTruthLocations[currExpIdx].position, groundTruthLocations[currExpIdx].orientation);
    env.setActualDroneLocation(actualLocation);
}

void DemoGUI::update(float deltaTime) {
    
    LE3SimpleDemo::update(deltaTime);
    updateDrone();

    if (shouldPlay && (droneControlMode == DroneControlMode::OFFLINE)) {
        expIdxFraction += deltaTime * speed;
        if (expIdxFraction >= manualDistances.size()) {
            expIdxFraction = 0.f;
        }
        if ((int)expIdxFraction > currExpIdx) {
            currExpIdx = (int)expIdxFraction;
            runLocalization();
        }
    }

    updateGUI();
}

void DemoGUI::updateGUI() {
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(300, LE3GetActiveScene()->getHeight()));
    ImGui::Begin("UAV FDM-Localization Demo", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    
    panelEnvironment();
    panelDrone();
    panelFDMLParams();
    if (droneControlMode == DroneControlMode::OFFLINE)
        panelOffline();
    else if (droneControlMode == DroneControlMode::ONLINE)
        panelOnline();
    panelGraphics();

    ImGui::End();
}


void DemoGUI::panelEnvironment() {
    ImGui::SeparatorText("Environment");
    static int comboCurr = 0;
    ImGui::Combo("Env", &comboCurr, &availableEnvsStr[0], availableEnvs.size());
    if (ImGui::Button("Load")) {
        loadEnvironment(availableEnvs[comboCurr]);
    }
}
void DemoGUI::panelDrone() {
    ImGui::SeparatorText("Drone");
    static float droneX, droneY, droneZ, droneR;
    droneX = env.getActualDroneLocation().position.x();
    droneY = env.getActualDroneLocation().position.y();
    droneZ = env.getActualDroneLocation().position.z();
    droneR = env.getActualDroneLocation().orientation;
    ImGui::SliderFloat("X", &droneX, -10.f, 10.f);
    ImGui::SliderFloat("Y", &droneY, -10.f, 10.f);
    ImGui::SliderFloat("Z", &droneZ, -10.f, 10.f);
    ImGui::SliderFloat("R", &droneR, 0.f, 2.f * M_PI);
    if (ImGui::Button("Random")) {
        droneX = (float)rand() / RAND_MAX * 3.f - 1.5f;
        droneY = (float)rand() / RAND_MAX * 3.f - 1.5f;
        droneZ = (float)rand() / RAND_MAX * 3.f - 1.5f;
        droneR = (float)rand() / RAND_MAX * 2.f * M_PI;
    }
    env.setActualDroneLocation(fdml::R3xS1(Point(droneX, droneY, droneZ), droneR));

    static int mode = 0;
    ImGui::RadioButton("Offline", &mode, 0); ImGui::SameLine();
    ImGui::RadioButton("Online", &mode, 1);
    droneControlMode = (mode == 0) ? DroneControlMode::OFFLINE : DroneControlMode::ONLINE;

    static int sensorType = 0;
    ImGui::RadioButton("Crazyflie", &sensorType, 0); ImGui::SameLine();
    ImGui::RadioButton("Elaborate Crown", &sensorType, 1);
    DroneSensorType dst = (sensorType == 0) ? DroneSensorType::CRAFZYFLIE : DroneSensorType::ELABORATE_CROWN;
    if (dst != droneSensorType) {
        droneSensorType = dst;
        setupDroneSensor();
    }
}
void DemoGUI::panelFDMLParams() {
    ImGui::SeparatorText("Experiment");
    static float epsilon;
    ImGui::InputDouble("epsilon", &errorBound);
    ImGui::Checkbox("Cluster", &cluster);
}
void DemoGUI::panelOnline() {
    ImGui::SeparatorText("Online Demonstration");
}
void DemoGUI::panelOffline() {
    ImGui::SeparatorText("Offline Trajectory");

    ImGui::BulletText("Trajectory");
    static int comboCurr = 0;
    ImGui::Combo("Trajectory", &comboCurr, &availableTrajectoriesStr[0], availableTrajectories.size());
    if (ImGui::Button("Load Trajectory")) {
        loadOfflineData(availableTrajectories[comboCurr]);
    }

    if (selectedTrajectory != "") {
        ImGui::BulletText("Playback");

        if (ImGui::Button("<<") && !shouldPlay) {
            currExpIdx = (currExpIdx - 10 + manualDistances.size()) % manualDistances.size();
            runLocalization();
        }
        ImGui::SameLine();
        if (ImGui::Button("<") && !shouldPlay) {
            currExpIdx = (currExpIdx - 1 + manualDistances.size()) % manualDistances.size();
            runLocalization();
        }
        ImGui::SameLine();
        ImGui::Text("%d", currExpIdx);
        ImGui::SameLine();
        if (ImGui::Button(">") && !shouldPlay) {
            currExpIdx = (currExpIdx + 1) % manualDistances.size();
            runLocalization();
        }
        ImGui::SameLine();
        if (ImGui::Button(">>") && !shouldPlay) {
            currExpIdx = (currExpIdx + 10) % manualDistances.size();
            runLocalization();
        }
        
        if (ImGui::Button("Play")) {
            shouldPlay = true;
            expIdxFraction = (float)currExpIdx;
        }
        ImGui::SameLine();
        if (ImGui::Button("Pause")) {
            shouldPlay = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            currExpIdx = 0;
            runLocalization();
        }

        ImGui::SliderFloat("Speed", &speed, 1.f, 25.f);
    }
}
void DemoGUI::panelGraphics() {
    ImGui::SeparatorText("Graphics Settings");
    static bool shouldCull = true;
    if (ImGui::Checkbox("Face Culling", &shouldCull))
        LE3GetActiveScene()->setCulling(shouldCull);
    
    if (pointCloud) {
        float pointSize = pointCloud->getPointSize();
        ImGui::SliderFloat("Point Size", &pointSize, 1.f, 30.f);
        pointCloud->setPointSize(pointSize);

        float opacity = pointCloud->getOpacity();
        ImGui::SliderFloat("Opacity", &opacity, 0.f, 1.f);
        pointCloud->setOpacity(opacity);
    }
}




void DemoGUI::loadEnvironment(std::string path) {
    if (auto obj = LE3GetActiveScene()->getObject<LE3DrawableObject>(envMeshName(selectedEnv))) obj->setHidden(true);
    selectedEnv = path;
    selectedTrajectory = "";

    std::string name = envMeshName(selectedEnv);
    if (auto obj = LE3GetActiveScene()->getObject<LE3DrawableObject>(name)) obj->setHidden(false);
    else {
        if (path.ends_with(".obj")) {
            LE3GetAssetManager().addStaticMesh(name, selectedEnv, true);
            LE3GetActiveScene()->addStaticModel(name, name, "M_room");
            FDML_LE3_LoadEnvironment(LE3GetAssetManager(), name, env);
            pointCloud = nullptr;
        }
        else if (path.ends_with(".ply")) {
            LE3GetActiveScene()->addPointCloud(name);
            LE3GetActiveScene()->getObject<LE3PointCloud>(name)->fromFile(selectedEnv, true);
            LE3GetActiveScene()->getObject<LE3PointCloud>(name)->create();
            FDML_LE3_LoadEnvironmentPointCloud(LE3GetActiveScene()->getObject<LE3PointCloud>(name), env, 0.1f);
            pointCloud = LE3GetActiveScene()->getObject<LE3PointCloud>(name);
            pointCloud->setPointSize(DEFAULT_PC_SIZE);
            pointCloud->setOpacity(DEFAULT_PC_OPACITY);
        }
    }

}

void DemoGUI::initAvailableEnvs() {
    for (auto filename : LE3GetDatFileSystem().getFilesFromDir("/fdml/scans")) {
        if (filename.ends_with(".ply")) {
            availableEnvs.push_back(filename);
            for (auto c : envDisplayName(filename))
                availableEnvsStr.push_back(c);
            availableEnvsStr.push_back('\0');
        }
    }
}
void DemoGUI::initAvailableTrajectories() {
    for (auto filename : LE3GetDatFileSystem().getFilesFromDir("/fdml/trajectories")) {
        if (filename.ends_with(".json")) {
            availableTrajectories.push_back(filename);
            for (auto c : trajectoryDisplayName(filename))
                availableTrajectoriesStr.push_back(c);
            availableTrajectoriesStr.push_back('\0');
        }
    }
}
std::string DemoGUI::envDisplayName(std::string path) {
    //Split by '/' and remove ".ply"
    std::string res;
    std::stringstream ss(path);
    while (std::getline(ss, res, '/'));
    auto idx = res.find(".ply");
    return res.substr(0, idx);
}
std::string DemoGUI::trajectoryDisplayName(std::string path) {
    //Split by '/' and remove ".json"
    std::string res;
    std::stringstream ss(path);
    while (std::getline(ss, res, '/'));
    auto idx = res.find(".json");
    return res.substr(0, idx);
}
std::string DemoGUI::envMeshName(std::string path) {
    return "SM_FDML_" + envDisplayName(path);
}

void DemoGUI::loadOfflineData(std::string path) {
    currExpIdx = -1; expIdxFraction = -1.f;
    manualDistances.clear();
    groundTruthLocations.clear();
    measurementSequences.clear();
    selectedTrajectory = path;

    LE3DatBuffer buffer = LE3GetDatFileSystem().getFileContent(selectedTrajectory);
    json j = json::parse(buffer.toString());
    for (auto m : j) {
        if (!m.contains("type") || m["type"] == "crazyflie") {
            double front = m["front"]; double back = m["back"]; double left = m["left"]; double right = m["right"];
            double x = m["x"]; double y = m["y"]; double z = m["z"]; double yaw = -(double)m["yaw"] + M_PI;

            std::vector<double> ds;
            ds.push_back(front); ds.push_back(back); ds.push_back(right); ds.push_back(left); ds.push_back(-1); ds.push_back(z);
            measurementSequences.push_back(ds);
            manualDistances.push_back(fmt::format("{},{},{},{},-1,{}", front, back, right,left, z));
            groundTruthLocations.push_back(fdml::R3xS1(Point(x, -y, z), yaw));
        }
    }

    trajectoryGroundTruth = groundTruthLocations;
}


// ------------------------------------------------


void DemoGUI::renderDebug() {
    glClear(GL_DEPTH_BUFFER_BIT);
    debugDrawToFCrown();
    for (fdml::R3xS1_Voxel v : localization) {
        debugDrawVoxel(v, glm::vec3(0.f, 1.f, 1.f));
    }
    if (showAxes) {
        LE3GetVisualDebug().drawDebugLine(glm::vec3(0.f), glm::vec3(1.f, 0.f, 0.f), glm::vec3(1.f, 0.f, 0.f));
        LE3GetVisualDebug().drawDebugLine(glm::vec3(0.f), glm::vec3(0.f, 1.f, 0.f), glm::vec3(0.f, 1.f, 0.f));
        LE3GetVisualDebug().drawDebugLine(glm::vec3(0.f), glm::vec3(0.f, 0.f, 1.f), glm::vec3(0.f, 0.f, 1.f));
    }

    debugDrawTrajectory(trajectoryGroundTruth, glm::vec3(0.f, 1.f, 0.f), false);
    debugDrawTrajectory(trajectoryPredicted, glm::vec3(0.f, 1.f, 1.f), true);
}

void DemoGUI::debugDrawToFCrown() {
    if (droneControlMode == DroneControlMode::OFFLINE && currExpIdx < 0) return;
    if (droneControlMode == DroneControlMode::ONLINE) return;

    fdml::OdometrySequence tofCrown = env.getToFCrown();
    int idx = 0;
    for (fdml::R3xS2 g : tofCrown) {
        fdml::R3xS2 g_ = env.getActualDroneLocation() * g;
        fdml::R3xS1_Voxel v;
        FT size = 0.01;
        v.bottomLeftPosition = Point(g_.position.x() - size, g_.position.y() - size, g_.position.z() - size);
        v.topRightPosition = Point(g_.position.x() + size, g_.position.y() + size, g_.position.z() + size);
        debugDrawVoxel(v, glm::vec3(1.f, 0.f, 1.f));

        // FT dist = g_.measureDistance(env.getTree());
        FT dist = measurementSequences[currExpIdx][idx++];
        if (dist < 0) continue;
        glm::vec3 pos(g_.position.x(), g_.position.z(), g_.position.y()); // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
        glm::vec3 color(1.f, 0.f, 1.f); if (dist > 4.0) color = glm::vec3(1.f, 0.f, 0.f);
        LE3GetVisualDebug().drawDebugLine(
            pos, pos + ((float)dist) * glm::vec3(g_.direction.x(), g_.direction.z(), g_.direction.y()),
            color
        );
        LE3GetVisualDebug().drawDebugBox(
            pos + ((float)dist) * glm::vec3(g_.direction.x(), g_.direction.z(), g_.direction.y()),
            glm::quat(), glm::vec3(0.05f, 0.05f, 0.05f), color);
    }
}

void DemoGUI::initDrone() {
    LE3GetSceneManager().getActiveScene()->addStaticModel("__drone", "SM_drone", "M_drone");
    LE3GetSceneManager().getActiveScene()->getObject("__drone")->getTransform().setScale(glm::vec3(2.f));
    setupDroneSensor();
}
void DemoGUI::setupDroneSensor() {
    if (droneSensorType == DroneSensorType::CRAFZYFLIE)
        env.createCrazyFlieCrown();
    else if (droneSensorType == DroneSensorType::ELABORATE_CROWN)
        env.createToFCrown(16, 0, 0.07687 + 0.01017, Point(0.5 * sqrt(2), 0, -0.5 * sqrt(2)));
}
void DemoGUI::updateDrone() {
    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    fdml::R3xS1 q = env.getActualDroneLocation();
    LE3GetSceneManager().getActiveScene()->getObject("__drone")->getTransform().setRotationRPY(0.f, 0.f, -q.orientation);
    LE3GetSceneManager().getActiveScene()->getObject("__drone")->getTransform().setPosition(glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.z()), CGAL::to_double(q.position.y())));
}

void DemoGUI::debugDrawVoxel(fdml::R3xS1_Voxel voxel, glm::vec3 color) {
    glm::vec3 bottomLeft = glm::vec3(voxel.bottomLeftPosition.x(), voxel.bottomLeftPosition.z(), voxel.bottomLeftPosition.y());
    glm::vec3 topRight = glm::vec3(voxel.topRightPosition.x(), voxel.topRightPosition.z(), voxel.topRightPosition.y());
    LE3GetVisualDebug().drawDebugBox(bottomLeft + 0.5f * (topRight - bottomLeft), glm::quat(), topRight - bottomLeft, color);
}

void DemoGUI::debugDrawTrajectory(std::vector<fdml::R3xS1> trajectory, glm::vec3 color, bool smooth, int smoothWindow) {
    if (trajectory.size() < 2) return;

    if (smooth) {
        std::vector<fdml::R3xS1> smoothedTrajectory;
        for (int i = 0; i < trajectory.size(); i++) {
            fdml::R3xS1 p = trajectory[i];
            int start = std::max(0, i - smoothWindow);
            int end = std::min((int)trajectory.size() - 1, i + smoothWindow);
            for (int j = start; j <= end; j++) {
                p.position = Point(p.position.x() + trajectory[j].position.x(), p.position.y() + trajectory[j].position.y(), p.position.z() + trajectory[j].position.z());
                p.orientation = p.orientation + trajectory[j].orientation;
            }
            p.position = Point(p.position.x() / (end - start + 1), p.position.y() / (end - start + 1), p.position.z() / (end - start + 1));
            p.orientation = p.orientation / (end - start + 1);
            smoothedTrajectory.push_back(p);
        }
        trajectory = smoothedTrajectory;
    }

    for (int i = 0; i < (trajectory.size() - 1); i++) {
        glm::vec3 p1 = glm::vec3(trajectory[i].position.x(), trajectory[i].position.z(), trajectory[i].position.y());
        glm::vec3 p2 = glm::vec3(trajectory[i + 1].position.x(), trajectory[i + 1].position.z(), trajectory[i + 1].position.y());
        glm::vec3 tmpColor = color;
        if (p1.y > 20.f || p2.y > 20.f) {
            tmpColor = glm::vec3(1.f, 0.f, 0.f);
            continue;
        }
        LE3GetVisualDebug().drawDebugLine(p1, p2, tmpColor);
    }
}