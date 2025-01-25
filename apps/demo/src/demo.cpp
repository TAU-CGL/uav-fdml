#include "demo.h"

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
    initGizmo();

    /// -------------------------------

    initDrone();
    initAvailableEnvs();
    // loadEnvironment("/fdml/scans/labs/lab446a.ply");
    loadEnvironment("/fdml/scans/labs/lab363.ply");

    // Load json measurements
    // LE3DatBuffer buffer = LE3GetDatFileSystem().getFileContent("/fdml/experiments/exp_mr_lh_446a.json");
    LE3DatBuffer buffer = LE3GetDatFileSystem().getFileContent("/fdml/experiments/exp_mr_lh_363.json");
    json j = json::parse(buffer.toString());
    for (auto m : j) {
        double front = m["front"]; double back = m["back"]; double left = m["left"]; double right = m["right"];
        double x = m["x"]; double y = m["y"]; double z = m["z"]; double yaw = 0;//-(double)m["yaw"] + M_PI;

        std::vector<double> ds;
        ds.push_back(front); ds.push_back(back); ds.push_back(right); ds.push_back(left); ds.push_back(-1); ds.push_back(z);
        measurementSequences.push_back(ds);
        manualDistances.push_back(fmt::format("{},{},{},{},-1,{}", front, back, right,left, z));
        groundTruthLocations.push_back(fdml::R3xS1(Point(x, -y, z), yaw));
    }

    env.setActualDroneLocation(groundTruthLocations[currExpIdx]);


    fflush(stdout);
}

void DemoGUI::runManualExperiment() {
    std::chrono::steady_clock::time_point begin, end;    
    std::chrono::duration<double, std::milli> __duration;
    begin = std::chrono::steady_clock::now();

    fdml::OdometrySequence odometrySequence = env.getToFCrown();
    fdml::MeasurementSequence measurementSequence;

    env.setActualDroneLocation(groundTruthLocations[currExpIdx]);
    
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

    fdml::R3xS1 nearestLocation = localization[0].middle();
    double bestDist = sqrt(CGAL::squared_distance(groundTruthLocations[currExpIdx].position, nearestLocation.position));
    for (fdml::R3xS1_Voxel v : localization) {
        fmt::print("{} {} {} {}\n", 
            v.bottomLeftPosition.x(), v.bottomLeftPosition.y(), v.bottomLeftPosition.z(), v.bottomLeftRotation
        );
        // if (v.contains(actualQ)) fmt::print("!!!!!!!!!!!\n");

        double dist = sqrt(CGAL::squared_distance(groundTruthLocations[currExpIdx].position, v.middle().position));
        if (dist < bestDist) {
            bestDist = dist;
            nearestLocation = v.middle();
        }
    }

    fdml::R3xS1 actualLocation(groundTruthLocations[currExpIdx].position, nearestLocation.orientation);
    env.setActualDroneLocation(actualLocation);
}

void DemoGUI::update(float deltaTime) {
    
    LE3SimpleDemo::update(deltaTime);
    updateGizmo();
    updateDrone();


    if (shouldPlay) {
        expIdxFraction += deltaTime * speed;
        if (expIdxFraction >= manualDistances.size()) {
            expIdxFraction = 0.f;
        }
        if ((int)expIdxFraction > currExpIdx) {
            currExpIdx = (int)expIdxFraction;
            runManualExperiment();
        }
    }


    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(300, LE3GetActiveScene()->getHeight()));
    ImGui::Begin("UAV FDM-Localization Demo", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    
        ImGui::SeparatorText("Environment");
        static int comboCurr = 0;
        ImGui::Combo("Env", &comboCurr, &availableEnvsStr[0], availableEnvs.size());
        if (ImGui::Button("Load")) {
            loadEnvironment(availableEnvs[comboCurr]);
        }

        // ImGui::SeparatorText("Drone");
        // static float droneX, droneY, droneZ, droneR;
        // ImGui::SliderFloat("X", &droneX, -10.f, 10.f);
        // ImGui::SliderFloat("Y", &droneY, -10.f, 10.f);
        // ImGui::SliderFloat("Z", &droneZ, -10.f, 10.f);
        // ImGui::SliderFloat("R", &droneR, 0.f, 2.f * M_PI);
        // if (ImGui::Button("Random")) {
        // }
        // env.setActualDroneLocation(fdml::R3xS1(Point(droneX, droneY, droneZ), droneR));

        if (selectedEnv != "") {
            ImGui::SeparatorText("Experiment");

            static float delta, epsilon;
            // ImGui::SliderInt("k", &params.k, 4, 50);
            // ImGui::InputDouble("delta", &params.delta);
            ImGui::InputDouble("epsilon", &errorBound);

            ImGui::SeparatorText("Manual Experiments");
            if (ImGui::Button("<<") && !shouldPlay) {
                currExpIdx = (currExpIdx - 10 + manualDistances.size()) % manualDistances.size();
                runManualExperiment();
            }
            ImGui::SameLine();
            if (ImGui::Button("<") && !shouldPlay) {
                currExpIdx = (currExpIdx - 1 + manualDistances.size()) % manualDistances.size();
                runManualExperiment();
            }
            ImGui::SameLine();
            ImGui::Text("%d", currExpIdx);
            ImGui::SameLine();
            if (ImGui::Button(">") && !shouldPlay) {
                currExpIdx = (currExpIdx + 1) % manualDistances.size();
                runManualExperiment();
            }
            ImGui::SameLine();
            if (ImGui::Button(">>") && !shouldPlay) {
                currExpIdx = (currExpIdx + 10) % manualDistances.size();
                runManualExperiment();
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
                runManualExperiment();
            }

            ImGui::SliderFloat("Speed", &speed, 1.f, 25.f);

            ImGui::Checkbox("Cluster", &cluster);
        }

        ImGui::SeparatorText("Visualization");
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
        

    ImGui::End();
}

void DemoGUI::loadEnvironment(std::string path) {
    if (auto obj = LE3GetActiveScene()->getObject<LE3DrawableObject>(envMeshName(selectedEnv))) obj->setHidden(true);
    selectedEnv = path;

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
        }
    }

}

void DemoGUI::initAvailableEnvs() {
    for (auto filename : LE3GetDatFileSystem().getFilesFromDir("/fdml/scans")) {
        // fmt::print("Trying: {}\n", filename);
        if (filename.ends_with(".ply")) {
            availableEnvs.push_back(filename);
            for (auto c : envDisplayName(filename))
                availableEnvsStr.push_back(c);
            availableEnvsStr.push_back('\0');
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
std::string DemoGUI::envMeshName(std::string path) {
    return "SM_FDML_" + envDisplayName(path);
}


// ------------------------------------------------


void DemoGUI::renderDebug() {
    // Draw configurations: measure downards (and draw hitpoint)
    int idx = 0;
    // for (auto q : configurations) {
    //     if (idx++ < configurationsHead) continue;
    //     double distance = q.measureDistance(env.getTree());
    //     glm::vec3 pos(q.position.x(), q.position.z(), q.position.y()); // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    //     LE3GetVisualDebug().drawDebugLine(
    //         pos, pos - glm::vec3(0.f, distance, 0.f),
    //         glm::vec3(1.f, 0.f, 1.f)
    //     );
    //     LE3GetVisualDebug().drawDebugBox(
    //         pos - glm::vec3(0.f, distance, 0.f),
    //         glm::quat(), glm::vec3(0.05f, 0.05f, 0.05f), glm::vec3(1.f, 0.f, 1.f));
    // }

    glClear(GL_DEPTH_BUFFER_BIT);

    debugDrawToFCrown();

    for (fdml::R3xS1_Voxel v : localization) {
        debugDrawVoxel(v, glm::vec3(0.f, 1.f, 1.f));
    }

    // LE3GetVisualDebug().drawDebugLine(glm::vec3(0.f), glm::vec3(1.f, 0.f, 0.f), glm::vec3(1.f, 0.f, 0.f));
    // LE3GetVisualDebug().drawDebugLine(glm::vec3(0.f), glm::vec3(0.f, 1.f, 0.f), glm::vec3(0.f, 1.f, 0.f));
    // LE3GetVisualDebug().drawDebugLine(glm::vec3(0.f), glm::vec3(0.f, 0.f, 1.f), glm::vec3(0.f, 0.f, 1.f));

    // Draw bounding box
    // debugDrawVoxel(env.getBoundingBox(), glm::vec3(1.f, 0.f, 0.f));
}

void DemoGUI::debugDrawToFCrown() {
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
    // LE3GetSceneManager().getActiveScene()->addStaticModel("__marker", "SM_cursor", "M_cursor", "", DRAW_PRIORITY_HIGH);
    LE3GetSceneManager().getActiveScene()->addStaticModel("__drone", "SM_drone", "M_drone");

    // LE3GetSceneManager().getActiveScene()->getObject("__marker")->getTransform().setScale(1.75f);
    // LE3GetSceneManager().getActiveScene()->getObject("__drone")->getTransform().setScale(0.05f);

    env.createToFCrown(16, 0, 0.07687 + 0.01017, Point(0.5 * sqrt(2), 0, -0.5 * sqrt(2)));
}
void DemoGUI::updateDrone() {
    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    fdml::R3xS1 q = env.getActualDroneLocation();
    // LE3GetSceneManager().getActiveScene()->getObject("__marker")->getTransform().setRotationRPY(0.f, 0.f, -q.orientation);
    // LE3GetSceneManager().getActiveScene()->getObject("__marker")->getTransform().setPosition(glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.z()), CGAL::to_double(q.position.y())));

    LE3GetSceneManager().getActiveScene()->getObject("__drone")->getTransform().setRotationRPY(0.f, 0.f, -q.orientation);
    LE3GetSceneManager().getActiveScene()->getObject("__drone")->getTransform().setPosition(glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.z()), CGAL::to_double(q.position.y())));
}

void DemoGUI::debugDrawVoxel(fdml::R3xS1_Voxel voxel, glm::vec3 color) {
    glm::vec3 bottomLeft = glm::vec3(voxel.bottomLeftPosition.x(), voxel.bottomLeftPosition.z(), voxel.bottomLeftPosition.y());
    glm::vec3 topRight = glm::vec3(voxel.topRightPosition.x(), voxel.topRightPosition.z(), voxel.topRightPosition.y());
    LE3GetVisualDebug().drawDebugBox(bottomLeft + 0.5f * (topRight - bottomLeft), glm::quat(), topRight - bottomLeft, color);
}

void DemoGUI::initGizmo() {
    // Add gizmo to bottom left (for clatiry)
    LE3GetSceneManager().getActiveScene()->addCustomObject("gizmo", std::make_shared<LE3Gizmo>());
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setHoverable(false);
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setDynamicScale(false);
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setMaterial(LE3GetAssetManager().getMaterial("M_custom_gizmo"));
    LE3GetActiveScene()->getObject("gizmo")->getTransform().setPosition(glm::vec3(0.f, .0f, -5.f));
}
void DemoGUI::updateGizmo() {
    glm::vec3 cameraPos = LE3GetActiveScene()->getMainCamera()->getPosition();
    glm::vec3 cameraForward = LE3GetActiveScene()->getMainCamera()->getForward();
    glm::vec3 cameraRight = LE3GetActiveScene()->getMainCamera()->getRight();
    glm::vec3 cameraUp = LE3GetActiveScene()->getMainCamera()->getUp();
    glm::vec3 gizmoPosition = cameraPos + cameraForward * 0.5f + cameraRight * 0.0f + cameraUp * 0.0f;
}