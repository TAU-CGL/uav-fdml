#include "demo.h"

void DemoGUI::init() {
    LE3SimpleDemo::init();

    LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");
    LE3GetSceneManager().getActiveScene()->load("/fdml/demo_scene.lua");
    LE3GetSceneManager().getActiveScene()->getMainCamera()->getTransform().setPosition(glm::vec3(0.f, 0.05f, 0.f));
    LE3GetSceneManager().getActiveScene()->getMainCamera()->setPitchYaw(0.f, -1.57f);
    LE3GetActiveScene()->setCulling(false);
    initGizmo();

    /// -------------------------------

    FDML_LE3_LoadEnvironment(LE3GetAssetManager(), "SM_room", env); 

    fflush(stdout);
}

void DemoGUI::runRandomExperiment() {
    std::chrono::steady_clock::time_point begin, end;    
    std::chrono::duration<double, std::milli> __duration;
    begin = std::chrono::steady_clock::now();

    env.runExperiment(20, 0.05, 0.01, true);

    end = std::chrono::steady_clock::now();
    __duration = end - begin;
    print("FDML method: {} [sec]\n", __duration.count() / 1000.0f);

    for (auto pred : env.predictions) {
        fmt::print("Prediction: {}, {}, {}, {}\n", pred.position.x(), pred.position.y(), pred.position.z(), pred.orientation);
        FT error = sqrt((pred.position.x() - env.q0.position.x()) * (pred.position.x() - env.q0.position.x()) +
                   (pred.position.y() - env.q0.position.y()) * (pred.position.y() - env.q0.position.y()) +
                   (pred.position.z() - env.q0.position.z()) * (pred.position.z() - env.q0.position.z()));
        fmt::print("Error: {}\n", error); 
    }
    fmt::print("Ground Truth: {}, {}, {}, {}\n", env.q0.position.x(), env.q0.position.y(), env.q0.position.z(), env.q0.orientation);
    fmt::print("--------------------\n");

    // Result visualization
    configurationsHead = configurations.size();
    for (int idx = 0; idx < configurations.size(); idx++) {
        std::string markerName = format("__marker_{}", idx + 1);
        std::string droneMarkerName = format("__drone_{}", idx + 1);
        LE3GetSceneManager().getActiveScene()->getObject(markerName)->getTransform().setScale(0.f);
        LE3GetSceneManager().getActiveScene()->getObject(droneMarkerName)->getTransform().setScale(0.f);
    }
    for (auto q : env.groundTruths) addConfigurationMarker(q);
}

void DemoGUI::update(float deltaTime) {
    
    LE3SimpleDemo::update(deltaTime);
    updateGizmo();

    ImGui::Begin("UAV FDM-Localization Demo");
    if (ImGui::Button("Randomize"))
        runRandomExperiment();
    ImGui::End();
}


// ------------------------------------------------


void DemoGUI::renderDebug() {
    // Draw configurations: measure downards (and draw hitpoint)
    int idx = 0;
    for (auto q : configurations) {
        if (idx++ < configurationsHead) continue;
        double distance = q.measureDistance(env.getTree());
        glm::vec3 pos(q.position.x(), q.position.z(), q.position.y()); // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
        LE3GetVisualDebug().drawDebugLine(
            pos, pos - glm::vec3(0.f, distance, 0.f),
            glm::vec3(1.f, 0.f, 1.f)
        );
        LE3GetVisualDebug().drawDebugBox(
            pos - glm::vec3(0.f, distance, 0.f),
            glm::quat(), glm::vec3(0.05f, 0.05f, 0.05f), glm::vec3(1.f, 0.f, 1.f));
    }

    // Draw ground truth location
    glm::vec3 pos(env.q0.position.x(), env.q0.position.z(), env.q0.position.y());
    LE3GetVisualDebug().drawDebugBox(pos, glm::quat(), glm::vec3(0.3f, 0.3f, 0.3f), glm::vec3(0.f, 1.f, 0.f));

    // Draw bounding box
    debugDrawVoxel(env.getBoundingBox(), glm::vec3(1.f, 0.f, 0.f));

    // Display the roadmap
    // displayRoadmap();

    for (auto v : env.localization) debugDrawVoxel(v, glm::vec3(0.f, 1.f, 1.f));
}

void DemoGUI::addConfigurationMarker(fdml::R3xS1 q) {
    configurations.push_back(q);
    std::string markerName = format("__marker_{}", configurations.size());
    std::string droneMarkerName = format("__drone_{}", configurations.size());

    LE3GetSceneManager().getActiveScene()->addStaticModel(markerName, "SM_cursor", "M_cursor", "", DRAW_PRIORITY_HIGH);
    LE3GetSceneManager().getActiveScene()->addStaticModel(droneMarkerName, "SM_drone", "M_drone");
    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    LE3GetSceneManager().getActiveScene()->getObject(markerName)->getTransform().setScale(1.75f);
    LE3GetSceneManager().getActiveScene()->getObject(markerName)->getTransform().setRotationRPY(0.f, 0.f, q.orientation);
    LE3GetSceneManager().getActiveScene()->getObject(markerName)->getTransform().setPosition(glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.z()), CGAL::to_double(q.position.y())));

    LE3GetSceneManager().getActiveScene()->getObject(droneMarkerName)->getTransform().setScale(0.05f);
    LE3GetSceneManager().getActiveScene()->getObject(droneMarkerName)->getTransform().setRotationRPY(0.f, 0.f, q.orientation);
    LE3GetSceneManager().getActiveScene()->getObject(droneMarkerName)->getTransform().setPosition(glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.z()), CGAL::to_double(q.position.y())));
}

void DemoGUI::debugDrawVoxel(fdml::R3xS1_Voxel voxel, glm::vec3 color) {
    glm::vec3 bottomLeft = glm::vec3(voxel.bottomLeftPosition.x(), voxel.bottomLeftPosition.z(), voxel.bottomLeftPosition.y());
    glm::vec3 topRight = glm::vec3(voxel.topRightPosition.x(), voxel.topRightPosition.z(), voxel.topRightPosition.y());
    LE3GetVisualDebug().drawDebugBox(bottomLeft + 0.5f * (topRight - bottomLeft), glm::quat(), topRight - bottomLeft, color);
}

void DemoGUI::displayRoadmap() {
    glm::vec3 color(0.f, 1.f, 0.f);

    for (auto n : env.getRoadmap().getNodes()) {
        glm::vec3 p1(n->p.x(), n->p.z(), n->p.y());
        for (auto neighbor : n->neighbors) {
            glm::vec3 p2(neighbor->p.x(), neighbor->p.z(), neighbor->p.y());
            LE3GetVisualDebug().drawDebugLine(p1, p2, color);
        }
    }
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