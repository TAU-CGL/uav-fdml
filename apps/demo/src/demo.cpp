#include "demo.h"

void DemoGUI::init() {
    LE3SimpleDemo::init();

    LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");
    LE3GetSceneManager().getActiveScene()->load("/fdml/demo_scene.lua");

    LE3GetSceneManager().getActiveScene()->getMainCamera()->getTransform().setPosition(glm::vec3(0.f, 0.05f, 0.f));
    LE3GetSceneManager().getActiveScene()->getMainCamera()->setPitchYaw(0.f, -1.57f);

    // Add gizmo to bottom left (for clatiry)
    LE3GetSceneManager().getActiveScene()->addCustomObject("gizmo", std::make_shared<LE3Gizmo>());
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setHoverable(false);
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setDynamicScale(false);
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setMaterial(LE3GetAssetManager().getMaterial("M_custom_gizmo"));
    LE3GetActiveScene()->getObject("gizmo")->getTransform().setPosition(glm::vec3(0.f, .0f, -5.f));

    LE3GetActiveScene()->setCulling(false);

    /// -------------------------------

    buildAABBTree();    
    // runRandomExperiment();

    fflush(stdout);
}

void DemoGUI::runRandomExperiment() {
    
    fdml::OdometrySequence groundTruths = env.getRoadmap().randomWalk(100);
    fdml::R3xS1 q0 = groundTruths[0];
    measurements = fdml::getMeasurementSequence(env.getTree(), groundTruths);

    odometrySequence.clear();
    odometrySequence.push_back(fdml::R3xS1(Point(0, 0, 0), 0));
    for (int i = 1; i < groundTruths.size(); i++) {
        odometrySequence.push_back(groundTruths[i] / groundTruths[i - 1]);
    }

    // Do localization
    std::chrono::steady_clock::time_point begin, end;    
    std::chrono::duration<double, std::milli> __duration;
    begin = std::chrono::steady_clock::now();

    fdml::ErrorBounds errorBounds;
    errorBounds.errorDistance = 0.005;
    errorBounds.errorOdometryX = 0.005;
    errorBounds.errorOdometryY = 0.005;
    errorBounds.errorOdometryZ = 0.005;

    // Add random errors to odometry and measurements
    // for (auto& odometry : odometrySequence) {
    //     odometry.position = Point(odometry.position.x() + 2.0 * errorBounds.errorOdometryX * (fdml::Random::randomDouble() - 0.5),
    //                               odometry.position.y() + 2.0 * errorBounds.errorOdometryY * (fdml::Random::randomDouble() - 0.5),
    //                               odometry.position.z() + 2.0 * errorBounds.errorOdometryZ * (fdml::Random::randomDouble() - 0.5));
    //     odometry.orientation = odometry.orientation + 2.0 * errorBounds.errorOdometryR * (fdml::Random::randomDouble() - 0.5);
    // }
    // for (auto& measurement : measurements) 
    //     measurement += 2.0 * errorBounds.errorDistance * (fdml::Random::randomDouble() - 0.5);
        

    localization = fdml::localize(env.getTree(), odometrySequence, measurements, env.getBoundingBox(), 8, errorBounds);
    auto predictions = fdml::clusterLocations(localization);

    for (auto pred : predictions) {
        fmt::print("Prediction: {}, {}, {}, {}\n", pred.position.x(), pred.position.y(), pred.position.z(), pred.orientation);
        FT error = sqrt((pred.position.x() - q0.position.x()) * (pred.position.x() - q0.position.x()) +
                   (pred.position.y() - q0.position.y()) * (pred.position.y() - q0.position.y()) +
                   (pred.position.z() - q0.position.z()) * (pred.position.z() - q0.position.z()));
        fmt::print("Error: {}\n", error); 
    }
    fmt::print("Ground Truth: {}, {}, {}, {}\n", q0.position.x(), q0.position.y(), q0.position.z(), q0.orientation);
    fmt::print("--------------------\n");

    end = std::chrono::steady_clock::now();
    __duration = end - begin;
    print("FDML method: {} [sec]\n", __duration.count() / 1000.0f);

    // Result visualization
    configurationsHead = configurations.size();
    for (int idx = 0; idx < configurations.size(); idx++) {
        std::string markerName = format("__marker_{}", idx + 1);
        std::string droneMarkerName = format("__drone_{}", idx + 1);
        LE3GetSceneManager().getActiveScene()->getObject(markerName)->getTransform().setScale(0.f);
        LE3GetSceneManager().getActiveScene()->getObject(droneMarkerName)->getTransform().setScale(0.f);
    }
    for (auto q : groundTruths) addConfiguration(q);
}

void DemoGUI::update(float deltaTime) {
    
    LE3SimpleDemo::update(deltaTime);

    glm::vec3 cameraPos = LE3GetActiveScene()->getMainCamera()->getPosition();
    glm::vec3 cameraForward = LE3GetActiveScene()->getMainCamera()->getForward();
    glm::vec3 cameraRight = LE3GetActiveScene()->getMainCamera()->getRight();
    glm::vec3 cameraUp = LE3GetActiveScene()->getMainCamera()->getUp();
    glm::vec3 gizmoPosition = cameraPos + cameraForward * 0.5f + cameraRight * 0.0f + cameraUp * 0.0f;
    // LE3GetSceneManager().getActiveScene()->getObject("gizmo")->getTransform().setPosition(cameraPos + glm::vec3(0.5f, 0.f, 0.f));

    ImGui::Begin("UAV FDM-Localization Demo");
    if (ImGui::Button("Randomize"))
        runRandomExperiment();
    if (badMeasurement)
        ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Bad measurement (not enough samples). Please randomize again.");
    ImGui::End();
}

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

    // Debug the algorithm
    debugDrawVoxel(env.getBoundingBox(), glm::vec3(1.f, 0.f, 0.f));

    // Display the roadmap
    // displayRoadmap();

    for (auto v : localization) debugDrawVoxel(v, glm::vec3(0.f, 1.f, 1.f));
    // if (localization.size() > 0) {
    //     auto v = localization[0];
    //     for (int j = 0; j < tildeOdometries.size(); j++) {
    //         debugDrawVoxel(v.forwardOdometry(tildeOdometries[j], measurements[j])[0], glm::vec3(1.f, 1.f, 0.f));
    //     }
    // }
}

void DemoGUI::buildAABBTree() {
    auto mesh = LE3GetAssetManager().getStaticMesh("SM_room");
    auto vertices = mesh->getKeptData();
    auto indices = mesh->getKeptIndices();

    std::chrono::steady_clock::time_point begin, end;    
    std::chrono::duration<double, std::milli> __duration;
    begin = std::chrono::steady_clock::now();
    
    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    for (int i = 0; i < indices.size(); i += 3) {
        Point p1(vertices[indices[i]].position[0], vertices[indices[i]].position[2], vertices[indices[i]].position[1]);
        Point p2(vertices[indices[i + 1]].position[0], vertices[indices[i + 1]].position[2], vertices[indices[i + 1]].position[1]);
        Point p3(vertices[indices[i + 2]].position[0], vertices[indices[i + 2]].position[2], vertices[indices[i + 2]].position[1]);
        Triangle t(p1, p2, p3);
        triangles.push_back(t);
    }
    env.loadTriangles(triangles);
    end = std::chrono::steady_clock::now();
    __duration = end - begin;
    print("Constructing environment: {} [sec]\n", __duration.count());
}

void DemoGUI::addConfiguration(fdml::R3xS1 q) {
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