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

    /// -------------------------------

    buildAABBTree();    
    runRandomExperiment();

    fflush(stdout);
}

void DemoGUI::runRandomExperiment() {
    FT r0 = fdml::Random::randomDouble() * 2 * M_PI * 0;
    fdml::R3xS1 q0(Point(0.5, 1.2, 0.3), r0);
    fdml::R3xS1 currentQ = q0;
    odometrySequence.clear();
    odometrySequence.push_back(fdml::R3xS1(Point(0, 0, 0), 0));

    for (int i = 0; i < 10; i++) {
        // We want a sequence of 10 true measurements
        while (true) {
            FT x = fdml::Random::randomDouble() * 2 - 1;
            FT y = fdml::Random::randomDouble() * 2 - 1;
            FT z = fdml::Random::randomDouble() * 0.7 - 0.45;
            FT r = fdml::Random::randomDouble() * 2 * M_PI;
            fdml::R3xS1 odometry(Point(x, y, z), r);
            fdml::R3xS1 tmpQ = odometry * currentQ;
            if (tmpQ.measureDistance(tree) < 0) continue;
            odometrySequence.push_back(odometry);
            currentQ = odometry * currentQ;
            break;
        }
    }

    fdml::OdometrySequence groundTruths = fdml::getGroundTruths(odometrySequence, q0);
    measurements = fdml::getMeasurementSequence(tree, groundTruths);

    // Do localization
    std::chrono::steady_clock::time_point begin, end;    
    std::chrono::duration<double, std::milli> __duration;
    begin = std::chrono::steady_clock::now();

    localization = fdml::localize(tree, odometrySequence, measurements, boundingBox, 10);

    end = std::chrono::steady_clock::now();
    __duration = end - begin;
    print("FDML method: {} [sec]\n", __duration.count());

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
        double distance = q.measureDistance(tree);
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
    debugDrawVoxel(boundingBox, glm::vec3(1.f, 0.f, 0.f));

    for (auto v : localization) debugDrawVoxel(v, glm::vec3(0.f, 1.f, 1.f));
    auto v = localization[0];
    for (int j = 0; j < tildeOdometries.size(); j++) {
        debugDrawVoxel(v.forwardOdometry(tildeOdometries[j], measurements[j]), glm::vec3(1.f, 1.f, 0.f));
    }
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
    tree = AABBTree(triangles.begin(), triangles.end());
    end = std::chrono::steady_clock::now();
    __duration = end - begin;
    print("Constructing AABB tree: {} [sec]\n", __duration.count());

    // Get bounding box from BB tree
    boundingBox.bottomLeftPosition = Point(tree.bbox().xmin(), tree.bbox().ymin(), tree.bbox().zmin());
    boundingBox.topRightPosition = Point(tree.bbox().xmax(), tree.bbox().ymax(), tree.bbox().zmax());
    boundingBox.bottomLeftRotation = 0.0f;
    boundingBox.topRightRotation = 2.f * M_PI;
}

void DemoGUI::addConfiguration(fdml::R3xS1 q) {
    configurations.push_back(q);
    std::string markerName = format("__marker_{}", configurations.size());
    std::string droneMarkerName = format("__drone_{}", configurations.size());

    LE3GetSceneManager().getActiveScene()->addStaticModel(markerName, "SM_cursor", "M_cursor", "", DRAW_PRIORITY_HIGH);
    LE3GetSceneManager().getActiveScene()->addStaticModel(droneMarkerName, "SM_drone", "M_drone");
    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
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