#include "demo.h"

namespace fdml {
    void printTriangle(Triangle t) {
        print("facet normal 0 0 0\n");
        print("\touter loop\n");
        print("\t\tvertex {} {} {}\n", CGAL::to_double(t.vertex(0).x()), CGAL::to_double(t.vertex(0).y()), CGAL::to_double(t.vertex(0).z()));
        print("\t\tvertex {} {} {}\n", CGAL::to_double(t.vertex(1).x()), CGAL::to_double(t.vertex(1).y()), CGAL::to_double(t.vertex(1).z()));
        print("\t\tvertex {} {} {}\n", CGAL::to_double(t.vertex(2).x()), CGAL::to_double(t.vertex(2).y()), CGAL::to_double(t.vertex(2).z()));
        print("\tendloop\n");
        print("endfacet\n");
    }

};

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
    fdml::R3xS1 q0(Point(0, 0.2, 0.3), 0);
    odometrySequence.push_back(fdml::R3xS1(Point(0, 0, 0), 0));
    odometrySequence.push_back(fdml::R3xS1(Point(0, 0.5, 0.2), 0.1));
    odometrySequence.push_back(fdml::R3xS1(Point(0.7, -1.0, -0.5), 0.2));
    odometrySequence.push_back(fdml::R3xS1(Point(-2.0, -1.0, -0.3), 0.1));
    odometrySequence.push_back(fdml::R3xS1(Point(1.5, 0.0, 0.6), 0.1));
    odometrySequence.push_back(fdml::R3xS1(Point(0.5, 0.5, 0.01), 0.1));

    fdml::OdometrySequence groundTruths;
    bool first = true;
    for (auto g : odometrySequence) {
        if (first) {
            groundTruths.push_back(g * q0);
            tildeOdometries.push_back(g);
            first = false;
        }
        else {
            groundTruths.push_back(g * groundTruths.back());
            tildeOdometries.push_back(g * tildeOdometries.back());
        }
    }

    buildAABBTree();

    for (auto q : groundTruths) {
        measurements.push_back(q.measureDistance(tree));
    }


    // Do localization
    std::chrono::steady_clock::time_point begin, end;    
    std::chrono::duration<double, std::milli> __duration;
    begin = std::chrono::steady_clock::now();

    std::vector<fdml::R3xS1_Voxel> voxels;
    fdml::R3xS1_Voxel v1 = boundingBox, v2 = boundingBox, v3 = boundingBox, v4 = boundingBox;
    v1.bottomLeftRotation = 0.0; v1.topRightRotation = M_PI / 2.0;
    v2.bottomLeftRotation = M_PI / 2.0; v2.topRightRotation = M_PI;
    v3.bottomLeftRotation = M_PI; v3.topRightRotation = 3.0 * M_PI / 2.0;
    v4.bottomLeftRotation = 3.0 * M_PI / 2.0; v4.topRightRotation = 2.0 * M_PI;
    voxels.push_back(v1); voxels.push_back(v2); voxels.push_back(v3); voxels.push_back(v4);

    for (int j = 0; j < tildeOdometries.size(); ++j) {
        fmt::print("j: {}| [{}, {}, {}] {}m\n", 
            j, tildeOdometries[j].position.x(), tildeOdometries[j].position.y(), tildeOdometries[j].position.z(), measurements[j]
        );
    }

    for (int i = 0; i < 8; i++) {
        localization.clear();
        for (auto v : voxels) {
            // if (i > 3 && tree.do_intersect(Box(v.bottomLeftPosition, v.topRightPosition))) {
            //     continue;
            // }
            bool flag = true;
            for (int j = 0; j < tildeOdometries.size(); ++j) {
                if (!v.predicate(tildeOdometries[j], measurements[j], tree)) {
                    flag = false;
                    break;
                }
            }
            if (flag) localization.push_back(v);
        }
        fmt::print("localization size: {}\n", localization.size());

        voxels.clear();
        for (auto v : localization) v.split(voxels);
    }

    end = std::chrono::steady_clock::now();
    __duration = end - begin;
    print("FDML method: {} [sec]\n", __duration.count());

    LE3GetActiveScene()->addPointCloud("localization", "M_default");

    for (auto v : localization) {
        LE3GetActiveScene()->getObject<LE3PointCloud>("localization")->addPoint(
            glm::vec3(v.bottomLeftPosition.x(), v.bottomLeftPosition.z(), v.bottomLeftPosition.y())
        );
        // fmt::print("{} {} {}\n", v.bottomLeftPosition.x(), v.bottomLeftPosition.z(), v.bottomLeftPosition.y());
    }
    LE3GetActiveScene()->getObject<LE3PointCloud>("localization")->create();
    LE3GetActiveScene()->getObject<LE3PointCloud>("localization")->setPointSize(10.f);
    LE3GetActiveScene()->getObject<LE3PointCloud>("localization")->setMaterial(LE3GetAssetManager().getMaterial("M_localization"));

    for (auto q : groundTruths) addConfiguration(q);

    fflush(stdout);
}

void DemoGUI::update(float deltaTime) {
    
    LE3SimpleDemo::update(deltaTime);

    glm::vec3 cameraPos = LE3GetActiveScene()->getMainCamera()->getPosition();
    glm::vec3 cameraForward = LE3GetActiveScene()->getMainCamera()->getForward();
    glm::vec3 cameraRight = LE3GetActiveScene()->getMainCamera()->getRight();
    glm::vec3 cameraUp = LE3GetActiveScene()->getMainCamera()->getUp();
    glm::vec3 gizmoPosition = cameraPos + cameraForward * 0.5f + cameraRight * 0.0f + cameraUp * 0.0f;
    // LE3GetSceneManager().getActiveScene()->getObject("gizmo")->getTransform().setPosition(cameraPos + glm::vec3(0.5f, 0.f, 0.f));

}

void DemoGUI::renderDebug() {
    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    for (auto q : configurations) {
        double distance = q.measureDistance(tree);
        glm::vec3 pos(q.position.x(), q.position.z(), q.position.y());
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

    // fdml::R3xS1_Voxel v_gt;
    // v_gt.bottomLeftPosition = Point(0 - 0.1, 0.2 - 0.1, 0.3 - 0.1);
    // v_gt.topRightPosition = Point(0 + 0.1, 0.2 + 0.1, 0.3 + 0.1);
    // v_gt.bottomLeftRotation = 0.0; v_gt.topRightRotation = 2.0 * M_PI;
    // // debugDrawVoxel(v_gt, glm::vec3(0.f, 1.f, 1.f));
    // for (int j = 0; j < tildeOdometries.size(); j++) {
    //     debugDrawVoxel(v_gt.forwardOdometry(tildeOdometries[j], measurements[j]), glm::vec3(1.f, 1.f, 0.f));
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
        updateBoundingBox(p1);
        updateBoundingBox(p2);
        updateBoundingBox(p3);
        triangles.push_back(t);
    }
    tree = AABBTree(triangles.begin(), triangles.end());
    end = std::chrono::steady_clock::now();
    __duration = end - begin;
    print("Constructing AABB tree: {} [sec]\n", __duration.count());
}

void DemoGUI::addConfiguration(fdml::R3xS1 q) {
    configurations.push_back(q);
    std::string markerName = format("__marker_{}", configurations.size());
    LE3GetSceneManager().getActiveScene()->addStaticModel(markerName, "SM_cursor", "M_cursor");
    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    LE3GetSceneManager().getActiveScene()->getObject(markerName)->getTransform().setPosition(glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.z()), CGAL::to_double(q.position.y())));
}

void DemoGUI::updateBoundingBox(Point pt) {
    if (pt.x() < boundingBox.bottomLeftPosition.x()) boundingBox.bottomLeftPosition = Point(pt.x(), boundingBox.bottomLeftPosition.y(), boundingBox.bottomLeftPosition.z());
    if (pt.y() < boundingBox.bottomLeftPosition.y()) boundingBox.bottomLeftPosition = Point(boundingBox.bottomLeftPosition.x(), pt.y(), boundingBox.bottomLeftPosition.z());
    if (pt.z() < boundingBox.bottomLeftPosition.z()) boundingBox.bottomLeftPosition = Point(boundingBox.bottomLeftPosition.x(), boundingBox.bottomLeftPosition.y(), pt.z());
    if (pt.x() > boundingBox.topRightPosition.x()) boundingBox.topRightPosition = Point(pt.x(), boundingBox.topRightPosition.y(), boundingBox.topRightPosition.z());
    if (pt.y() > boundingBox.topRightPosition.y()) boundingBox.topRightPosition = Point(boundingBox.topRightPosition.x(), pt.y(), boundingBox.topRightPosition.z());
    if (pt.z() > boundingBox.topRightPosition.z()) boundingBox.topRightPosition = Point(boundingBox.topRightPosition.x(), boundingBox.topRightPosition.y(), pt.z());
}

void DemoGUI::debugDrawVoxel(fdml::R3xS1_Voxel voxel, glm::vec3 color) {
    glm::vec3 bottomLeft = glm::vec3(voxel.bottomLeftPosition.x(), voxel.bottomLeftPosition.z(), voxel.bottomLeftPosition.y());
    glm::vec3 topRight = glm::vec3(voxel.topRightPosition.x(), voxel.topRightPosition.z(), voxel.topRightPosition.y());
    LE3GetVisualDebug().drawDebugBox(bottomLeft + 0.5f * (topRight - bottomLeft), glm::quat(), topRight - bottomLeft, color);
}