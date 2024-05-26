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

    buildAABBTree();
    addConfiguration(fdml::R3xS1(Point(0, 0.2, 0.3), 0));
    addConfiguration(fdml::R3xS1(Point(0, 0.3, 0.5), 0));
    addConfiguration(fdml::R3xS1(Point(0.7, 0.1, 0.2), 0));

    // Add gizmo to bottom left (for clatiry)
    LE3GetSceneManager().getActiveScene()->addCustomObject("gizmo", std::make_shared<LE3Gizmo>());
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setHoverable(false);
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setDynamicScale(false);
    LE3GetSceneManager().getActiveScene()->getObject<LE3Gizmo>("gizmo")->setMaterial(LE3GetAssetManager().getMaterial("M_custom_gizmo"));
    LE3GetActiveScene()->getObject("gizmo")->getTransform().setPosition(glm::vec3(0.f, .0f, -5.f));

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
}

void DemoGUI::buildAABBTree() {
    auto mesh = LE3GetAssetManager().getStaticMesh("SM_room");
    auto vertices = mesh->getKeptData();
    auto indices = mesh->getKeptIndices();

    std::chrono::steady_clock::time_point begin, end;
    // std::list<Triangle> triangles;
    
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
}

void DemoGUI::addConfiguration(fdml::R3xS1 q) {
    configurations.push_back(q);
    std::string markerName = format("__marker_{}", configurations.size());
    LE3GetSceneManager().getActiveScene()->addStaticModel(markerName, "SM_cursor", "M_cursor");
    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    LE3GetSceneManager().getActiveScene()->getObject(markerName)->getTransform().setPosition(glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.z()), CGAL::to_double(q.position.y())));
}
