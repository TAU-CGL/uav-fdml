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
    m_scene.load("/fdml/demo_scene.lua");

    m_scene.getMainCamera()->getTransform().setPosition(glm::vec3(0.f, 0.05f, 0.f));
    m_scene.getMainCamera()->setPitchYaw(0.f, -1.57f);

    buildAABBTree();
    addConfiguration(fdml::R3xS1(Point(0, 0.2, 0.3), 0));
    addConfiguration(fdml::R3xS1(Point(0, 0.3, 0.5), 0));

    fflush(stdout);
}

void DemoGUI::renderDebug() {
    for (auto q : configurations) {
        double distance = q.measureDistance(tree);
        LE3GetVisualDebug().drawDebugLine(
            glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.y()), CGAL::to_double(q.position.z())),
            glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.y()) - distance, CGAL::to_double(q.position.z())),
            glm::vec3(1.f, 0.f, 1.f)
        );
        LE3GetVisualDebug().drawDebugBox(
            glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.y()) - distance, CGAL::to_double(q.position.z())),
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
    K k;
    
    for (int i = 0; i < indices.size(); i += 3) {
        Point p1(vertices[indices[i]].position[0], vertices[indices[i]].position[1], vertices[indices[i]].position[2]);
        Point p2(vertices[indices[i + 1]].position[0], vertices[indices[i + 1]].position[1], vertices[indices[i + 1]].position[2]);
        Point p3(vertices[indices[i + 2]].position[0], vertices[indices[i + 2]].position[1], vertices[indices[i + 2]].position[2]);
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
    m_scene.addStaticModel(markerName, "SM_cursor", "M_cursor");
    m_scene.getObject(markerName)->getTransform().setPosition(glm::vec3(CGAL::to_double(q.position.x()), CGAL::to_double(q.position.y()), CGAL::to_double(q.position.z())));
}
