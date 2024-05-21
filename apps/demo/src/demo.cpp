#include "demo.h"

void DemoGUI::init() {
    LE3SimpleDemo::init();

    LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");
    m_scene.load("/fdml/demo_scene.lua");

    m_scene.getMainCamera()->getTransform().setPosition(glm::vec3(0.f, 0.05f, 0.f));
    m_scene.getMainCamera()->setPitchYaw(0.f, -1.57f);

    buildAABBTree();
}

void DemoGUI::update(float deltaTime) {
    LE3SimpleDemo::update(deltaTime);
}

void DemoGUI::buildAABBTree() {
    auto mesh = LE3GetAssetManager().getStaticMesh("SM_room");
    auto vertices = mesh->getKeptData();
    auto indices = mesh->getKeptIndices();

    std::chrono::steady_clock::time_point begin, end;
    std::list<Triangle> triangles;
    
    std::chrono::duration<double, std::milli> __duration;
    begin = std::chrono::steady_clock::now();
    
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
    print("Constructing AABB tree: {} [sec]", __duration.count());
}