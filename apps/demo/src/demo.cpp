#include "demo.h"

void DemoGUI::init() {
    LE3SimpleDemo::init();

    LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");

    LE3GetAssetManager().getMaterial("M_default")->specularIntensity = 0.f;
    // m_scene.addBox("floor", "M_default", glm::vec3(0.f, -1.1f, 0.f), glm::vec3(50.f, 0.1f, 50.f));

    LE3GetAssetManager().addStaticMesh("240521-141038", "/fdml/scans/240521-141038/240521-141038-scaled.obj");
    LE3GetAssetManager().addTexture("240521-141038", "/fdml/scans/240521-141038/240521-141038.jpg");
    LE3GetAssetManager().addMaterial("M_room", "S_default");
    LE3GetAssetManager().getMaterial("M_room")->diffuseTexture = LE3GetAssetManager().getTexture("240521-141038");
    LE3GetAssetManager().getMaterial("M_room")->bUseDiffuseTexture = true;
    m_scene.addStaticModel("room", "240521-141038", "M_room");

    m_scene.addAmbientLight("ambientLight");
    m_scene.getObject<LE3AmbientLight>("ambientLight")->setIntensity(0.8);
}

void DemoGUI::update(float deltaTime) {
    LE3SimpleDemo::update(deltaTime);
}