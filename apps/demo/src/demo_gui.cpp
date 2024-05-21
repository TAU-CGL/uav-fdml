#include "demo_gui.h"

void DemoGUI::init() {
    LE3SimpleDemo::init();

    LE3GetAssetManager().getMaterial("M_default")->specularIntensity = 0.f;
    m_scene.addBox("floor", "M_default", glm::vec3(0.f, -1.1f, 0.f), glm::vec3(50.f, 0.1f, 50.f));

    m_scene.addAmbientLight("ambientLight");
    m_scene.getObject<LE3AmbientLight>("ambientLight")->setIntensity(0.3);
}

void DemoGUI::update(float deltaTime) {
    LE3SimpleDemo::update(deltaTime);
}

void DemoGUI::updateGeometry() {

}