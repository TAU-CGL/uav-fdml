#include "demo_gui.h"

void DemoGUI::init() {
    LE3SimpleDemo::init();

    LE3GetAssetManager().getMaterial("M_default")->specularIntensity = 0.f;
    m_scene.addBox("floor", "M_default", glm::vec3(0.f, -1.1f, 0.f), glm::vec3(50.f, 0.1f, 50.f));

    // m_scene.addDirectionalLight("sun2");
    // std::dynamic_pointer_cast<LE3DirectionalLight>(m_scene.getObject("sun2"))->getTransform().setRotationRPY(-0.785, -0.785, -0.785);
    // std::dynamic_pointer_cast<LE3DirectionalLight>(m_scene.getObject("sun2"))->setIntensity(0.3);

    // m_scene.addStaticModel("spoly", "", "M_default");
    m_scene.addPointCloud("spoly", "M_default");
    updateGeometry();
}

void DemoGUI::update(float deltaTime) {
    LE3SimpleDemo::update(deltaTime);

    if (ImGui::CollapsingHeader("Display Properties", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::LabelText("##lpointSize", "Point Size");
        if (ImGui::SliderFloat("##pointSize", &pointSize, 1.f, 20.f)) updateGeometry();
    }

    if (ImGui::CollapsingHeader("Spherical Polygon", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::LabelText("##ltheta1", "theta1");
        if (ImGui::SliderFloat("##theta1", &demoObj.m_spoly.theta1, 0, M_PI)) updateGeometry();
        ImGui::LabelText("##ltheta2", "theta2");
        if (ImGui::SliderFloat("##theta2", &demoObj.m_spoly.theta2, 0, M_PI)) updateGeometry();
        ImGui::LabelText("##lphi1", "phi1");
        if (ImGui::SliderFloat("##phi1", &demoObj.m_spoly.phi1, 0, 2*M_PI)) updateGeometry();
        ImGui::LabelText("##lphi2", "phi2");
        if (ImGui::SliderFloat("##phi2", &demoObj.m_spoly.phi2, 0, 2*M_PI)) updateGeometry();
        ImGui::LabelText("##lresolution", "resolution");
        if (ImGui::SliderInt("##resolution", &demoObj.m_spolyResolution, 1000, 100000)) updateGeometry();
    }
}

void DemoGUI::updateGeometry() {
    // std::vector<LE3Vertex> vertices = demoObj.createSPoly();
    // LE3StaticMeshPtr sm = std::make_shared<LE3StaticMesh>(vertices);
    // std::dynamic_pointer_cast<LE3StaticModel>(m_scene.getObject("spoly"))->setMesh(sm);

    std::vector<glm::vec3> pc = demoObj.createSPolyPC();
    m_scene.getObject<LE3PointCloud>("spoly")->setPointSize(pointSize);
    m_scene.getObject<LE3PointCloud>("spoly")->clear();
    m_scene.getObject<LE3PointCloud>("spoly")->addPoints(pc, pc);
    m_scene.getObject<LE3PointCloud>("spoly")->create();
}