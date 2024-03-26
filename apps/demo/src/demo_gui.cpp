#include "demo_gui.h"

void DemoGUI::init() {
    LE3SimpleDemo::init();

    LE3GetAssetManager().getMaterial("M_default")->specularIntensity = 0.f;
    m_scene.addBox("floor", "M_default", glm::vec3(0.f, -1.1f, 0.f), glm::vec3(50.f, 0.1f, 50.f));

    m_scene.addAmbientLight("ambientLight");
    m_scene.getObject<LE3AmbientLight>("ambientLight")->setIntensity(0.3);

    m_scene.addPointCloud("spoly", "M_default");
    updateGeometry();
}

void DemoGUI::update(float deltaTime) {
    LE3SimpleDemo::update(deltaTime);

    if (ImGui::CollapsingHeader("Display Properties", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::LabelText("##lpointSize", "Point Size");
        if (ImGui::SliderFloat("##pointSize", &pointSize, 1.f, 20.f)) updateGeometry();
        ImGui::LabelText("##lresolution", "Resolution");
        if (ImGui::SliderInt("##resolution", &demoObj.m_spolyResolution, 10000, 1000000)) updateGeometry();
    }

    if (ImGui::CollapsingHeader("Spherical Polygon", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::LabelText("##ltheta1", "theta1");
        if (ImGui::SliderFloat("##theta1", &demoObj.m_spoly.theta1, 0, M_PI)) updateGeometry();
        ImGui::LabelText("##ltheta2", "theta2");
        if (ImGui::SliderFloat("##theta2", &demoObj.m_spoly.theta2, 0, M_PI)) updateGeometry();
        // ImGui::LabelText("##lphi1", "phi1");
        // if (ImGui::SliderFloat("##phi1", &demoObj.m_spoly.phi1, 0, 2*M_PI)) updateGeometry();
        // ImGui::LabelText("##lphi2", "phi2");
        // if (ImGui::SliderFloat("##phi2", &demoObj.m_spoly.phi2, 0, 2*M_PI)) updateGeometry();
    }

    if (ImGui::CollapsingHeader("Box", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::LabelText("##lx1", "x1");
        if (ImGui::SliderFloat("##x1", &demoObj.m_box.min.data[0], -2.f, 2.f)) updateGeometry();
        ImGui::LabelText("##lx2", "x2");
        if (ImGui::SliderFloat("##x2", &demoObj.m_box.max.data[0], -2.f, 2.f)) updateGeometry();
        ImGui::LabelText("##ly1", "y1");
        if (ImGui::SliderFloat("##y1", &demoObj.m_box.min.data[1],  -2.f, 2.f)) updateGeometry();
        ImGui::LabelText("##ly2", "y2");
        if (ImGui::SliderFloat("##y2", &demoObj.m_box.max.data[1],  -2.f, 2.f)) updateGeometry();
        ImGui::LabelText("##lz1", "z1");
        if (ImGui::SliderFloat("##z1", &demoObj.m_box.min.data[2],  -2.f, 2.f)) updateGeometry();
        ImGui::LabelText("##lz2", "z2");
        if (ImGui::SliderFloat("##z2", &demoObj.m_box.max.data[2],  -2.f, 2.f)) updateGeometry();
    }

    if (ImGui::CollapsingHeader("Offset (g)", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::LabelText("##lgx", "g_x");
        if (ImGui::SliderFloat("##gx", &demoObj.m_g.x, -2.f, 2.f)) updateGeometry();
        ImGui::LabelText("##lgy", "g_y");
        if (ImGui::SliderFloat("##gy", &demoObj.m_g.y, -2.f, 2.f)) updateGeometry();
    }
}

void DemoGUI::updateGeometry() {
    // std::vector<glm::vec3> pc = demoObj.createSPolyPC();
    // std::vector<glm::vec3> pc = demoObj.createSPolyPCMinkSum();
    std::vector<glm::vec3> pc = demoObj.createCircleMinkSum();
    m_scene.getObject<LE3PointCloud>("spoly")->setPointSize(pointSize);
    m_scene.getObject<LE3PointCloud>("spoly")->clear();
    m_scene.getObject<LE3PointCloud>("spoly")->addPoints(pc, pc);
    m_scene.getObject<LE3PointCloud>("spoly")->create();
}