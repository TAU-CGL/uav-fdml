#include "demo.h"

DemoObject::DemoObject() {
    m_spoly.theta1 = 0.1; m_spoly.theta2 = 0.9;
    m_spoly.phi1 = 0.3; m_spoly.phi2 = 0.9;
}

std::vector<LE3Vertex> DemoObject::createSPoly() {
    boost::container::vector<se3loc::Triangle3<float>> mesh = m_spoly.toMesh(m_spolyResolution);
    std::vector<LE3Vertex> vertices;
    for (auto triangle : mesh) {
        glm::vec3 a = glm::vec3(triangle.a.data[0], triangle.a.data[1], triangle.a.data[2]);
        glm::vec3 b = glm::vec3(triangle.b.data[0], triangle.b.data[1], triangle.b.data[2]);
        glm::vec3 c = glm::vec3(triangle.c.data[0], triangle.c.data[1], triangle.c.data[2]);
        vertices.push_back(vertexFromGLM(a, glm::vec2(0.f), a));
        vertices.push_back(vertexFromGLM(b, glm::vec2(0.f), b));
        vertices.push_back(vertexFromGLM(c, glm::vec2(0.f), c));
    }
    return vertices;
}
std::vector<glm::vec3> DemoObject::createSPolyPC() {
    std::vector<glm::vec3> vertices;

    for (int i = 0; i < m_spolyResolution; ++i) {
        double theta = se3loc::Random::randomDouble();
        double phi = se3loc::Random::randomDouble();
        theta = theta * m_spoly.theta1 + (1 - theta) * m_spoly.theta2;
        phi = phi * m_spoly.phi1 + (1 - phi) * m_spoly.phi2;

        auto pt = m_spoly.point(theta, phi);
        vertices.push_back(glm::vec3(pt.data[0], pt.data[1], pt.data[2]));
    }

    return vertices;
}