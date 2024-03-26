#pragma once

#include <le3.h>
using namespace le3;

#include <se3loc/se3loc.h>

class DemoObject {
public:
    DemoObject();

    std::vector<LE3Vertex> createSPoly();
    std::vector<glm::vec3> createSPolyPC();
    std::vector<glm::vec3> createSPolyPCMinkSum();

    // For R3xS1 localization
    std::vector<glm::vec3> createCircleMinkSum();

    se3loc::SphericalPolygon<float> m_spoly;
    se3loc::Box3<float> m_box;
    glm::vec3 m_g;
    int m_spolyResolution = 100000;

};