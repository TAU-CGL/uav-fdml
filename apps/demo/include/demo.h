#pragma once

#include <le3.h>
using namespace le3;

#include <se3loc/se3loc.h>

class DemoObject {
public:
    DemoObject();

    std::vector<LE3Vertex> createSPoly();
    std::vector<glm::vec3> createSPolyPC();

    se3loc::SphericalPolygon<float> m_spoly;
    int m_spolyResolution = 10000;

};