#pragma once

#include <chrono>
#include <fmt/core.h>
using fmt::format, fmt::print;

#include <le3.h>
using namespace le3;

#include "demo.h"
#include <fdml/fdml.h>

class DemoGUI : public LE3SimpleDemo {
public:   
    void init();
    void renderDebug();
    void update(float deltaTime);

    void buildAABBTree();
    void addConfiguration(fdml::R3xS1 q);

protected:
    AABBTree tree;
    
    std::vector<fdml::R3xS1> configurations;
    std::list<Triangle> triangles;
};
