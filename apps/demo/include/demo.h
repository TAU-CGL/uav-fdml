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
    void update(float deltaTime);

    void buildAABBTree();
protected:
    AABBTree tree;
};
