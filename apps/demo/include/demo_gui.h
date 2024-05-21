#pragma once

#include <le3.h>
using namespace le3;

#include "demo.h"

class DemoGUI : public LE3SimpleDemo {
public:   
    void init();
    void update(float deltaTime);

protected:
    DemoObject demoObj;
    void updateGeometry();
};
