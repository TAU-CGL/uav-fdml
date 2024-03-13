#include <queue>

#include <fmt/core.h>
using fmt::print, fmt::format;

#include <se3loc/se3loc.h>

#include "demo_gui.h"

int main() {
    se3loc::Random::seed(0);

    LE3Application app(std::make_unique<DemoGUI>());
    app.run();
    
    return 0;
}