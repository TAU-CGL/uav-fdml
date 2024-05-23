#include <queue>

#include <fmt/core.h>
using fmt::print, fmt::format;

#include "demo.h"

int main() {
    fdml::Random::seed(0);

    LE3Application app(std::make_unique<DemoGUI>());
    app.run();
    
    return 0;
}