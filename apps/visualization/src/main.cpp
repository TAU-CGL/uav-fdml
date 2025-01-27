#include "visualization.h"

int main() {
    fdml::Random::seed(0);

    LE3Application app(std::make_unique<DemoGUI>());
    app.getSettings().bImGuiDocking = false;
    app.run();
    
    return 0;
}