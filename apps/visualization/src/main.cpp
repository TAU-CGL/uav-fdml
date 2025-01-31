#include "visualization.h"

int main(int argc, char** argv) {
    fdml::Random::seed(0);

    std::string url = "";
    if (argc > 1) url = std::string(argv[1]);

    LE3Application app(std::make_unique<DemoGUI>(url));
    app.getSettings().bImGuiDocking = false;
    app.startNetworking();
    app.run();
    
    return 0;
}