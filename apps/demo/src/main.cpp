#include <queue>

#include <fmt/core.h>
using fmt::print, fmt::format;

#include <se3loc/se3loc.h>

int main() {
    se3loc::Random::seed(0);

    se3loc::SphericalPolygon<double> spoly;
    spoly.theta1 = 0.1; spoly.theta2 = 0.9;
    spoly.phi1 = 0.3; spoly.phi2 = 0.9;

    boost::container::vector<se3loc::Triangle3<double>> mesh = spoly.toMesh(100);
    print("{}\n", se3loc::meshToSTL(mesh));
    
    return 0;
}