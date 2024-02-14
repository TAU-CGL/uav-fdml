#include <fmt/core.h>
using fmt::print, fmt::format;

#include <se3loc/se3loc.h>

int main() {
    se3loc::Random::seed(0);

    se3loc::Point3<double> point = se3loc::Point3<double>::uniform();
    se3loc::Vec3<double> vec = se3loc::Vec3<double>::uniform();
 
    return 0;
}