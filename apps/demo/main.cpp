#include <fmt/core.h>
using fmt::print, fmt::format;

#include <se3loc/include/geometry.h>

int main() {
    se3loc::Point3<double> point(1, 2, 3);
    se3loc::Vec3<double> vec(4, 5, 6);
    print("{}, {}\n", point.str(), vec.str());

    return 0;
}