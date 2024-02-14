#pragma once

#include <string>

namespace se3loc {
    template <typename dtype>
    struct Point3 {
    public:
        Point3(dtype x = 0, dtype y = 0, dtype z = 0) : x(x), y(y), z(z) {}
        dtype x, y, z;

        std::string str();
    };
    template <typename dtype>
    using Vec3 = Point3<dtype>;
}