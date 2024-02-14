#include "geometry.h"
using namespace se3loc;

#include <fmt/core.h>
using fmt::format;

template <typename dtype>
std::string Point3<dtype>::str() {
    return format("({}, {}, {})", x, y, z);
}



// Decalre supported class templates
namespace se3loc {
    template class Point3<int32_t>;
    template class Point3<int64_t>;
    template class Point3<float>;
    template class Point3<double>;
}