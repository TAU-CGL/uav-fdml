#pragma once

#include <string>

#include <fmt/format.h>
#include <boost/core/span.hpp>
using fmt::format;

#include "random.h"

namespace se3loc {
    template <int dim, typename dtype>
    struct Point {
    public:
        dtype data[dim] = {0};
        Point(uint32_t k, ...) {
            va_list va;
            va_start(va, dim);
            for (int i = 0; i < k && i < dim; ++i) data[i] = va_arg(va, dtype);
            va_end(va);
        }
        Point() {}


        inline dtype operator[](const uint8_t& idx) { return data[idx]; }
        inline Point<dim, dtype> operator-(const Point<dim, dtype>& other) { 
            Point res;
            for (int i = 0; i < dim; ++i) res.data[i] = this->data[i] - other.data[i];
            return res;
        }

        dtype squaredNorm() const { 
            dtype res = 0;
            for (int i = 0; i < dim; ++i) res += data[i] * data[i];
            return res;
        }

        std::string str() {
            return fmt::format("point_{}({})", dim, fmt::join(boost::span(data, data + dim), ", "));
        }



        static Point uniform() {
            Point p;
            for (int i = 0; i < dim; i++) 
                p.data[i] = (dtype)Random::randomDouble();
            return p;
        }
        // static Point randomPointInt() {
        //     for (int i = 0; i < dim; i++) data[i] = distInt(rng);
        // }
    };

    template <int dim, typename dtype>
    using Vec = Point<dim, dtype>;

    template <typename dtype>
    using Point3 = Point<3, dtype>;
    template <typename dtype>
    using Vec3 = Vec<3, dtype>;
}