#pragma once

#include <string>

#include <fmt/format.h>
#include <boost/core/span.hpp>
#include <boost/container/vector.hpp>
using fmt::format;

#include "random.h"

namespace se3loc {
    template <int dim, typename dtype>
    struct Point {
    public:
        dtype data[dim] = {0};

        // Causes a lot of trouble..
        // Point(uint32_t k, ...) {
        //     va_list va;
        //     va_start(va, k);
        //     for (int i = 0; i < k && i < dim; ++i) data[i] = va_arg(va, dtype);
        //     va_end(va);
        // }
        Point() {}


        inline dtype operator[](const uint8_t& idx) { return data[idx]; }
        inline Point<dim, dtype> operator-(Point<dim, dtype>& other) { 
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

    template <typename dtype>
    struct Triangle3 {
        Point3<dtype> a, b, c;

        Vec3<dtype> getNormal(){
            Vec3<dtype> v1 = b - a, v2 = c - a;
            Vec3<dtype> n;
            n.data[0] = v1[1] * v2[2] - v1[2] * v2[1];
            n.data[1] = v1[2] * v2[0] - v1[0] * v2[2];
            n.data[2] = v1[0] * v2[1] - v1[1] * v2[0];
            return n;
        }
    };

    template <typename dtype>
    struct SphericalPolygon {
        dtype theta1, theta2; // Theta is from 0 to pi
        dtype phi1, phi2; // Phi is from 0 to 2pi

        inline static Point3<dtype> point(dtype theta, dtype phi) {
            Point3<dtype> res;
            res.data[0] = sin(theta) * cos(phi);
            res.data[1] = sin(theta) * sin(phi);
            res.data[2] = cos(theta);
            return res;
        }

        boost::container::vector<Triangle3<dtype>> toMesh(int32_t resolution) {
            boost::container::vector<Triangle3<dtype>> mesh;
            for (int32_t i = 0; i < resolution - 1; ++i) {
                dtype t = theta1 + (dtype)i / (dtype)(resolution - 1) * (theta2 - theta1);
                dtype t_ = theta1 + (dtype)(i+1) / (dtype)(resolution - 1) * (theta2 - theta1);

                for (int32_t j = 0; j < resolution - 1; ++j) {
                    dtype p = phi1 + (dtype)j / (dtype)(resolution - 1) * (phi2 - phi1);
                    dtype p_ = phi1 + (dtype)(j+1) / (dtype)(resolution - 1) * (phi2 - phi1);

                    Triangle3<dtype> t1, t2;
                    t1.a = SphericalPolygon::point(t, p);
                    t1.b = SphericalPolygon::point(t, p_);
                    t1.c = SphericalPolygon::point(t_, p_);
                    t2.a = SphericalPolygon::point(t, p);
                    t2.b = SphericalPolygon::point(t_, p_);
                    t2.c = SphericalPolygon::point(t_, p);
                    mesh.push_back(t1); 
                    mesh.push_back(t2);
                }
            }
            return mesh;
        }
    };

    template <typename dtype>
    std::string meshToSTL(boost::container::vector<Triangle3<dtype>> mesh) {
        std::string stl = "solid name\n";
        for (auto triangle : mesh) {
            Vec3<dtype> n = triangle.getNormal();
            stl += format("\tfacet normal {} {} {}\n", n[0], n[1], n[2]);
            stl += format("\t\touter loop\n");
            stl += format("\t\t\tvertex {} {} {}\n", triangle.a[0], triangle.a[1], triangle.a[2]);
            stl += format("\t\t\tvertex {} {} {}\n", triangle.b[0], triangle.b[1], triangle.b[2]);
            stl += format("\t\t\tvertex {} {} {}\n", triangle.c[0], triangle.c[1], triangle.c[2]);
            stl += format("\tendloop\n");
            stl += format("\tendfacet\n");
        }
        stl += "endsolid name";
        return stl;
    }

}