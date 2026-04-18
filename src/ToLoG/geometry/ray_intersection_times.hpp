#pragma once
#include <ToLoG/geometry/Ray.hpp>
#include <ToLoG/geometry/Shapes.hpp>
#include <ToLoG/math/polynomial_roots.hpp>
#include <ToLoG/geometry/vector_math.hpp>

namespace ToLoG
{

template<vector Point>
std::vector<double> ray_intersection_times(const Ray<Point>& _ray, const Sphere<Point>& _sphere)
{
    Point oc = _ray.origin() - _sphere.center();
    double a = 1.0;
    double b = 2.0 * dot(_ray.dir(), oc);
    double c = dot(oc, oc) - _sphere.squared_radius();
    return solve_quadratic(a, b, c);
}

template<vector Point>
std::vector<double> ray_intersection_times(const Ray<Point>& _ray, const AABB<Point>& _box)
{
    double tmin = -std::numeric_limits<double>::infinity();
    double tmax = std::numeric_limits<double>::infinity();
    for (int i = 0; i < Traits<Point>::dim; ++i) {
        double o = _ray.origin()[i];
        double d = _ray.dir()[i];
        double a = _box.min()[i];
        double b = _box.max()[i];

        if (std::abs(d)<=1e-14) { // Ray parallel to slab
            if (o < a || o > b) {
                return {}; // no intersection
            }
        } else {
            double t1 = (a - o) / d;
            double t2 = (b - o) / d;
            if (t1 > t2) {std::swap(t1, t2);}
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) {
                return {}; // outside
            }
        }
    }
    if (tmin == tmax) {return {tmin};}
    return {tmin, tmax};
}

}
