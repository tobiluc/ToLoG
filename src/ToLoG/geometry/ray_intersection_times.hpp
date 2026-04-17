#pragma once
#include <ToLoG/geometry/ray.hpp>
#include <ToLoG/geometry/shapes.hpp>
#include <ToLoG/math/polynomial_roots.hpp>
#include <ToLoG/geometry/vector_math.hpp>

namespace ToLoG
{

template<vector Point>
std::vector<typename Traits<Point>::value_type> ray_intersection_times(const Ray<Point>& _ray, const Sphere<Point>& _sphere)
{
    using FT = typename Traits<Point>::value_type;
    Point oc = _ray.origin() - _sphere.center();
    FT a = 1.0;
    FT b = 2.0 * dot(_ray.dir(), oc);
    FT c = dot(oc, oc) - _sphere.squared_radius();
    return solve_quadratic(a, b, c);
}

template<vector Point>
std::vector<typename Traits<Point>::value_type> ray_intersection_times(const Ray<Point>& _ray, const AABB<Point>& _box)
{
    using FT = typename Traits<Point>::value_type;
    FT tmin = -std::numeric_limits<FT>::infinity();
    FT tmax = std::numeric_limits<FT>::infinity();
    for (int i = 0; i < Traits<Point>::dim; ++i) {
        FT o = _ray.origin()[i];
        FT d = _ray.dir()[i];
        FT a = _box.min()[i];
        FT b = _box.max()[i];

        if (is_near_zero(d)) { // Ray parallel to slab
            if (o < a || o > b) {
                return {}; // no intersection
            }
        } else {
            FT t1 = (a - o) / d;
            FT t2 = (b - o) / d;
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
