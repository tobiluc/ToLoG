#pragma once
#include <ToLoG/Core.hpp>
#include <ToLoG/math/polynomial_roots.hpp>

namespace ToLoG
{

template<typename P> requires(is_vector_type<P>::value)
class Ray
{
public:
    Ray(P _o, P _d) : o_(_o), d_(normalized(_d)) {}
    inline const P& origin() const {return o_;}
    inline const P& dir() const {return d_;}
    inline P point(Traits<P>::value_type _t) const {return o_ + scaled(d_, _t);}
private:
    P o_;
    P d_;
};

template<typename Point>
struct Traits<Ray<Point>>
{
    using value_type = Traits<Point>::value_type;
    using vector_type = Point;
    constexpr static int dim = Traits<Point>::dim;
};

template<typename Point, typename Object> requires(is_vector_type<Point>::value)
std::vector<typename Traits<Point>::value_type> ray_intersection_times(const Ray<Point>& _ray, const Object& _obj);

template<typename Point> requires(is_vector_type<Point>::value)
std::vector<typename Traits<Point>::value_type> ray_intersection_times(const Ray<Point>& _ray, const Sphere<Point>& _sphere)
{
    using FT = typename Traits<Point>::value_type;
    Point oc = _ray.origin() - _sphere.center();
    FT a = 1.0;
    FT b = 2.0 * dot(_ray.dir(), oc);
    FT c = dot(oc, oc) - _sphere.squared_radius();
    return solve_quadratic(a, b, c);
}

}
