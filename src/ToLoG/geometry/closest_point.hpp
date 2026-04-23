#pragma once
#include <ToLoG/geometry/Shapes.hpp>
#include <ToLoG/geometry/barycentric.hpp>
#include <ToLoG/geometry/squared_distance.hpp>

namespace ToLoG
{

template<vector P>
P closest_point(const P& _q, const P& _p)
{
    return _q;
}

template<vector P>
P closest_point(const Segment<P>& _s, const P& _p)
{
    using FT = Traits<P>::value_type;
    if (_s.end() == _s.start()) [[unlikely]] {
        return _s.start();
    }
    P ab = _s.end() - _s.start();
    P ap = _p - _s.start();
    FT t = std::clamp(dot(ap,ab) / squared_norm(ab), FT(0), FT(1));
    return _s.start() + ab*t;
}

template<vector P>
P closest_point(const Triangle<P>& _tri, const P& _p)
{
    using FT = Traits<P>::value_type;
    constexpr int DIM = Traits<P>::dim;

    std::array<FT,3> bary = barycentric_coordinates(_p, _tri);

    // If inside triangle, project onto triangle plane
    if (bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0) {
        return _tri[0]*bary[0] + _tri[1]*bary[1] + _tri[2]*bary[2];
    }

    // closest point is on edge
    P q0 = closest_point(_tri.segment(0), _p);
    FT d0 = squared_norm(static_cast<P>(_p - q0));
    P q1 = closest_point(_tri.segment(1), _p);
    FT d1 = squared_norm(static_cast<P>(_p - q1));
    P q2 = closest_point(_tri.segment(2), _p);
    FT d2 = squared_norm(static_cast<P>(_p - q2));
    if (d0 < d1 && d0 < d2) {return q0;}
    if (d1 < d2) {return q1;}
    return q2;
}

template<vector P>
P closest_point(const Tetrahedron<P>& _tet, const P& _p)
{
    using FT = Traits<P>::value_type;
    constexpr int DIM = Traits<P>::dim;

    const std::array<FT, 4> bary = barycentric_coordinates(_p, _tet);

    // Check if point is inside tetrahedron
    if (bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0 && bary[3] >= 0) {
        return _p;
    }

    // Otherwise, closest point lies on a face triangle
    P q0 = closest_point( _tet.triangle(0,1,2), _p);
    P q1 = closest_point( _tet.triangle(0,1,3), _p);
    P q2 = closest_point( _tet.triangle(0,2,3), _p);
    P q3 = closest_point( _tet.triangle(1,2,3), _p);
    FT d0 = squared_norm(static_cast<P>(_p - q0));
    FT d1 = squared_norm(static_cast<P>(_p - q1));
    FT d2 = squared_norm(static_cast<P>(_p - q2));
    FT d3 = squared_norm(static_cast<P>(_p - q3));
    if (d0 < d1 && d0 < d2 && d0 < d3) {return q0;}
    if (d1 < d2 && d1 < d3) {return q1;}
    if (d2 < d3) {return q2;}
    return q3;
}

template<vector P>
P closest_point(const Sphere<P>& _s, const P& _p)
{
    using FT = Traits<P>::value_type;
    if (squared_norm(static_cast<P>(_p - _s.center())) <= _s.squared_radius()) {
        return _p;
    }
    return _s.center() + normalized(_p - _s.center()) * _s.radius();
}

}
