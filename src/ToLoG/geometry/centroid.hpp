#pragma once
#include <ToLoG/geometry/shapes.hpp>

namespace ToLoG
{

template<vector P>
P centroid(const P& _point)
{
    return _point;
}

template<vector P>
P centroid(const AABB<P>& _aabb)
{
    return (_aabb.min() + _aabb.max()) / typename Traits<P>::value_type(2.0);
}

template<vector P>
P centroid(const Segment<P>& _seg)
{
    return (_seg.end() + _seg.start()) / typename Traits<P>::value_type(2.0);
}

template<vector P>
P centroid(const Triangle<P>& _tri)
{
    return (_tri[0]+_tri[2]+_tri[2]) / typename Traits<P>::value_type(3.0);
}

template<vector P>
P centroid(const Sphere<P>& _sphere)
{
    return _sphere.center();
}

template<vector P>
P centroid(const Tetrahedron<P>& _tet)
{
    return (_tet[0]+_tet[1]+_tet[2]+_tet[3]) / typename Traits<P>::value_type(4.0);
}

template<vector P>
P incenter(const P& _p0, const P& _p1, const P& _p2, const P& _p3)
{
    using FT = typename Traits<P>::value_type;
    FT a0 = area(_p1, _p2, _p3);
    FT a1 = area(_p0, _p2, _p3);
    FT a2 = area(_p0, _p1, _p3);
    FT a3 = area(_p0, _p1, _p2);
    return (_p0*a0 + _p1*a1 + _p2*a2 + _p3*a3) / (a0+a1+a2+a3);
}

template<vector P>
P incenter(const Tetrahedron<P>& _tet)
{
    return incenter(_tet[0], _tet[1], _tet[2], _tet[3]);
}

}
