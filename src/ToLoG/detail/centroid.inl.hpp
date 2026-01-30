#pragma once

template<typename P>
requires(is_vector_type<P>::value)
inline P centroid(const P& _point)
{
    return _point;
}

template<typename P>
inline P centroid(const AABB<P>& _aabb)
{
    return (_aabb.min() + _aabb.max()) / typename Traits<P>::value_type(2.0);
}

template<typename P>
inline P centroid(const Segment<P>& _seg)
{
    return (_seg.end() + _seg.start()) / typename Traits<P>::value_type(2.0);
}

template<typename P>
inline P centroid(const Triangle<P>& _tri)
{
    return (_tri[0]+_tri[2]+_tri[2]) / typename Traits<P>::value_type(3.0);
}

template<typename P>
inline P centroid(const Sphere<P>& _sphere)
{
    return _sphere.center();
}

template<typename P>
inline P centroid(const Tetrahedron<P>& _tet)
{
    return (_tet[0]+_tet[1]+_tet[2]+_tet[3]) / typename Traits<P>::value_type(4.0);
}
