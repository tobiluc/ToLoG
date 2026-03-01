#pragma once

template<vector_type P>
inline AABB<P> aabb(const P& _point)
{
    return AABB<P>({_point});
}

template<vector_type P>
inline AABB<P> aabb(const AABB<P>& _aabb)
{
    return _aabb;
}

template<vector_type P>
inline AABB<P> aabb(const Segment<P>& _seg)
{
    return AABB<P>({_seg.start(), _seg.end()});
}

template<vector_type P>
inline AABB<P> aabb(const Triangle<P>& _tri)
{
    return AABB<P>({_tri[0], _tri[1], _tri[2]});
}

template<vector_type P>
inline AABB<P> aabb(const Sphere<P>& _sphere)
{
    return AABB<P>({
        _sphere.center() - filled<P>(_sphere.radius()),
        _sphere.center() + filled<P>(_sphere.radius())
    });
}

template<vector_type P>
inline AABB<P> aabb(const Tetrahedron<P>& _tet)
{
    return AABB<P>({_tet[0], _tet[1], _tet[2], _tet[3]});
}
