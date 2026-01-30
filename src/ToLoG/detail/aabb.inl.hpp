#pragma once

template<class P>
requires(is_vector_type<P>::value)
inline AABB<P> aabb(const P& _point)
{
    return AABB<P>({_point});
}

template<typename P>
inline AABB<P> aabb(const AABB<P>& _aabb)
{
    return _aabb;
}

template<typename P>
inline AABB<P> aabb(const Segment<P>& _seg)
{
    return AABB<P>({_seg.start(), _seg.end()});
}

template<typename P>
inline AABB<P> aabb(const Triangle<P>& _tri)
{
    return AABB<P>({_tri[0], _tri[1], _tri[2]});
}

template<typename P>
inline AABB<P> aabb(const Sphere<P>& _sphere)
{
    return AABB<P>({
        _sphere.center() - filled<P>(_sphere.radius()),
        _sphere.center() + filled<P>(_sphere.radius())
    });
}

template<typename P>
inline AABB<P> aabb(const Tetrahedron<P>& _tet)
{
    return AABB<P>({_tet[0], _tet[1], _tet[2], _tet[3]});
}

// template<class P, typename = std::enable_if_t<is_vector_type<P>::value>>
// inline AABB<P> aabb(const P& _point)
// {
//     return AABB<P>({_point});
// }

