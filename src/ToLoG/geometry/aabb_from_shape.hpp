#pragma once
#include <ToLoG/geometry/shapes.hpp>
#include <ToLoG/geometry/vector_math.hpp>

namespace ToLoG
{

template<vector P>
inline AABB<P> aabb(const P& _point)
{
    return AABB<P>({_point});
}

template<vector P>
inline AABB<P> aabb(const AABB<P>& _aabb)
{
    return _aabb;
}

template<vector P>
inline AABB<P> aabb(const Segment<P>& _seg)
{
    return AABB<P>({_seg.start(), _seg.end()});
}

template<vector P>
inline AABB<P> aabb(const Triangle<P>& _tri)
{
    return AABB<P>({_tri[0], _tri[1], _tri[2]});
}

template<vector P>
inline AABB<P> aabb(const Sphere<P>& _sphere)
{
    return AABB<P>({
        _sphere.center() - filled<P>(_sphere.radius()),
        _sphere.center() + filled<P>(_sphere.radius())
    });
}

template<vector P>
inline AABB<P> aabb(const Tetrahedron<P>& _tet)
{
    return AABB<P>({_tet[0], _tet[1], _tet[2], _tet[3]});
}

template<vector P>
inline AABB<P> aabb(const Ellipsoid<P>& _ell)
{
    using FT = typename Traits<P>::value_type;
    constexpr int DIM = Traits<P>::dim;

    std::array<FT, DIM> extent = {0};

    for (int i = 0; i < DIM; ++i) {
        for (int j = 0; j < DIM; ++j) {
            FT v = _ell.radius(i) * _ell.direction(i)[j];
            extent[j] += v * v;
        }
    }

    AABB<P> box;
    for (int j = 0; j < DIM; ++j) {
        extent[j] = std::sqrt(extent[j]);
        box.min()[j] = _ell.center()[j] - extent[j];
        box.max()[j] = _ell.center()[j] + extent[j];
    }

    return box;
}

}
