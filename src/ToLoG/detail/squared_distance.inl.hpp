#pragma once

template<vector_type P>
inline auto squared_distance(const P& _p, const P& _q)
{
    return squared_norm(static_cast<P>(_p - _q));
}

template<vector_type P>
inline auto squared_distance(const AABB<P>& _a, const AABB<P>& _b)
{
    using FT = typename Traits<P>::value_type;
    constexpr int DIM = Traits<P>::dim;
    FT d2(0);
    for (int i = 0; i < DIM; ++i) {
        if (_a.max()[i] < _b.min()[i]) {
            FT d = _b.min()[i] - _a.max()[i];
            d2 += d * d;
        } else if (_b.max()[i] < _a.min()[i]) {
            FT d = _a.min()[i] - _b.max()[i];
            d2 += d * d;
        }
    }
    return d2;
}

template<vector_type PointT>
inline Traits<PointT>::value_type squared_distance(const PointT& _p, const AABB<PointT>& _aabb)
{
    using FT = Traits<PointT>::value_type;
    FT res(0);
    for (int i = 0; i < Traits<PointT>::dim; ++i) {
        FT d = std::max(std::max(
            _aabb.min()[i] - _p[i],
            FT(0)),
            _p[i] - _aabb.max()[i]
        );
        res += d*d;
    }
    return res;
}

template<vector_type PointT>
inline Traits<PointT>::value_type squared_distance(const PointT& _p, const Segment<PointT>& _s)
{
    return squared_norm(static_cast<PointT>(_p - closest_point(_s, _p)));
}

template<vector_type PointT>
inline Traits<PointT>::value_type squared_distance(const PointT& _p, const Sphere<PointT>& _s)
{
    using FT = Traits<PointT>::value_type;
    FT d = std::max(norm(_p - _s.center()) - _s.radius(), static_cast<FT>(0));
    return d*d;
}

template<vector_type PointT>
inline Traits<PointT>::value_type squared_distance(const PointT& _p, const Triangle<PointT>& _tri)
{
    return squared_norm(static_cast<PointT>(_p - closest_point(_tri, _p)));
}

template<vector_type PointT>
inline Traits<PointT>::value_type squared_distance(const PointT& _p, const Tetrahedron<PointT>& _tet)
{
    return squared_norm(static_cast<PointT>(_p - closest_point(_tet, _p)));
}
