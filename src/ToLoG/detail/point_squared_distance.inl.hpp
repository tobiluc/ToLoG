#pragma once

template<class P>
requires(is_vector_type<P>::value)
inline auto point_squared_distance(const P& _p, const P& _q)
{
    return squared_norm(_p - _q);
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const AABB<PointT>& _aabb)
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

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const Segment<PointT>& _s)
{
    using FT = Traits<PointT>::value_type;
    if (_s.end() == _s.start()) {
        return point_squared_distance(_p, _s.start());
    }
    PointT ab = _s.end() - _s.start();
    PointT ap = _p - _s.start();
    FT t = std::clamp(dot(ap,ab) / squared_norm(ab), FT(0), FT(1));
    PointT closest = _s.start() + ab*t;
    return squared_norm(_p - closest);
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const Sphere<PointT>& _s)
{
    using FT = Traits<PointT>::value_type;
    FT d = std::max(norm(_p - _s.center()) - _s.radius(), static_cast<FT>(0));
    return d*d;
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const Triangle<PointT>& _tri)
{
    using FT = Traits<PointT>::value_type;
    constexpr int DIM = Traits<PointT>::dim;

    std::array<FT,3> bary = barycentric_coordinates(_p. _tri);

    // If inside triangle, project onto triangle plane
    if (bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0) {
        Point<FT,DIM> q = _tri[0] + (_tri[1] - _tri[0])*bary[0] + (_tri[2] - _tri[0])*bary[1]; // closest point
        return (_p - q).squared_norm();
    }

    // Otherwise, closest point is on an edge
    return std::min({
        point_squared_distance(_p, _tri.edge(0)),
        point_squared_distance(_p, _tri.edge(1)),
        point_squared_distance(_p, _tri.edge(2))
    });
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const Tetrahedron<PointT>& _tet)
{
    using FT = Traits<PointT>::value_type;

    const std::array<FT, 4> bary = barycentric_coordinates(_p, _tet);

    // Check if point is inside tetrahedron
    if (bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0 && bary[3] >= 0) {
        return static_cast<FT>(0);
    }

    // Otherwise, closest point lies on a face triangle
    return std::min({
        point_squared_distance(_p, _tet.face(0,1,2)),
        point_squared_distance(_p, _tet.face(0,1,3)),
        point_squared_distance(_p, _tet.face(0,2,3)),
        point_squared_distance(_p, _tet.face(1,2,3))
    });
}
