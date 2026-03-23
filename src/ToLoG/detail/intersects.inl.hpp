#pragma once

template<vector_type P>
inline bool intersects(const P& _p, const P& _q)
{
    return _p == _q;
}

template<vector_type P>
inline bool intersects(const AABB<P>& _a, const AABB<P>& _b)
{
    using FT = typename Traits<P>::value_type;
    constexpr int DIM = Traits<P>::dim;
    for (int i = 0; i < DIM; ++i) {
        if (_a.max()[i] < _b.min()[i] || _b.max()[i] < _a.min()[i]) {
            return false;
        }
    }
    return true;
}

template<vector_type P>
inline bool intersects(const AABB<P>& _box, const P& _q)
{
    for (int i = 0; i < Traits<P>::dim; ++i) {
        if (_q[i] < _box.min()[i] || _q[i] > _box.max()[i]) {
            return false;
        }
    }
    return true;
}

template<vector_type P>
inline bool intersects(const P& _q, const AABB<P>& _box)
{
    return intersects(_box, _q);
}

template<vector_type P>
inline bool intersects(const Sphere<P>& _sphere, const P& _q)
{
    return squared_norm(static_cast<P>(_q - _sphere.center())) <= _sphere.squared_radius();
}

template<vector_type P>
inline bool intersects(const P& _q, const Sphere<P>& _sphere)
{
    return intersects(_sphere, _q);
}

template<vector_type P>
inline bool intersects(const Sphere<P>& _s1, const Sphere<P>& _s2)
{
    return norm(static_cast<P>(_s1.center()-_s2.center())) <= _s1.radius()+_s2.radius();
}

template<vector_type P>
inline bool intersects(const Ellipsoid<P>& _e, const P& _p)
{
    using FT = typename Traits<P>::value_type;
    constexpr int DIM = Traits<P>::dim;
    const P p = _p - _e.center();
    FT Q(0);
    for (int i = 0; i < DIM; ++i) {
        FT t = dot(p, _e.direction(i)) / _e.radius(i);
        Q += t*t;
    }
    return Q <= FT(1);
}

template<vector_type P>
inline bool intersects(const P& _p, const Ellipsoid<P>& _e)
{
    return intersects(_e, _p);
}

template<vector_type P> requires(Traits<P>::dim==2)
inline bool intersects(const Triangle<P>& _tri, const P& _q)
{
    const double o0 = point_orient2d(_tri[0],_tri[1],_q);
    const double o1 = point_orient2d(_tri[1],_tri[2],_q);
    const double o2 = point_orient2d(_tri[2],_tri[0],_q);
    return !((o0>0.0||o1>0.0||o2>0.0) && (o0<0.0||o1<0.0||o2<0.0));
}

template<vector_type P>
inline bool intersects(const P& _q, const Triangle<P>& _tri)
{
    return intersects(_tri, _q);
}

template<vector_type P> requires(Traits<P>::dim==3)
inline bool intersects(const Tetrahedron<P>& _tet, const P& _q)
{
    const double o0 = point_orient3d(_tet[0],_tet[1],_tet[2],_q);
    const double o1 = point_orient3d(_tet[0],_tet[3],_tet[1],_q);
    const double o2 = point_orient3d(_tet[0],_tet[2],_tet[3],_q);
    const double o3 = point_orient3d(_tet[1],_tet[3],_tet[2],_q);
    return !((o0>0.0||o1>0.0||o2>0.0||o3>0.0) && (o0<0.0||o1<0.0||o2<0.0||o3<0.0));
}

template<vector_type P>
inline bool intersects(const P& _q, const Tetrahedron<P>& _tet)
{
    return intersects(_tet, _q);
}
