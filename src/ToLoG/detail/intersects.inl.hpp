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

template<vector_type P> requires(Traits<P>::dim==2)
inline bool intersects(const Segment<P>& _seg, const P& _p)
{
    AABB<P> seg_aabb({_seg.start(), _seg.end()});
    return point_orient2d(_seg.start(), _seg.end(), _p) == 0.0
           && intersects(seg_aabb, _p);
}

template<vector_type P> requires(Traits<P>::dim==2)
inline bool intersects(const P& _p, const Segment<P>& _seg)
{
    return intersects(_seg, _p);
}

template<vector_type P> requires(Traits<P>::dim==2)
inline bool intersects(const Segment<P>& _s0, const Segment<P>& _s1)
{
    const P& a = _s0.start();
    const P& b = _s0.end();
    const P& c = _s1.start();
    const P& d = _s1.end();

    const double o1 = point_orient2d(a, b, c);
    const double o2 = point_orient2d(a, b, d);
    const double o3 = point_orient2d(c, d, a);
    const double o4 = point_orient2d(c, d, b);

    return ((o1 > 0.0 && o2 < 0.0 || o1 < 0.0 && o2 > 0.0) &&
            (o3 > 0.0 && o4 < 0.0 || o3 < 0.0 && o4 > 0.0))
        || (o1 == 0.0 && intersects(_s0, c))
        || (o2 == 0.0 && intersects(_s0, d))
        || (o3 == 0.0 && intersects(_s1, a))
        || (o4 == 0.0 && intersects(_s1, b));
}

template<vector_type P> requires(Traits<P>::dim==2)
inline bool intersects(const Segment<P>& _seg, const Triangle<P>& _tri)
{
    return intersects(_seg.start(), _tri)
        || intersects(_seg.end(), _tri)
        || intersects(_seg, _tri.segment(0))
        || intersects(_seg, _tri.segment(1))
        || intersects(_seg, _tri.segment(2));
}

template<vector_type P> requires(Traits<P>::dim==2)
inline bool intersects(const Triangle<P>& _tri, const Segment<P>& _seg)
{
    return intersects(_seg, _tri);
}

template<vector_type P> requires(Traits<P>::dim==3)
inline bool intersects(const Segment<P>& _seg, const Triangle<P>& _tri)
{
    double o0, o1, o2;
    o0 = point_orient3d(_tri[0], _tri[1], _tri[2], _seg.start());
    o1 = point_orient3d(_tri[0], _tri[1], _tri[2], _seg.end());
    if ((o0>0.0&&o1>0.0) || (o0<0.0&&o1<0.0)) {return false;}
    if (o0==0.0&&o1==0.0) [[unlikely]] {
        // coplanar -> project to 2d
        using FT = typename Traits<P>::value_type;
        using P2 = Point<FT,2>;
        auto proj2 = [&](const P& _p, int _a) -> P2 {
            return P2(_p[(_a+1)%3], _p[(_a+2)%3]);
        };
        const AABB<P> tri_aabb({_tri[0], _tri[1], _tri[2]});
        int a = argmax(static_cast<P>(tri_aabb.max()-tri_aabb.min()));
        Segment<P2> seg2(proj2(_seg.start(), a), proj2(_seg.end(), a));
        Triangle<P2> tri2(proj2(_tri[0], a), proj2(_tri[1], a), proj2(_tri[2], a));
        return intersects(seg2, tri2);
    }
    o0 = point_orient3d(_tri[0], _tri[1], _seg.start(), _seg.end());
    o1 = point_orient3d(_tri[1], _tri[2], _seg.start(), _seg.end());
    o2 = point_orient3d(_tri[2], _tri[0], _seg.start(), _seg.end());
    return !((o0>0.0||o1>0.0||o2>0.0) && (o0<0.0||o1<0.0||o2<0.0));
}

template<vector_type P> requires(Traits<P>::dim==3)
inline bool intersects(const Triangle<P>& _tri, const Segment<P>& _seg)
{
    return intersects<P>(_seg, _tri);
}
