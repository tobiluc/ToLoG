#pragma once
#include <ToLoG/geometry/Shapes.hpp>
#include <ToLoG/geometry/predicates/predicates.hpp>
#include <ToLoG/geometry/predicates/is_degenerate.hpp>

namespace ToLoG
{

template<vector P>
bool intersects(const P& _p, const P& _q)
{
    return _p == _q;
}

template<vector P>
bool intersects(const AABB<P>& _a, const AABB<P>& _b)
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

template<vector P>
bool intersects(const AABB<P>& _box, const P& _q)
{
    for (int i = 0; i < Traits<P>::dim; ++i) {
        if (_q[i] < _box.min()[i] || _q[i] > _box.max()[i]) {
            return false;
        }
    }
    return true;
}

template<vector P>
bool intersects(const P& _q, const AABB<P>& _box)
{
    return intersects(_box, _q);
}

template<vector_2_exact_real P>
bool intersects(const Segment<P>& _seg, const P& _p)
{
    return sign_orient2d(_seg.start(), _seg.end(), _p) == ORI::ZERO
           && intersects(AABB<P>({_seg.start(), _seg.end()}), _p);
}

template<vector_2_exact_real P>
bool intersects(const P& _p, const Segment<P>& _seg)
{
    return intersects(_seg, _p);
}

template<vector_2_exact_real P>
bool intersects(const Triangle<P>& _tri, const P& _q)
{
    if (is_degenerate(_tri)) [[unlikely]] {
        return intersects(_tri.segment(0), _q) || intersects(_tri.segment(1), _q);
    }
    const ORI o0 = sign_orient2d(_tri[0],_tri[1],_q);
    const ORI o1 = sign_orient2d(_tri[1],_tri[2],_q);
    const ORI o2 = sign_orient2d(_tri[2],_tri[0],_q);
    return !((o0==ORI::CCW||o1==ORI::CCW||o2==ORI::CCW)
             && (o0==ORI::CW||o1==ORI::CW||o2==ORI::CW));
}

template<vector_2_exact_real P>
bool intersects(const P& _q, const Triangle<P>& _tri)
{
    return intersects(_tri, _q);
}

template<vector_3_exact_real P>
bool intersects(const Tetrahedron<P>& _tet, const P& _q)
{
    if (is_degenerate(_tet)) [[unlikely]] {
        return intersects(_tet.triangle(0,1,2), _q)
            || intersects(_tet.triangle(0,1,3), _q)
            || intersects(_tet.triangle(0,2,3), _q)
            || intersects(_tet.triangle(1,2,3), _q);
    }
    const ORI o0 = sign_orient3d(_tet[0],_tet[1],_tet[2],_q);
    const ORI o1 = sign_orient3d(_tet[0],_tet[3],_tet[1],_q);
    const ORI o2 = sign_orient3d(_tet[0],_tet[2],_tet[3],_q);
    const ORI o3 = sign_orient3d(_tet[1],_tet[3],_tet[2],_q);
    return !((o0==ORI::CCW||o1==ORI::CCW||o2==ORI::CCW||o3==ORI::CCW)
             && (o0==ORI::CW||o1==ORI::CW||o2==ORI::CW||o3==ORI::CW));
}

template<vector_3_exact_real P>
bool intersects(const P& _q, const Tetrahedron<P>& _tet)
{
    return intersects(_tet, _q);
}

template<vector_2_exact_real P>
bool intersects(const Segment<P>& _s0, const Segment<P>& _s1)
{
    const P& a = _s0.start();
    const P& b = _s0.end();
    const P& c = _s1.start();
    const P& d = _s1.end();

    const ORI o1 = sign_orient2d(a, b, c);
    const ORI o2 = sign_orient2d(a, b, d);
    const ORI o3 = sign_orient2d(c, d, a);
    const ORI o4 = sign_orient2d(c, d, b);

    return ((o1 == ORI::CCW && o2 == ORI::CW || o1 == ORI::CW && o2 == ORI::CCW) &&
            (o3 == ORI::CCW && o4 == ORI::CW || o3 == ORI::CW && o4 == ORI::CCW))
           || (o1 == ORI::ZERO && intersects(_s0, c))
           || (o2 == ORI::ZERO && intersects(_s0, d))
           || (o3 == ORI::ZERO && intersects(_s1, a))
           || (o4 == ORI::ZERO && intersects(_s1, b));
}

template<vector_2_exact_real P>
bool intersects(const Segment<P>& _seg, const Triangle<P>& _tri)
{
    return intersects(_seg.start(), _tri)
        || intersects(_seg.end(), _tri)
        || intersects(_seg, _tri.segment(0))
        || intersects(_seg, _tri.segment(1))
        || intersects(_seg, _tri.segment(2));
}

template<vector_3_exact_real P>
bool intersects(const Segment<P>& _seg, const P& _p)
{
    return sign_orient2d_xy(_seg.start(), _seg.end(), _p) == ORI::ZERO
        && sign_orient2d_xz(_seg.start(), _seg.end(), _p) == ORI::ZERO
        && sign_orient2d_yz(_seg.start(), _seg.end(), _p) == ORI::ZERO
        && intersects(AABB<P>({_seg.start(), _seg.end()}), _p);
}

template<vector_3_exact_real P>
bool intersects(const P& _p, const Segment<P>& _seg)
{
    return intersects(_seg, _p);
}

template<vector_3_exact_real P>
bool intersects(const Triangle<P>& _tri, const P& _p)
{
    if (is_degenerate(_tri)) [[unlikely]] {
        return intersects(_tri.segment(0), _p) || intersects(_tri.segment(1), _p);
    }
    // point must lie in triangle plane
    if (sign_orient3d(_tri[0], _tri[1], _tri[2], _p) != ORI::ZERO) {
        return false;
    }
    // project to 2d
    using FT = typename Traits<P>::value_type;
    using P2 = Point<FT,2>;
    auto proj2 = [&](const P& _p, int _a) -> P2 {
        return P2(_p[(_a+1)%3], _p[(_a+2)%3]);
    };
    int a = argmax(abs(cross(static_cast<P>(_tri[1] - _tri[0]), static_cast<P>(_tri[2] - _tri[0]))));
    P2 p2(proj2(_p, a));
    Triangle<P2> tri2(proj2(_tri[0], a), proj2(_tri[1], a), proj2(_tri[2], a));
    return intersects(p2, tri2);
}

template<vector_3_exact_real P>
bool intersects(const P& _p, const Triangle<P>& _tri)
{
    return intersects(_tri, _p);
}

template<vector_2_exact_real P>
bool intersects(const Triangle<P>& _tri, const Segment<P>& _seg)
{
    return intersects(_seg, _tri);
}

template<vector_3_exact_real P>
bool intersects(const Segment<P>& _seg, const Triangle<P>& _tri)
{
    ORI o0, o1, o2;
    o0 = sign_orient3d(_tri[0], _tri[1], _tri[2], _seg.start());
    o1 = sign_orient3d(_tri[0], _tri[1], _tri[2], _seg.end());
    if ((o0==ORI::CCW&&o1==ORI::CCW) || (o0==ORI::CW&&o1==ORI::CW)) {return false;}
    if (o0==ORI::ZERO&&o1==ORI::ZERO) [[unlikely]] {
        // coplanar
        if (is_degenerate(_tri)) [[unlikely]] {
            throw std::runtime_error("triangle segment 3d intersection not implemented for degenerate triangle");
        }
        // not degenerate -> project to 2d
        using FT = typename Traits<P>::value_type;
        using P2 = Point<FT,2>;
        auto proj2 = [&](const P& _p, int _a) -> P2 {
            return P2(_p[(_a+1)%3], _p[(_a+2)%3]);
        };
        int a = argmax(abs(cross(_tri[1] - _tri[0], _tri[2] - _tri[0])));
        Segment<P2> seg2(proj2(_seg.start(), a), proj2(_seg.end(), a));
        Triangle<P2> tri2(proj2(_tri[0], a), proj2(_tri[1], a), proj2(_tri[2], a));
        return intersects(seg2, tri2);
    }
    o0 = sign_orient3d(_tri[0], _tri[1], _seg.start(), _seg.end());
    o1 = sign_orient3d(_tri[1], _tri[2], _seg.start(), _seg.end());
    o2 = sign_orient3d(_tri[2], _tri[0], _seg.start(), _seg.end());
    return !((o0==ORI::CCW||o1==ORI::CCW||o2==ORI::CCW)
             && (o0==ORI::CW||o1==ORI::CW||o2==ORI::CW));
}

template<vector_3_exact_real P>
bool intersects(const Triangle<P>& _tri, const Segment<P>& _seg)
{
    return intersects<P>(_seg, _tri);
}

template<vector_of_dim<1> P>
bool intersects(const P& _p, const Sphere<P>& _sphere)
{
    return _p[0] >= (_sphere.center()[0]-_sphere.radius())
           && _p[0] <= (_sphere.center()[0]+_sphere.radius());
}

template<vector_2_exact_real P>
bool intersects(const P& _p, const Sphere<P>& _sphere)
{
    // We first get three point on the circle to use
    // the exact predicate
    P a = _sphere.center();
    a[0] -= _sphere.radius();
    P b = _sphere.center();
    b[0] += _sphere.radius();
    P c = _sphere.center();
    c[1] += _sphere.radius();
    assert(sign_orient2d(a, b, c) == ORI::CCW);
    return sign_incircle(a, b, c, _p) != ORI::CW;
}

template<vector_3_exact_real P>
bool intersects(const P& _p, const Sphere<P>& _sphere)
{
    // We first get four point on the sphere to use
    // the exact predicate
    P a = _sphere.center();
    a[2] += _sphere.radius();
    P b = _sphere.center();
    b[0] += _sphere.radius();
    P c = _sphere.center();
    c[2] -= _sphere.radius();
    P d = _sphere.center();
    d[1] += _sphere.radius();
    assert(sign_orient3d(a, b, c, d) == ORI::CCW);
    return sign_insphere(a, b, c, d, _p) != ORI::CW;
}

//-----------------------------------------
// The following checks are inexact!
//-----------------------------------------

template<vector P>
bool intersects(const Sphere<P>& _sphere, const P& _q)
{
    return squared_norm(static_cast<P>(_q - _sphere.center())) <= _sphere.squared_radius();
}

template<vector P>
bool intersects(const P& _q, const Sphere<P>& _sphere)
{
    return intersects(_sphere, _q);
}

template<vector P>
bool intersects(const Sphere<P>& _s1, const Sphere<P>& _s2)
{
    return norm(static_cast<P>(_s1.center()-_s2.center())) <= _s1.radius()+_s2.radius();
}

template<vector P>
bool intersects(const Ellipsoid<P>& _e, const P& _p)
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

template<vector P>
bool intersects(const P& _p, const Ellipsoid<P>& _e)
{
    return intersects(_e, _p);
}

}
