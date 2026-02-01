#pragma once
#include <ToLoG/Core.hpp>
#include <ToLoG/predicates/predicates_wrapper.hpp>


namespace ToLoG
{

template<class P> requires(is_vector_type<P>::value)
inline bool contains_point(const P& _p, const P& _q)
{
    return _p == _q;
}

template<class P> requires(is_vector_type<P>::value)
inline bool contains_point(const AABB<P>& _box, const P& _q)
{
    for (int i = 0; i < Traits<P>::dim; ++i) {
        if (_q[i] < _box.min()[i] || _q[i] > _box.max()[i]) {
            return false;
        }
    }
    return true;
}

template<class P> requires(is_vector_type<P>::value)
inline bool contains_point(const Sphere<P>& _sphere, const P& _q)
{
    return squared_norm(_q - _sphere.center()) <= _sphere.squared_radius();
}

template<class P> requires(is_vector_type<P>::value && Traits<P>::dim==2)
inline bool contains_point(const Triangle<P>& _tri, const P& _q)
{
    const double t0[2] = {static_cast<double>(_tri[0][0]),static_cast<double>(_tri[0][1])};
    const double t1[2] = {static_cast<double>(_tri[1][0]),static_cast<double>(_tri[1][1])};
    const double t2[2] = {static_cast<double>(_tri[2][0]),static_cast<double>(_tri[2][1])};
    const double p[2] = {static_cast<double>(_q[0]),static_cast<double>(_q[1])};
    const double o0 = orient2d(t0,t1,p);
    const double o1 = orient2d(t1,t2,p);
    const double o2 = orient2d(t2,t0,p);
    return !((o0>0.0||o1>0.0||o2>0.0) && (o0<0.0||o1<0.0||o2<0.0));
}

template<class P> requires(is_vector_type<P>::value && Traits<P>::dim==3)
inline bool contains_point(const Tetrahedron<P>& _tet, const P& _q)
{
    const double t0[3] = {static_cast<double>(_tet[0][0]),static_cast<double>(_tet[0][1]),static_cast<double>(_tet[0][2])};
    const double t1[3] = {static_cast<double>(_tet[1][0]),static_cast<double>(_tet[1][1]),static_cast<double>(_tet[1][2])};
    const double t2[3] = {static_cast<double>(_tet[2][0]),static_cast<double>(_tet[2][1]),static_cast<double>(_tet[2][2])};
    const double t3[3] = {static_cast<double>(_tet[3][0]),static_cast<double>(_tet[3][1]),static_cast<double>(_tet[3][2])};
    const double p[3] = {static_cast<double>(_q[0]),static_cast<double>(_q[1]),static_cast<double>(_q[2])};
    const double o0 = orient3d(t0,t1,t2,p);
    const double o1 = orient3d(t0,t3,t1,p);
    const double o2 = orient3d(t0,t2,t3,p);
    const double o3 = orient3d(t1,t3,t2,p);
    return !((o0>0.0||o1>0.0||o2>0.0||o3>0.0) && (o0<0.0||o1<0.0||o2<0.0||o3<0.0));
}

}
