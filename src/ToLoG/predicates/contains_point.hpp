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
    const double o0 = point_orient2d(_tri[0],_tri[1],_q);
    const double o1 = point_orient2d(_tri[1],_tri[2],_q);
    const double o2 = point_orient2d(_tri[2],_tri[0],_q);
    return !((o0>0.0||o1>0.0||o2>0.0) && (o0<0.0||o1<0.0||o2<0.0));
}

template<class P> requires(is_vector_type<P>::value && Traits<P>::dim==3)
inline bool contains_point(const Tetrahedron<P>& _tet, const P& _q)
{
    const double o0 = point_orient3d(_tet[0],_tet[1],_tet[2],_q);
    const double o1 = point_orient3d(_tet[0],_tet[3],_tet[1],_q);
    const double o2 = point_orient3d(_tet[0],_tet[2],_tet[3],_q);
    const double o3 = point_orient3d(_tet[1],_tet[3],_tet[2],_q);
    return !((o0>0.0||o1>0.0||o2>0.0||o3>0.0) && (o0<0.0||o1<0.0||o2<0.0||o3<0.0));
}

}
