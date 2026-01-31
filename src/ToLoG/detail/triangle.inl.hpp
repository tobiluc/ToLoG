#pragma once

template<class P>
inline typename Traits<P>::value_type area(const Triangle<P>& _tri) {
    using FT = typename Traits<P>::value_type;
    P u = _tri[1] - _tri[0];
    P v = _tri[2] - _tri[0];
    FT uu = dot(u,u);
    FT vv = dot(v,v);
    FT uv = dot(u,v);
    FT det = uu * vv - uv * uv;
    return std::sqrt<FT>(std::max<FT>(det, FT(0))) / FT(2);
}

template<class P> requires(Traits<P>::dim==3)
inline P normal(const Triangle<P>& _tri) {
    return cross((_tri[1] - _tri[0]),(_tri[2] - _tri[0]));
}

template<class P>
inline typename Traits<P>::value_type circumference(const Triangle<P>& _tri) {
    return norm(_tri[1]-_tri[0]) + norm(_tri[2]-_tri[1]) + norm(_tri[0]-_tri[2]);
}
