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

template<class P> requires(Traits<P>::dim==3)
inline typename Traits<P>::value_type winding_number(const P& _p, const Triangle<P>& _tri)
{
    using FT = typename Traits<P>::value_type;

    P v0 = _tri[0] - _p;
    P v1 = _tri[1] - _p;
    P v2 = _tri[2] - _p;

    FT n0 = norm(v0);
    if (is_near_zero(n0)) {return FT(0)};
    FT n1 = norm(v1);
    if (is_near_zero(n1)) {return FT(0)};
    FT n2 = norm(v2);
    if (is_near_zero(n2)) {return FT(0)};

    for (int i = 0; i < 3; ++i) {
        v0[i] /= n0;
        v1[i] /= n1;
        v2[i] /= n2;
    }

    FT numerator = dot(v0, cross(v1, v2));
    FT denominator = FT(1.) + dot(v0, v1) + dot(v0, v2) + dot(v1, v2);
    return FT(2.) * std::atan2(numerator, denominator);
}
