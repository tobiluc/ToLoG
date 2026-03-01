#pragma once

template<vector_type P>
inline typename Traits<P>::value_type area(const P& _p0, const P& _p1, const P& _p2) {
    using FT = typename Traits<P>::value_type;
    P u = _p1 - _p0;
    P v = _p2 - _p0;
    FT uu = dot(u,u);
    FT vv = dot(v,v);
    FT uv = dot(u,v);
    FT det = uu * vv - uv * uv;
    return std::sqrt<FT>(std::max<FT>(det, FT(0))) / FT(2);
}

template<vector_type P>
inline typename Traits<P>::value_type area(const Triangle<P>& _tri) {
    return area(_tri[0], _tri[1], _tri[2]);
}

template<vector_type P> requires(Traits<P>::dim==3)
inline P normal(const Triangle<P>& _tri) {
    return cross(static_cast<P>(_tri[1] - _tri[0]),static_cast<P>(_tri[2] - _tri[0]));
}

template<vector_type P>
inline typename Traits<P>::value_type circumference(const Triangle<P>& _tri) {
    return norm(static_cast<P>(_tri[1]-_tri[0]))
        + norm(static_cast<P>(_tri[2]-_tri[1]))
        + norm(static_cast<P>(_tri[0]-_tri[2]));
}

/// Signed Tet volume
template<vector_type P>
inline typename Traits<P>::value_type volume(const P& _a, const P& _b, const P& _c, const P& _d) {
    using FT = typename Traits<P>::value_type;
    P ba = _b - _a;
    P ca = _c - _a;
    P da = _d - _a;
    return dot(ba, cross(ca, da)) / static_cast<FT>(6.);
}

/// Signed Tet volume
template<vector_type P>
inline typename Traits<P>::value_type volume(const Tetrahedron<P>& _tet) {
    return volume(_tet[0], _tet[1], _tet[2], _tet[3]);
}

template<vector_type P>
inline typename Traits<P>::value_type volume(const AABB<P>& _box) {
    using FT = typename Traits<P>::value_type;
    FT vol(1);
    for (int i = 0; i < Traits<P>::dim; ++i) {
        vol *= _box.max()[i] - _box.min()[i];
    }
    return vol;
}
