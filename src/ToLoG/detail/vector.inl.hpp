#pragma once

template<vector_type P>
inline P rounded(const P& _p) {
    using FT = typename Traits<P>::value_type;
    P r;
    for (int i = 0; i < Traits<P>::dim; ++i) {
        r[i] = static_cast<int>((_p[i] < 0)? (_p[i] - static_cast<FT>(0.5)) : (_p[i] + static_cast<FT>(0.5)));
    }
    return r;
}

template<vector_type P>
inline typename Traits<P>::value_type dot(const P& _lhs, const P& _rhs) {
    typename Traits<P>::value_type res(0);
    for (int i = 0; i < Traits<P>::dim; ++i) {
        res += _lhs[i] * _rhs[i];
    }
    return res;
}

template<vector_type P>
inline typename Traits<P>::value_type squared_norm(const P& _p) {
    return dot(_p, _p);
}

template<vector_type P>
inline typename Traits<P>::value_type norm(const P& _p) {
    return std::sqrt<typename Traits<P>::value_type>(dot(_p, _p));
}

template<vector_type P> requires(Traits<P>::dim==3)
inline P normalized(const P& _v) {
    using FT = typename Traits<P>::value_type;
    FT n = norm(_v);
    if (n == 0) [[unlikely]] {
        //std::cerr << "Warning: Normalizing with 0 norm" << std::endl;
        return P(FT(0),FT(0),FT(0));
    }
    return _v / n;
}

template<vector_type P> requires(Traits<P>::dim==3)
inline P cross(const P& _lhs, const P& _rhs) {
    return P(
        _lhs[1]*_rhs[2] - _lhs[2]*_rhs[1],
        _lhs[2]*_rhs[0] - _lhs[0]*_rhs[2],
        _lhs[0]*_rhs[1] - _lhs[1]*_rhs[0]
        );
}

template<vector_type P>
inline P filled(typename Traits<P>::value_type _val)
{
    P res;
    for (int i = 0; i < Traits<P>::dim; ++i) {
        res[i] = _val;
    };
    return res;
}

template<vector_type P>
inline P abs(const P& _p)
{
    P res;
    for (int i = 0; i < Traits<P>::dim; ++i) {
        res[i] = std::abs(_p[i]);
    };
    return res;
}

template<vector_type P>
inline int argmax(const P& _p)
{
    int idx(0);
    for (int i = 0; i < Traits<P>::dim; ++i) {
        if (_p[i] > _p[idx]) {
            idx = i;
        }
    };
    return idx;
}

template<vector_type P>
inline int argmin(const P& _p)
{
    int idx(0);
    for (int i = 0; i < Traits<P>::dim; ++i) {
        if (_p[i] < _p[idx]) {
            idx = i;
        }
    };
    return idx;
}

template<vector_type P>
inline typename Traits<P>::value_type max(const P& _p)
{
    using FT = typename Traits<P>::value_type;
    FT max_val = -std::numeric_limits<FT>::infinity();
    for (int i = 0; i < Traits<P>::dim; ++i) {
        max_val = std::max<FT>(max_val, _p[i]);
    };
    return max_val;
}

template<vector_type P>
inline typename Traits<P>::value_type min(const P& _p)
{
    using FT = typename Traits<P>::value_type;
    FT min_val = std::numeric_limits<FT>::infinity();
    for (int i = 0; i < Traits<P>::dim; ++i) {
        min_val = std::min<FT>(min_val, _p[i]);
    };
    return min_val;
}

template<vector_type P>
inline typename Traits<P>::value_type angle(const P& _a,
                                            const P& _b)
{
    using FT = typename Traits<P>::value_type;
    FT na = norm(_a);
    FT nb = norm(_b);

    if (na == FT(0) || nb == FT(0)) [[unlikely]] {
        return FT(0);
    }

    FT cos_alpha = dot(_a, _b) / (na * nb);
    cos_alpha = std::max(FT(-1), std::min(FT(1), cos_alpha));
    return std::acos(cos_alpha);
}

/**
 * Dihedral Angle between two halfplanes in radians.
 * One halfplane is given by abp, the other by baq
 **/
template<vector_type P> requires(Traits<P>::dim == 3)
inline typename Traits<P>::value_type dihedral_angle(
    const P& _a,
    const P& _b,
    const P& _p,
    const P& _q)
{
    using FT = typename Traits<P>::value_type;

    P e = _b - _a;

    // Plane normals
    P n1 = cross(e, _p - _a);
    P n2 = cross(e, _q - _b);

    FT nn1 = norm(n1);
    FT nn2 = norm(n2);

    if (nn1 == FT(0) || nn2 == FT(0)) [[unlikely]] {
        std::cerr << "Warning: Cannot compute dihedral angle between degenerate triangles" << std::endl;
        return FT(0);
    }

    n1 = n1 / nn1;
    n2 = n2 / nn2;

    // Compute cosine and sine of angle
    FT cos_theta = std::max(FT(-1), std::min(FT(1), dot(n1, n2)));
    FT sin_theta = dot(cross(n1, n2), normalized(e));

    return std::atan2(sin_theta, cos_theta);
}
