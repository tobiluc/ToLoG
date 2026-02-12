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
inline P scaled(const P& _p, typename Traits<P>::value_type _s) {
    P res;
    for (int i = 0; i < Traits<P>::dim; ++i) {
        res[i] = _s * _p[i];
    }
    return res;
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
    return scaled(_v, typename Traits<P>::value_type(1.0) / norm(_v));
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
