#pragma once

template<class P> requires(is_vector_type<P>::value && Traits<P>::dim==3)
inline P scaled(const P& _p, typename Traits<P>::value_type _s) {
    P res;
    for (int i = 0; i < Traits<P>::dim; ++i) {
        res[i] = _s * _p[i];
    }
    return res;
}

template<class P> requires(is_vector_type<P>::value)
inline typename Traits<P>::value_type dot(const P& _lhs, const P& _rhs) {
    typename Traits<P>::value_type res(0);
    for (int i = 0; i < Traits<P>::dim; ++i) {
        res += _lhs[i] * _rhs[i];
    }
    return res;
}

template<class P> requires(is_vector_type<P>::value)
inline typename Traits<P>::value_type squared_norm(const P& _p) {
    return dot(_p, _p);
}

template<class P> requires(is_vector_type<P>::value)
inline typename Traits<P>::value_type norm(const P& _p) {
    return std::sqrt<typename Traits<P>::value_type>(dot(_p, _p));
}

template<class P> requires(is_vector_type<P>::value && Traits<P>::dim==3)
inline P normalized(const P& _v) {
    return scaled(_v, typename Traits<P>::value_type(1.0) / norm(_v));
}

template<class P> requires(is_vector_type<P>::value && Traits<P>::dim==3)
inline P cross(const P& _lhs, const P& _rhs) {
    return P(
        _lhs[1]*_rhs[2] - _lhs[2]*_rhs[1],
        _lhs[2]*_rhs[0] - _lhs[0]*_rhs[2],
        _lhs[0]*_rhs[1] - _lhs[1]*_rhs[0]
        );
}

template<class P> requires(is_vector_type<P>::value)
inline P filled(typename Traits<P>::value_type _val)
{
    P res;
    for (int i = 0; i < Traits<P>::dim; ++i) {
        res[i] = _val;
    };
    return res;
}

template<class P> requires(is_vector_type<P>::value)
inline P abs(const P& _p)
{
    P res;
    for (int i = 0; i < Traits<P>::dim; ++i) {
        res[i] = std::abs(_p[i]);
    };
    return res;
}

template<class P> requires(is_vector_type<P>::value)
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

template<class P> requires(is_vector_type<P>::value)
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
