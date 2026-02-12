#pragma once
#include <concepts>

namespace ToLoG
{

template<typename T>
struct Traits {};

template<typename V, typename FT>
concept vector_type_t =
    requires(const V& p, V& rp, int i, FT ft) {
        {p.operator[](i)} -> std::convertible_to<FT>;
        {rp.operator[](i)} -> std::same_as<FT&>;
        {p.operator+(p)} -> std::convertible_to<V>;
        {p.operator-(p)} -> std::convertible_to<V>;
        {p.operator*(ft)} -> std::convertible_to<V>;
        {p.operator/(ft)} -> std::convertible_to<V>;
    };
;

template<typename V>
concept vector_type = vector_type_t<V, typename Traits<V>::value_type>;

template<typename P, int k>
concept vector_kd = vector_type_t<P, double> && Traits<P>::dim == k;

template<typename P>
concept vector_3d = vector_kd<P,3>;

template<typename P>
concept vector_2d = vector_kd<P,2>;

}
