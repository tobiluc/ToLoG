#pragma once
#include <concepts>

namespace ToLoG
{

template<typename T>
struct Traits {};

template<typename V, typename FT>
concept vector_type_t =
    requires(const V& p, const V& q, V& rp, int i, FT ft) {
        {p.operator[](i)} -> std::convertible_to<FT>;
        {rp.operator[](i)} -> std::same_as<FT&>;
        {p+q} -> std::convertible_to<V>;
        {p-q} -> std::convertible_to<V>;
        {p*ft} -> std::convertible_to<V>;
        {p/ft} -> std::convertible_to<V>;
    };
;

template<typename V>
concept vector_type = vector_type_t<V, typename Traits<V>::value_type>;

template<typename P, int k, typename FT>
concept vector_kt = vector_type_t<P,FT> && Traits<P>::dim == k;

template<typename P>
concept vector_3d = vector_kt<P,3,double>;

template<typename P>
concept vector_2d = vector_kt<P,2,double>;

}
