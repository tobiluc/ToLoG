#pragma once
#include <concepts>

namespace ToLoG
{

template<typename T>
struct Traits {};

template<typename V, typename FT>
concept vector_of =
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
concept vector = vector_of<V, typename Traits<V>::value_type>;

template<typename P>
concept vector_3d = vector_of<P,double> && Traits<P>::dim == 3;

template<typename P>
concept vector_2d = vector_of<P,double> && Traits<P>::dim == 2;

}
