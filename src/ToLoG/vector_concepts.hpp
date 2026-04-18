#pragma once
#include <concepts>
#include <ToLoG/Traits_fwd.hpp>

namespace ToLoG
{

template<typename V, typename FT>
concept vector_of_type =
    requires(const V& p, const V& q, V& rp, int i, FT ft) {
        {p[i]} -> std::convertible_to<FT>;
        {rp[i]} -> std::same_as<FT&>;
        {p+q} -> std::convertible_to<V>;
        {p-q} -> std::convertible_to<V>;
        {p*ft} -> std::convertible_to<V>;
        {p/ft} -> std::convertible_to<V>;
    };
;

template<typename V>
concept vector =
    requires(V& rp, int i) {
        requires vector_of_type<
            V,
            std::remove_reference_t<decltype(rp[i])>
            >;
    };
;

template<typename P, int DIM>
concept vector_of_dim = vector<P> && Traits<P>::dim == DIM;

template<typename P, typename FT, int DIM>
concept vector_of_type_and_dim = vector_of_type<P,FT> && vector_of_dim<P,DIM>;

template<typename P>
concept vector_3_double = vector_of_type_and_dim<P,double,3>;

template<typename P>
concept vector_2_double = vector_of_type_and_dim<P,double,2>;

}
