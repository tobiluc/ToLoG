#pragma once
#include <type_traits>
#include <concepts>
#include <vector>

namespace ToLoG
{

template<typename T>
struct Traits {};

template<typename T>
struct is_vector_type : std::false_type {};

template<typename P, int k>
concept vector_kd =
    is_vector_type<P>::value &&
    Traits<P>::dim == k &&
    std::is_same_v<typename Traits<P>::value_type, double> &&
    requires(const P& p) {
        {p.data()} -> std::same_as<const typename Traits<P>::value_type*>;
    };
;

template<typename P>
concept vector_3d = vector_kd<P,3>;

template<typename P>
concept vector_2d = vector_kd<P,2>;

}
