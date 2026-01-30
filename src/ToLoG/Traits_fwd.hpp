#pragma once
#include <type_traits>

namespace ToLoG
{

template<typename T>
struct Traits {};

template<typename T>
struct is_vector_type : std::false_type {};

}
