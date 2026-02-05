#pragma once
#include <ToLoG/Traits_fwd.hpp>
#include <Eigen/Core>

namespace ToLoG
{

template<typename FT, int DIM>
struct is_vector_type<Eigen::Vector<FT,DIM>> : std::true_type {};

template<typename FT, int DIM>
struct Traits<Eigen::Vector<FT,DIM>>
{
    using value_type = FT;
    using vector_type = Eigen::Vector<FT,DIM>;
    constexpr static int dim = DIM;
};

}
