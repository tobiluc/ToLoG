#pragma once
#include <ToLoG/Traits_fwd.hpp>
#include <glm/glm.hpp>

namespace ToLoG
{

template<typename FT, int DIM>
struct Traits<glm::vec<DIM,FT>>
{
    using value_type = FT;
    using vector_type = glm::vec<DIM,FT>;
    constexpr static int dim = DIM;
};
}
