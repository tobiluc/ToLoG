#pragma once

#include <ToLoG/Traits_fwd.hpp>
#include <vector>

namespace ToLoG
{

template<vector P>
struct TriangleMesh
{
    std::vector<P> positions;
    std::vector<std::array<uint32_t,3>> triangles;
};

template<vector P>
struct QuadMesh
{
    std::vector<P> positions;
    std::vector<std::array<uint32_t,4>> quads;
};

}
