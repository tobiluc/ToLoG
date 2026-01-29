#pragma once
#include <array>

namespace ToLoG
{

static const std::array<std::array<unsigned int,4>,6> cube_vertex_indices = {{
    {0,1,3,2},{4,6,7,5},{0,4,5,1},{2,3,7,6},{0,2,6,4},{1,5,7,3}
}};

static const std::array<std::array<unsigned int,3>,4> tetrahedron_vertex_indices = {{
    {0,1,2},{0,3,1},{1,3,2},{0,2,3}
}};

}
