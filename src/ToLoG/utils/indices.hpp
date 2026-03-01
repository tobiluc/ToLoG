#pragma once
#include <array>

namespace ToLoG
{

static const std::array<std::array<int,4>,6> cube_vertex_indices = {{
    {0,1,3,2},{4,6,7,5},{0,4,5,1},{2,3,7,6},{0,2,6,4},{1,5,7,3}
}};

static const std::array<std::array<int,3>,4> tet_vertex_indices_ccw = {{
    {0,1,2},{0,3,1},{0,2,3},{1,3,2}
}};

static const std::array<std::array<int,3>,4> tet_vertex_indices_cw = {{
    {0,2,1},{0,1,3},{0,3,2},{1,2,3}
}};

}
