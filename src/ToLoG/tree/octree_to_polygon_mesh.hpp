#pragma once

#include <ToLoG/utils/indices.hpp>
#include <ToLoG/mesh/mesh_data.hpp>
#include <ToLoG/tree/Octree.hpp>

namespace ToLoG
{

template<vector_type P>
requires(Traits<P>::dim==3)
QuadMesh<P> octree_to_polygon_mesh(const Octree& _tree)
{
    QuadMesh<P> mesh;
    for (uint32_t i = 0; i < _tree.n_nodes(); ++i) {
        const auto& corners = _tree.node_aabb(_tree.node_coords(i)).corners();
        std::vector<uint32_t> vhs;
        for (const auto& corner : corners) {
            uint32_t vh = mesh.positions.size();
            mesh.positions.push_back(corner);
            vhs.push_back(vh);
        }
        for (const auto& f : ToLoG::cube_vertex_indices) {
            mesh.quads.push_back({vhs[f[0]], vhs[f[1]], vhs[f[2]], vhs[f[3]]});
        }
    }
    return mesh;
}

}
