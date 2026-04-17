#pragma once

#include <ToLoG/tree/AABBTree.hpp>
#include <ToLoG/utils/indices.hpp>
#include <ToLoG/mesh/mesh_data.hpp>

namespace ToLoG
{

template<vector_type P, typename T>
requires(Traits<P>::dim==3 && std::is_same_v<P,typename Traits<T>::vector_type>)
QuadMesh<P> aabbtree_to_polygon_mesh(const AABBTree<T>& _tree)
{
    QuadMesh<P> mesh;
    for (uint32_t i = 0; i < _tree.n_nodes(); ++i) {
        const auto& corners = _tree.node_aabb(i).corners();
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
