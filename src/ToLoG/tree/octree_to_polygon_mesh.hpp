#pragma once

#include "ToLoG/mesh/polygon_mesh.hpp"
#include "ToLoG/tree/Octree.hpp"

namespace ToLoG
{

template<polygon_mesh_3 Mesh>
PolygonMesh<Point<float,3>> octree_to_polygon_mesh(const Octree& _tree)
{
    Mesh mesh;
    for (uint32_t i = 0; i < _tree.n_nodes(); ++i) {
        const auto& corners = _tree.node_aabb(_tree.node_coords(i)).corners();
        std::vector<typename Traits<Mesh>::vertex_index> vhs;
        for (const auto& corner : corners) {
            vhs.push_back(mesh.add_vertex(corner));
        }
        for (const auto& f : ToLoG::cube_vertex_indices) {
            mesh.add_face(std::vector<typename Traits<Mesh>::vertex_index>{
                vhs[f[0]], vhs[f[1]], vhs[f[2]], vhs[f[3]]
            });
        }
    }
    return mesh;
}

}
