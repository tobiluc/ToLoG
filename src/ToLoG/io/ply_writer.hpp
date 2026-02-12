#pragma once

#include <ToLoG/io/happly.hpp>
#include <ToLoG/utils/indices.hpp>
#include <ToLoG/AABBTree.hpp>
#include <filesystem>
#include <ToLoG/mesh/polygon_mesh_concepts.hpp>

namespace ToLoG::IO
{

template<typename T>
int write_aabb_tree_ply(const std::filesystem::path& _path, const AABBTree<T>& _tree, const bool _include_only_leafs=true)
{
    static_assert(AABBTree<T>::DIM==3);

    // Prepare data vectors
    std::vector<std::array<typename AABBTree<T>::FT, 3>> v_positions;
    v_positions.reserve(8*_tree.n_nodes());
    double max_node_size = 0.0;
    std::vector<double> node_sizes;
    node_sizes.reserve(_tree.n_nodes());
    std::vector<std::vector<size_t>> f_indices;
    f_indices.reserve(6*_tree.n_nodes());

    // Collect data
    unsigned int idx(0);
    for (uint32_t node_i = 0; node_i < _tree.n_nodes(); ++node_i)
    {
        // Skip inner node if desired
        if (_include_only_leafs && !_tree.is_leaf_node(node_i)) {continue;}

        // Get node size
        const auto& aabb = _tree.node_aabb(node_i);
        double node_size = static_cast<double>((aabb.max()-aabb.min()).norm());
        node_sizes.push_back(node_size);
        if (max_node_size < node_size) {max_node_size = node_size;}

        // Add Vertices
        for (const auto& v : aabb.corners()) {
            v_positions.push_back({v[0],v[1],v[2]});
        }
        // Add Faces
        for (const auto& f : cube_vertex_indices) {
            f_indices.push_back({idx+f[0],idx+f[1],idx+f[2],idx+f[3]});
        }
        idx += 8;
    }

    // Add vertex colors based on node size
    std::vector<std::array<double,3>> v_colors;
    v_colors.reserve(v_positions.size());
    for (unsigned int i = 0; i < node_sizes.size(); ++i) {
        double t = node_sizes[i]/max_node_size;
        for (unsigned int j = 0; j < 8; ++j) {
            v_colors.push_back({t,0.0,1.0-t});
        }
    }

    // Let happly handle the rest :D
    happly::PLYData out;
    out.addVertexPositions(v_positions);
    out.addVertexColors(v_colors);
    out.addFaceIndices(f_indices);
    out.write(_path, happly::DataFormat::ASCII);
    return 0;
}

template<vector_type P, typename T>
requires(Traits<P>::dim==2||Traits<P>::dim==3)
int write_triangles_ply(const std::filesystem::path& _path,
                        const std::vector<P>& _vertices,
                        const std::vector<T>& _triangles)
{
    static constexpr int DIM = Traits<P>::dim;

    std::vector<std::array<double, 3>> v_positions;
    v_positions.reserve(_vertices.size());
    for (const P& p : _vertices) {
        if constexpr(DIM == 2) {v_positions.push_back({p[0], p[1], 0.0});}
        else if constexpr(DIM == 3) {v_positions.push_back({p[0], p[1], p[2]});}
    }

    std::vector<std::vector<size_t>> f_indices;
    f_indices.reserve(_triangles.size());
    for (const T& t : _triangles) {
        f_indices.push_back({t[0],t[1],t[2]});
    }

    happly::PLYData out;
    out.addVertexPositions(v_positions);
    out.addFaceIndices(f_indices);
    out.write(_path, happly::DataFormat::ASCII);
    return 0;
}

template<polygon_mesh_3 Mesh>
int write_polygon_mesh_ply(
    const std::filesystem::path& _path,
    const Mesh& _mesh)
{
    using P = typename Traits<Mesh>::vector_type;
    using FT = typename Traits<P>::value_type;
    using vertex_index = typename Traits<Mesh>::vertex_index;
    using face_index = typename Traits<Mesh>::face_index;

    std::vector<std::array<double, 3>> v_positions;
    for (int i = 0; i < _mesh.n_vertices; ++i) {
        const auto& p = _mesh.point(vertex_index(i));
        v_positions.push_back({
            static_cast<double>(p[0]),
            static_cast<double>(p[1]),
            static_cast<double>(p[2])
        });
    }

    std::vector<std::vector<size_t>> f_indices;
    for (const auto& f : _mesh.faces()) {
        std::vector<size_t> is;
        for (const auto& i : f.vertices()) {
            is.push_back(index(i));
        }
        f_indices.push_back(is);
    }

    happly::PLYData out;
    out.addVertexPositions(v_positions);
    out.addFaceIndices(f_indices);
    out.write(_path, happly::DataFormat::ASCII);
    return 0;
}

}
