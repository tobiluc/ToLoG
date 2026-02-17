#pragma once

#include <ToLoG/io/happly.hpp>
#include <ToLoG/utils/indices.hpp>
#include <filesystem>
#include <ToLoG/mesh/polygon_mesh_concepts.hpp>

namespace ToLoG::IO
{

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
