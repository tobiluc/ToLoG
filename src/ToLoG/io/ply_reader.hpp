#pragma once
#include <ToLoG/io/happly.hpp>
#include <ToLoG/mesh/polygon_mesh_concepts.hpp>
#include <filesystem>

namespace ToLoG::IO
{

template<polygon_mesh_3 Mesh>
int read_polygon_mesh_ply(
    const std::filesystem::path& _path,
    Mesh& _mesh)
{
    using P = typename Traits<Mesh>::vector_type;
    using FT = typename Traits<P>::value_type;
    using vertex_index = typename Traits<Mesh>::vertex_index;
    using face_index = typename Traits<Mesh>::face_index;

    happly::PLYData ply(_path);
    std::vector<vertex_index> vhs;

    if (!ply.hasElement("vertex")) {return 0;}
    for (const auto& p : ply.getVertexPositions()) {
        vhs.push_back(_mesh.add_vertex(P(p[0],p[1],p[2])));
    }

    if (!ply.hasElement("face")) {return 0;}
    for (const auto& f : ply.getFaceIndices()) {
        std::vector<vertex_index> f_vhs;
        for (auto i : f) {f_vhs.push_back(vhs[i]);}
        _mesh.add_face(f_vhs);
    }

    return 0;
}

}
