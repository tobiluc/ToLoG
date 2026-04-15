#include <ToLoG/utils/indices.hpp>
#include <filesystem>
#include <fstream>
#include <ToLoG/mesh/polygon_mesh_concepts.hpp>

namespace ToLoG::IO
{

template<polygon_mesh_3 Mesh>
[[deprecated("use rapidobj")]]
int write_polygon_mesh_obj(const std::filesystem::path& _path,
    const Mesh& _mesh)
{
    using P = typename Traits<Mesh>::vector_type;
    using vertex_index = typename Traits<Mesh>::vertex_index;
    using face_index = typename Traits<Mesh>::face_index;

    std::ofstream file(_path);
    file << "# https://github.com/tobiluc/ToLoG" << std::endl;
    for (int i = 0; i < _mesh.n_vertices(); ++i) {
        // Vertex Position
        const auto& p = _mesh.point(vertex_index(i));
        file << "v " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }
    size_t n_idx(1);
    for (const auto& f : _mesh.faces()) {
        std::vector<vertex_index> vs;
        vs.reserve(f.valence());
        for (const auto& v : f.vertices()) {vs.push_back(v);}

        // Flat Normal
        P n = normal(Triangle<P>(
            _mesh.point(vs[0]),
            _mesh.point(vs[1]),
            _mesh.point(vs[2])));
        file << "vn " << n[0] << " " << n[1] << " " << n[2] << std::endl;

        // Face Vertex Indices
        file << "f ";
        for (const auto& vi : vs) {
            file << (index(vi)+1) << "//" << n_idx << " ";
        }
        file << std::endl;

        ++n_idx;
    }
    file.close();

    return 0;
}

}
