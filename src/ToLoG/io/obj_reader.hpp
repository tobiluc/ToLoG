#pragma once
#include <ToLoG/mesh/polygon_mesh_concepts.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace ToLoG::IO
{

template<polygon_mesh_3 Mesh>
int read_polygon_mesh_obj(const std::filesystem::path& _path,
    Mesh& _mesh)
{
    using P = typename Traits<Mesh>::vector_type;
    using FT = typename Traits<P>::value_type;
    using vertex_index = typename Traits<Mesh>::vertex_index;
    using face_index = typename Traits<Mesh>::face_index;

    std::ifstream file(_path);
    if (!file) {return 1;}

    std::vector<vertex_index> vertices;
    std::string line;

    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
            continue;

        std::istringstream iss(line);
        std::string tag;
        iss >> tag;

        // Vertex
        if (tag == "v") {
            FT x, y, z;
            iss >> x >> y >> z;

            vertex_index v = _mesh.add_vertex(P(x, y, z));
            vertices.push_back(v);
        }
        // Line
        else if (tag == "l")
        {
            int idx;
            iss >> idx;
            vertex_index v0 = vertices[idx-1];
            iss >> idx;
            vertex_index v1 = vertices[idx-1];
            // edge?
        }
        // Face
        else if (tag == "f")
        {
            std::vector<vertex_index> face;
            std::string token;
            while (iss >> token)
            {
                // "1", "1/2", "1/2/3", "1//3"
                // ignore vt and vn for now)
                std::istringstream t(token);
                int idx;
                t >> idx;

                // OBJ is 1-based
                face.push_back(vertices[idx-1]);
            }

            if (face.size() >= 3) {
                _mesh.add_face(face);
            } else {
                std::cerr << "Warning: Ignore face with valence "
                          << face.size() << " in OBJ reader" << std::endl;
            }
        }
    }

    return 0;
}

}
