#pragma once
#include <ToLoG/mesh/polygon_mesh_concepts.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
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

    _mesh.clear();

    std::ifstream file(_path);
    if (!file) {return 1;}

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

            _mesh.add_vertex(P(x, y, z));
        }
        // Line
        else if (tag == "l")
        {
            std::vector<vertex_index> vhs;
            std::string token;
            while (iss >> token) {
                vhs.push_back(vertex_index(std::stoi(token)-1));
            }
            for (int i = 0; i < vhs.size()-1; ++i) {
                _mesh.add_edge(vhs[i], vhs[i+1]);
            }
        }
        // Face
        else if (tag == "f")
        {
            std::vector<vertex_index> vhs;
            std::string token;
            while (iss >> token) {
                size_t pos = token.find('/');
                if (pos != std::string::npos) {
                    token = token.substr(0, pos); // Only use vertex index (no other data)
                }
                vhs.push_back(vertex_index(std::stoi(token)-1));
            }
            if (vhs.size() >= 3) {
                _mesh.add_face(vhs);
            } else {
                std::cerr << "Warning: Ignore face with valence "
                          << vhs.size() << " in OBJ reader" << std::endl;
            }
        }
    }

    return 0;
}

}
