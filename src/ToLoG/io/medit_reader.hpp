#pragma once

#include <filesystem>
#include <iostream>
#include <ToLoG/mesh/polyhedral_mesh_concepts.hpp>
#include <fstream>

namespace ToLoG::IO
{

template<polyhedral_mesh_3 Mesh>
int read_polyhedral_mesh_medit(const std::filesystem::path& _path, Mesh& _mesh)
{
    using P = Traits<Mesh>::vector_type;
    using VI = Traits<Mesh>::vertex_index;

    std::ifstream in(_path);
    if (!in) {
        std::cerr << "Error: Cannot open file " << _path << std::endl;
        return 1;
    }

    std::string keyword;

    while (in >> keyword) {
        if (keyword == "MeshVersionFormatted") {
            int version;
            in >> version;
        }
        else if (keyword == "Dimension") {
            int dim;
            in >> dim;
            if (dim != 3) {
                std::cerr << "Dimension != 3" << std::endl;
                return 1;
            }
        }
        else if (keyword == "Vertices") {
            size_t num_vertices;
            in >> num_vertices;
            for (int i = 0; i < num_vertices; ++i) {
                double x, y, z;
                int region;
                in >> x >> y >> z >> region;
                _mesh.add_vertex(P(x, y, z));
            }
        }
        else if (keyword == "Triangles") {
            size_t num_triangles;
            in >> num_triangles;
            for (int i = 0; i < num_triangles; ++i) {
                std::vector<VI> vhs;
                for (int j = 0; j < 3; ++j) {
                    int tmp = 0;
                    in >> tmp;
                    vhs.push_back(VI(tmp-1));
                }
                int region;
                in >> region;
                _mesh.add_face(vhs);
            }
        }
        else if (keyword == "Tetrahedra") {
            size_t num_tets;
            in >> num_tets;
            for (int i = 0; i < num_tets; ++i) {
                std::vector<VI> vhs;
                for (int j = 0; j < 4; ++j) {
                    int tmp = 0;
                    in >> tmp;
                    vhs.push_back(VI(tmp-1));
                }
                int region;
                in >> region;

                // Add Tet Cell
                _mesh.add_cell(vhs);
            }
        }
        else if (keyword == "Hexahedra") {
            size_t num_hexes;
            in >> num_hexes;
            for (int i = 0; i < num_hexes; ++i) {
                std::vector<VI> vhs;
                for (int j = 0; j < 8; ++j) {
                    int v;
                    in >> v;
                    vhs.push_back(VI(v-1));
                }
                int region;
                in >> region;

                // Add Hex Cell
                _mesh.add_cell(vhs);
            }
        }
        else if (keyword == "#") {
            std::string comment;
            std::getline(in, comment);
        }
        else if (keyword == "End") {
            std::string dummy;
            std::getline(in, dummy);
        }
        else {
            std::cerr << "Warning: Unknown Keyword '" << keyword << std::endl;
            std::string dummy;
            std::getline(in, dummy);
        }
    }

    in.close();
    return 0;
}

}
