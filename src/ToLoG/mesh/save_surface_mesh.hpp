#pragma once

#include <ToLoG/mesh/CellComplex.hpp>
#include <filesystem>
#include <fstream>

namespace ToLoG::Mesh
{

template<vector Point, typename TopoKernel>
void save_surface_mesh(const std::filesystem::path& _path,
                       const GeometricCellComplex<Point, TopoKernel>& _mesh)
{
    if (_mesh.has_deleted()) {
        throw std::runtime_error("saving does not currently work with deleted entities.");
    }

    std::ofstream file(_path);
    for (const auto& p : _mesh.points()) {
        file << "v";
        for (int i = 0; i < Traits<Point>::dim; ++i) {
            file << " " << p[i];
        }
        file << std::endl;
    }
    for (const auto& eh : _mesh.edges()) {
        if (!_mesh.edge_is_disconnected(eh)) {continue;}
        HEH heh = eh.heh(0);
        file << "l " << (_mesh.vh0(heh).idx()+1) << " "
             << (_mesh.vh1(heh).idx()+1) << std::endl;
    }
    for (const auto& fh : _mesh.faces()) {
        file << "f";
        for (VH vh : _mesh.face_vertices(fh)) {
            file << " " << (vh.idx()+1);
        }
        file << std::endl;
    }
    file.close();
}

}
