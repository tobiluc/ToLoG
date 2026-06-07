#pragma once

#include <ToLoG/mesh/CellComplex.hpp>

namespace ToLoG::Mesh
{

enum class TriangulationStrategy {
    BARY, // insert new vertex at barycenter
    FAN
};

template<vector Point, typename TopoKernel>
void triangulate_faces(GeometricCellComplex<Point,TopoKernel>& _mesh,
                       const TriangulationStrategy _strat = TriangulationStrategy::BARY)
{
    if (_mesh.has_deleted()) {
        throw std::runtime_error("triangulation does not currently work with deleted entities.");
    }

    GeometricCellComplex<Point,TopoKernel> tri_mesh;
    tri_mesh.reserve_vertices(_mesh.num_allocated_vertices());
    tri_mesh.reserve_edges(_mesh.num_allocated_edges());
    tri_mesh.reserve_faces(_mesh.num_allocated_faces());
    for (const auto& p : _mesh.points()) {
        tri_mesh.add_vertex(p);
    }
    if (_strat == TriangulationStrategy::BARY)
    {
        for (FH fh : _mesh.faces()) {
            VH v0 = tri_mesh.add_vertex(_mesh.barycenter(fh));
            const auto& f = _mesh.face_vertices(fh);
            for (int i = 0; i < f.size(); ++i) {
                tri_mesh.add_face({
                    v0,
                    f[i],
                    f[(i+1)%f.size()]
                });
            }
        }
    }
    else if (_strat == TriangulationStrategy::FAN)
    {
        for (FH fh : _mesh.faces()) {
            const auto& f = _mesh.face_vertices(fh);
            for (int i = 1; i < f.size()-1; ++i) {
                tri_mesh.add_face({
                    f[0],
                    f[i],
                    f[i+1]
                });
            }
        }
    }
    _mesh = std::move(tri_mesh);
}

}
