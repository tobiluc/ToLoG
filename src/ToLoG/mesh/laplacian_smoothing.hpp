#pragma once

#include <ToLoG/mesh/CellComplex.hpp>

namespace ToLoG::Mesh
{

template<vector Point, typename TopoKernel>
void laplacian_smoothing(GeometricCellComplex<Point, TopoKernel>& _mesh,
                         int _smoothing_iters = 1,
                         bool _skip_nonmanifold = true)
{
    if (_mesh.has_deleted()) {
        throw std::runtime_error("smoothing does not currently work with deleted entities.");
    }

    using FT = Traits<Point>::value_type;
    using M = GeometricCellComplex<Point, TopoKernel>;

    // Cache Vertex Manifoldness
    VertexPropT<uint8_t> v_manifold(_mesh.num_allocated_vertices(), false);
    for (VH vh : _mesh.vertices()) {
        v_manifold[vh] = _mesh.surface_vertex_is_manifold(vh);
    }

    for (int iter = 0; iter < _smoothing_iters; ++iter) {
        VertexPropT<Point> next_points(_mesh.num_allocated_vertices());
        for (VH vh0 : _mesh.vertices()) {
            FT n(1);
            next_points[vh0] = _mesh.point(vh0);
            if (_skip_nonmanifold && !v_manifold[vh0]) {continue;}
            for (EH eh : _mesh.vertex_edges(vh0)) {
                if (_skip_nonmanifold && !_mesh.surface_edge_is_manifold(eh)) {continue;}
                VH vh1 = _mesh.opposite_vertex(vh0, eh);
                if (_skip_nonmanifold && !v_manifold[vh1]) {continue;}
                next_points[vh0] += _mesh.point(vh1);
                ++n;
            }
            next_points[vh0] /= n;
        }
        _mesh.set_points(next_points);
    }
}

}
