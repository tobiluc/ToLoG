#pragma once
#include <ToLoG/Traits_fwd.hpp>
#include <OpenVolumeMesh/Core/Handles.hh>
#include <OpenVolumeMesh/Geometry/Vector11T.hh>
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>
#include <OpenVolumeMesh/Mesh/TetrahedralGeometryKernel.hh>
#include <ToLoG/mesh/tet_topology.hpp>

namespace ToLoG
{

template<typename FT, int DIM>
struct Traits<OpenVolumeMesh::VectorT<FT,DIM>>
{
    using value_type = FT;
    using vector_type = OpenVolumeMesh::VectorT<FT,DIM>;
    constexpr static int dim = DIM;
};

template<typename P>
struct Traits<OpenVolumeMesh::GeometryKernel<P, OpenVolumeMesh::TopologyKernel>>
{
    using vector_type = P;
    using vertex_index = OpenVolumeMesh::VH;
    using face_index = OpenVolumeMesh::FH;
    using cell_index = OpenVolumeMesh::CH;
};

template<typename P>
struct Traits<OpenVolumeMesh::TetrahedralGeometryKernel<P, OpenVolumeMesh::TetrahedralMeshTopologyKernel>>
{
    using vector_type = P;
    using vertex_index = OpenVolumeMesh::VH;
    using face_index = OpenVolumeMesh::FH;
    using cell_index = OpenVolumeMesh::CH;
};

class OVMTetTopology : public TetTopologyT<
    OpenVolumeMesh::VertexHandle,
    OpenVolumeMesh::HalfEdgeHandle,
    OpenVolumeMesh::HalfFaceHandle,
    OpenVolumeMesh::CellHandle>
{
private:
    template<typename P>
    using TetMesh = OpenVolumeMesh::TetrahedralGeometryKernel<P, OpenVolumeMesh::TetrahedralMeshTopologyKernel>;
public:
    template<typename P>
    OVMTetTopology(const TetMesh<P>& _mesh,
                 const OpenVolumeMesh::CellHandle& _ch)
    {
        init<P>(_mesh, _ch);
    }

    OVMTetTopology() {}

    template<typename P>
    void init(const TetMesh<P>& _mesh,
              const OpenVolumeMesh::CellHandle& _ch)
    {
        using TT = TetTopology;
        int idx(0);
        for (OpenVolumeMesh::VertexHandle vh : _mesh.get_cell_vertices(_ch)) {
            v_[idx] = vh;
            hf_[idx++] = _mesh.vertex_opposite_halfface(_ch, vh);
        }
        for (int i = 0; i < 4; ++i) {
            for (int j = i+1; j < 4; ++j) {
                TT::V v0 = TT::i2v(i);
                TT::V v1 = TT::i2v(j);
                TT::HE he = TT::he(v0,v1);
                OpenVolumeMesh::HEH heh = _mesh.find_halfedge_in_cell(vertex(v0), vertex(v1), _ch);
                he_[TT::i(he)] = heh;
                he_[TT::i(TT::opp(he))] = heh.opposite_handle();
            }
        }
        c_ = _ch;
    }

    template<typename P>
    static OpenVolumeMesh::CellPropertyT<OVMTetTopology> create_property(const TetMesh<P>& _mesh) {
        auto prop = _mesh.template create_private_cell_property<OVMTetTopology>("", OVMTetTopology());
        for (auto c_it = _mesh.c_iter(); c_it.is_valid(); ++c_it) {
            prop[*c_it] = OVMTetTopology(_mesh, *c_it);
        }
        return prop;
    }
};

}
