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
struct is_vector_type<OpenVolumeMesh::VectorT<FT,DIM>> : std::true_type {};

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
};

class OVMTetTopology : public TetTopologyT<
    OpenVolumeMesh::VertexHandle,
    OpenVolumeMesh::HalfEdgeHandle,
    OpenVolumeMesh::HalfFaceHandle>
{
public:
    template<typename P>
    OVMTetTopology(const OpenVolumeMesh::TetrahedralGeometryKernel<P, OpenVolumeMesh::TetrahedralMeshTopologyKernel>& _mesh,
                 const OpenVolumeMesh::CellHandle& _ch)
    {
        using TT = TetTopology;
        int idx(0);
        for (OpenVolumeMesh::VertexHandle vh : _mesh.cell_vertices(_ch)) {
            v_[idx] = vh;
            hf_[idx++] = _mesh.vertex_opposite_halfface(_ch, vh);
        }
        for (int i = 0; i < 4; ++i) {
            for (int j = i+1; j < 4; ++j) {
                TT::V v0 = TT::v(i);
                TT::V v1 = TT::v(j);
                TT::HE he = TT::he(v0,v1);
                OpenVolumeMesh::HEH heh = _mesh.find_halfedge_in_cell(vertex(v0), vertex(v1), _ch);
                he_[TT::i(he)] = heh;
                he_[TT::i(TT::opp(he))] = heh.opposite_handle();
            }
        }
    }
};

}
