#pragma once
#include <ToLoG/Traits_fwd.hpp>
#include <OpenVolumeMesh/Geometry/Vector11T.hh>
#include <OpenVolumeMesh/Mesh/PolyhedralMesh.hh>

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

}
