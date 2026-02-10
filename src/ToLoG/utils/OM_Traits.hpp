// #pragma once
// #include <ToLoG/Traits_fwd.hpp>
// #include <OpenMesh/Core/Geometry/Vector11T.hh>
// #include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

// namespace ToLoG
// {

// template<typename FT, int DIM>
// struct is_vector_type<OpenMesh::VectorT<FT,DIM>> : std::true_type {};

// template<typename FT, int DIM>
// struct Traits<OpenMesh::VectorT<FT,DIM>>
// {
//     using value_type = FT;
//     using vector_type = OpenMesh::VectorT<FT,DIM>;
//     constexpr static int dim = DIM;
// };

// template<typename T>
// struct Traits<OpenMesh::PolyMesh_ArrayKernelT<T>>
// {
//     using vertex_index = OpenMesh::SmartVertexHandle;
//     using edge_index = OpenMesh::SmartEdgeHandle;
//     using face_index = OpenMesh::SmartFaceHandle;
//     using vector_type = T::Point;
// };

// }
