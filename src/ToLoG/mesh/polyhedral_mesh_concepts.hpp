// #pragma once
// #include <ToLoG/mesh/mesh_concepts.hpp>
// #include <ToLoG/Traits_fwd.hpp>

// namespace ToLoG
// {

// template<class M>
// concept polyhedral_mesh =
//     mesh_index<typename Traits<M>::vertex_index> &&
//     requires(
//         M& rm, // ref
//         const M& crm, // const ref
//         typename Traits<M>::vector_type point,
//         typename Traits<M>::vertex_index vertex_index,
//         typename Traits<M>::face_index face_index,
//         std::vector<typename Traits<M>::vertex_index> vertex_indices
//         ) {
//         {rm.add_vertex(point)} -> std::same_as<typename Traits<M>::vertex_index>;
//         {rm.add_face(vertex_indices)} -> std::same_as<typename Traits<M>::face_index>;
//         {rm.add_cell(vertex_indices)} -> std::same_as<typename Traits<M>::cell_index>;

//         {crm.n_vertices()} -> std::same_as<size_t>;
//         {crm.n_faces()} -> std::same_as<size_t>;
//         {crm.n_cells()} -> std::same_as<size_t>;

//         {crm.vertex(vertex_index)} -> std::same_as<const typename Traits<M>::vector_type&>;

//         {rm.clear()};
//     };

// template<class M>
// concept polyhedral_mesh_3 = polyhedral_mesh<M>
//     && Traits<typename Traits<M>::vector_type>::dim==3;

// }
