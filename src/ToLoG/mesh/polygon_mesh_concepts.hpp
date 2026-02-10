#pragma once
#include <ToLoG/Core.hpp>
#include <ToLoG/mesh/mesh_concepts.hpp>

namespace ToLoG
{

// A Polygon Mesh Face f is an object that contains two functions.
// f.vertices() returns a range of vertices (templated)
// f.valence() returns the valence of the face
template<class Face, class VertexIndex>
concept polygon_mesh_face =
    requires(const Face& f) {
        {f.vertices()} -> std::ranges::range;
        requires std::convertible_to<
            std::ranges::range_reference_t<decltype(f.vertices())>,
            VertexIndex>;
        {f.valence()} -> std::convertible_to<size_t>;
    };

// Works with ToLoG::PolygonMesh3 and
template<class M>
concept polygon_mesh =
    mesh_index<typename Traits<M>::vertex_index> &&
    requires(
        M& rm, // ref
        const M& crm, // const ref
        typename Traits<M>::vector_type point,
        typename Traits<M>::vertex_index vertex_index,
        typename Traits<M>::face_index face_index,
        std::vector<typename Traits<M>::vertex_index> vertex_indices
    ) {
    {rm.add_vertex(point)} -> std::same_as<typename Traits<M>::vertex_index>;
    {rm.add_edge(vertex_index, vertex_index)} -> std::same_as<typename Traits<M>::edge_index>;
    {rm.add_face(vertex_indices)} -> std::same_as<typename Traits<M>::face_index>;

    {crm.n_vertices()} -> std::same_as<size_t>;
    {crm.n_edges()} -> std::same_as<size_t>;
    {crm.n_faces()} -> std::same_as<size_t>;

    {crm.point(vertex_index)} -> std::same_as<const typename Traits<M>::vector_type&>;

    {crm.faces()} -> std::ranges::range;
    requires polygon_mesh_face<
        std::ranges::range_reference_t<decltype(crm.faces())>,
        typename Traits<M>::vertex_index>;

    {rm.clear()};
};

template<class M>
concept polygon_mesh_3 = polygon_mesh<M>
    && Traits<typename Traits<M>::vector_type>::dim==3;

}
