#pragma once
#include "ToLoG/Core.hpp"
#include "ToLoG/HashMap.hpp"
#include <ToLoG/Traits_fwd.hpp>
#include <ToLoG/utils/indices.hpp>
#include <vector>
#include <ToLoG/mesh/polygon_mesh_concepts.hpp>
#include <ToLoG/utils/std_pair_hash.hpp>

namespace ToLoG
{

template<vector P>
class PolygonMesh
{
private:
    using vertex_index = Traits<PolygonMesh<P>>::vertex_index;
    using edge_index = Traits<PolygonMesh<P>>::edge_index;
    using face_index = Traits<PolygonMesh<P>>::face_index;
    using FT = Traits<P>::value_type;

    struct Vertex {
    private:
        P position_;
    public:
        Vertex(const P& _position) : position_(_position) {}
        Vertex() {}
        inline const P& position() const {return position_;}
    };
    struct Face {
    private:
        std::vector<vertex_index> vertices_;
    public:
        Face(const std::vector<vertex_index>& _vertices) : vertices_(_vertices) {}
        Face() {}
        inline const std::vector<vertex_index>& vertices() const {return vertices_;}
        inline size_t valence() const {return vertices_.size();}
    };
    struct Edge {
    private:
        std::array<vertex_index,2> vertices_;
    public:
        Edge(vertex_index _v0, vertex_index _v1) : vertices_({_v0, _v1}) {}
        inline vertex_index vertex(int _i) const {return vertices_[_i];}
    };

public:
    class VertexVertexIterator {
    private:
        const PolygonMesh* mesh_;
        vertex_index vh_;
        size_t i_;
    public:
        VertexVertexIterator(const PolygonMesh* _mesh,
            vertex_index _vh, size_t _i)
            : mesh_(_mesh), vh_(_vh), i_(_i) {}
        vertex_index operator*() const {
            const edge_index eh = mesh_->v_e_map_[index(vh_)][i_];
            const Edge& e = mesh_->edge(eh);
            vertex_index v0 = e.vertex(0);
            vertex_index v1 = e.vertex(1);
            return (v0 == vh_) ? v1 : v0;
        }
        VertexVertexIterator& operator++() {
            ++i_;
            return *this;
        }
        bool operator==(const VertexVertexIterator& _o) const {
            return i_ == _o.i_ &&
                   vh_ == _o.vh_ &&
                   mesh_ == _o.mesh_;
        }
        bool operator!=(const VertexVertexIterator& _o) const {
            return !(*this == _o);
        }
        bool is_valid() const {
            return i_ < mesh_->v_e_map_[index(vh_)].size();
        }
    };

public:
    PolygonMesh() {}
    inline vertex_index add_vertex(const P& _p) {
        vertices_.emplace_back(_p);
        v_e_map_.emplace_back();
        return vertex_index(vertices_.size()-1);
    }
    inline edge_index add_edge(vertex_index _v0, vertex_index _v1) {
        // Resize edge map if containing not yet existing vertex
        if (v_e_map_.size() <= std::max(index(_v0), index(_v1))) {
            v_e_map_.resize(std::max(index(_v0), index(_v1))+1);
        }
        // Look for edge in existing map
        for (edge_index eh : v_e_map_[index(_v0)]) {
            const Edge& e = edge(eh);
            if ((e.vertex(0)==_v0 && e.vertex(1)==_v1) ||
                (e.vertex(1)==_v0 && e.vertex(0)==_v1)) {
                return eh;
            }
        }
        edges_.emplace_back(_v0, _v1);
        edge_index eh(edges_.size()-1);
        v_e_map_[index(_v0)].push_back(eh);
        v_e_map_[index(_v1)].push_back(eh);
        return eh;
    }
    inline face_index add_face(const std::vector<vertex_index>& _f) {
        faces_.emplace_back(_f);
        for (int i = 0; i < _f.size()-1; ++i) {add_edge(_f[i], _f[i+1]);}
        return face_index(faces_.size()-1);
    }
    inline size_t n_vertices() const {
        return vertices_.size();
    }
    inline size_t n_edges() const {
        return edges_.size();
    }
    inline size_t n_faces() const {
        return faces_.size();
    }
    inline const P& point(vertex_index _vi) const {
        return vertices_[_vi].position();
    }
    inline const Vertex& vertex(vertex_index _vi) const {
        return vertices_[index(_vi)];
    }
    inline const Edge& edge(edge_index _ei) const {
        return edges_[index(_ei)];
    }
    inline const Face& face(face_index _fi) const {
        return faces_[index(_fi)];
    }
    inline const std::vector<Vertex>& vertices() const {
        return vertices_;
    }
    inline const std::vector<Edge>& edges() const {
        return edges_;
    }
    inline const std::vector<Face>& faces() const {
        return faces_;
    }
    inline void clear() {
        vertices_.clear();
        edges_.clear();
        faces_.clear();
    }
    inline VertexVertexIterator vv_iter(vertex_index _vi) const {
        return VertexVertexIterator(this, _vi, 0);
    }
private:
    std::vector<Vertex> vertices_;
    std::vector<Edge> edges_;
    std::vector<Face> faces_;
    std::vector<std::vector<edge_index>> v_e_map_;
};

template<vector P>
struct Traits<PolygonMesh<P>>
{
    using vertex_index = int;
    using edge_index = int;
    using face_index = int;

    using vector_type = P;
};

template<polygon_mesh M>
M triangulated_faces(const M& _mesh)
{
    using vertex_index = typename Traits<M>::vertex_index;
    M m;
    // Add Vertices
    for (int i = 0; i < _mesh.n_vertices(); ++i) {
        m.add_vertex(_mesh.point(vertex_index(i)));
    }
    // Add triangulated faces
    for (const auto& f : _mesh.faces()) {
        std::vector<vertex_index> vs;
        vs.reserve(f.valence());
        for (const auto& v : f.vertices()) {vs.push_back(v);}
        for (int i = 2; i < f.valence(); ++i) {
            m.add_face(std::vector<vertex_index>{
                vs[0],
                vs[i-1],
                vs[i]
            });
        }
    }
    return m;
}

template<polygon_mesh_3 M,
    typename FT = typename Traits<typename Traits<M>::vector_type>::value_type>
M cube(FT _size=1)
{
    using P = typename Traits<M>::vector_type;
    using vertex_index = typename Traits<M>::vertex_index;
    using Face = std::vector<vertex_index>;

    M m;

    FT V = _size/static_cast<FT>(2);
    std::array<vertex_index,8> vs = {
        m.add_vertex(P(-V,-V,-V)),
        m.add_vertex(P(-V,-V,+V)),
        m.add_vertex(P(-V,+V,-V)),
        m.add_vertex(P(-V,+V,+V)),
        m.add_vertex(P(+V,-V,-V)),
        m.add_vertex(P(+V,-V,+V)),
        m.add_vertex(P(+V,+V,-V)),
        m.add_vertex(P(+V,+V,+V))
    };
    for (const auto& f : cube_vertex_indices) {
        m.add_face(Face{vs[f[0]],vs[f[1]],vs[f[2]],vs[f[3]]});
    }
    return m;
}

template<polygon_mesh_3 M,
    typename FT = typename Traits<typename Traits<M>::vector_type>::value_type>
M cylinder(std::function<FT(FT)> _rt = [](FT t){return FT(0.5);} , FT height=2,
    int r_slices = 32, int h_slices = 1)
{
    using P = typename Traits<M>::vector_type;
    using vertex_index = typename Traits<M>::vertex_index;
    using Face = std::vector<vertex_index>;

    M m;

    // rings[k][i] = vertex at height slice k, radial slice i
    std::vector<Face> rings(h_slices + 1);

    for (int k = 0; k <= h_slices; ++k) {
        FT t = FT(k) / h_slices;
        FT z = t * height;
        FT r = _rt(t);

        rings[k].reserve(r_slices);

        for (int i = 0; i < r_slices; ++i) {
            FT a = 2.0 * M_PI * i / r_slices;
            FT x = r * std::cos(a);
            FT y = r * std::sin(a);

            rings[k].push_back(m.add_vertex(P(x, y, z)));
        }
    }

    // Side Faces (Quads)
    for (int k = 0; k < h_slices; ++k) {
        for (int i = 0; i < r_slices; ++i) {
            int j = (i + 1) % r_slices;
            m.add_face(Face{
                rings[k][i],
                rings[k][j],
                rings[k+1][j],
                rings[k+1][i]
            });
        }
    }

    // Bottom Cap
    auto cb = m.add_vertex(P(0, 0, 0));
    for (int i = 0; i < r_slices; ++i) {
        int j = (i + 1) % r_slices;
        m.add_face(Face{cb, rings[0][j], rings[0][i]});
    }

    // Top Cap
    auto ct = m.add_vertex(P(0, 0, height));
    for (int i = 0; i < r_slices; ++i) {
        int j = (i + 1) % r_slices;
        m.add_face(Face{ct, rings[h_slices][i], rings[h_slices][j]});
    }

    return m;
}

template<polygon_mesh Mesh>
inline AABB<typename Traits<Mesh>::vector_type> aabb(const Mesh& _mesh)
{
    AABB<typename Traits<Mesh>::vector_type> bbox;
    for (uint32_t i = 0; i < _mesh.n_vertices(); ++i) {
        bbox.expand(_mesh.point(typename Traits<Mesh>::vertex_index(i)));
    }
    return bbox;
}

}
