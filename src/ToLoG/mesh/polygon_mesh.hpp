#pragma once
#include <ToLoG/Traits_fwd.hpp>
#include <ToLoG/utils/indices.hpp>
#include <vector>
#include <ToLoG/mesh/mesh_concepts.hpp>

namespace ToLoG
{

template<typename P>
requires(is_vector_type<P>::value && Traits<P>::dim==3)
class PolygonMesh3
{
private:
    using vertex_index = Traits<PolygonMesh3<P>>::vertex_index;
    using face_index = Traits<PolygonMesh3<P>>::face_index;
    using FT = Traits<P>::value_type;

    struct Face {
        std::vector<vertex_index> vertices_;
        inline const std::vector<vertex_index>& vertices() const {return vertices_;}
        inline size_t valence() const {return vertices_.size();}
    };
public:
    PolygonMesh3() {}
    inline vertex_index add_vertex(const P& _p) {
        points_.push_back(_p);
        return vertex_index(points_.size()-1);
    }
    inline face_index add_face(const std::vector<vertex_index>& _f) {
        faces_.push_back(Face{_f});
        return face_index(faces_.size()-1);
    }
    inline size_t n_vertices() const {
        return points_.size();
    }
    inline size_t n_faces() const {
        return faces_.size();
    }
    inline const P& point(vertex_index _vi) const {
        return points_[_vi];
    }
    inline const std::vector<Face>& faces() const {
        return faces_;
    }
    inline void clear() {
        points_.clear();
        faces_.clear();
    }
private:
    std::vector<P> points_;
    std::vector<Face> faces_;
};

template<typename P>
struct Traits<PolygonMesh3<P>>
{
    using vertex_index = int;
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

template<polygon_mesh_3 M>
M cube(typename Traits<typename Traits<M>::vector_type>::value_type _size=1)
{
    using P = typename Traits<M>::vector_type;
    using FT = typename Traits<P>::value_type;
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

template<polygon_mesh_3 M>
M cylinder(
    typename Traits<typename Traits<M>::vector_type>::value_type r1=0.5,
    typename Traits<typename Traits<M>::vector_type>::value_type r2=0.5,
    typename Traits<typename Traits<M>::vector_type>::value_type height=2,
    int r_slices = 32, int h_slices = 1)
{
    using P = typename Traits<M>::vector_type;
    using FT = typename Traits<P>::value_type;
    using vertex_index = typename Traits<M>::vertex_index;
    using Face = std::vector<vertex_index>;

    M m;

    // rings[k][i] = vertex at height slice k, radial slice i
    std::vector<Face> rings(h_slices + 1);

    for (int k = 0; k <= h_slices; ++k) {
        FT t = FT(k) / h_slices;
        FT z = t * height;
        FT r = (1.0 - t) * r1 + t * r2;

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

}
