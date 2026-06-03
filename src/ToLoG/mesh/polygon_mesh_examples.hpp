#pragma once
#include <ToLoG/mesh/PolygonMesh.hpp>

namespace ToLoG
{

// template<vector_of_dim<3> Point>
// PolygonMesh<Point> cylinder_mesh(std::function<FT(FT)> _rt = [](FT t){return FT(0.5);} , FT height=2,
//     int r_slices = 32, int h_slices = 1)
// {
//     using P = typename Traits<M>::vector_type;
//     using vertex_index = typename Traits<M>::vertex_index;
//     using Face = std::vector<vertex_index>;

//     M m;

//     // rings[k][i] = vertex at height slice k, radial slice i
//     std::vector<Face> rings(h_slices + 1);

//     for (int k = 0; k <= h_slices; ++k) {
//         FT t = FT(k) / h_slices;
//         FT z = t * height;
//         FT r = _rt(t);

//         rings[k].reserve(r_slices);

//         for (int i = 0; i < r_slices; ++i) {
//             FT a = 2.0 * M_PI * i / r_slices;
//             FT x = r * std::cos(a);
//             FT y = r * std::sin(a);

//             rings[k].push_back(m.add_vertex(P(x, y, z)));
//         }
//     }

//     // Side Faces (Quads)
//     for (int k = 0; k < h_slices; ++k) {
//         for (int i = 0; i < r_slices; ++i) {
//             int j = (i + 1) % r_slices;
//             m.add_face(Face{
//                 rings[k][i],
//                 rings[k][j],
//                 rings[k+1][j],
//                 rings[k+1][i]
//             });
//         }
//     }

//     // Bottom Cap
//     auto cb = m.add_vertex(P(0, 0, 0));
//     for (int i = 0; i < r_slices; ++i) {
//         int j = (i + 1) % r_slices;
//         m.add_face(Face{cb, rings[0][j], rings[0][i]});
//     }

//     // Top Cap
//     auto ct = m.add_vertex(P(0, 0, height));
//     for (int i = 0; i < r_slices; ++i) {
//         int j = (i + 1) % r_slices;
//         m.add_face(Face{ct, rings[h_slices][i], rings[h_slices][j]});
//     }

//     return m;
// }

// template<polygon_mesh_3 M,
//     typename FT = typename Traits<typename Traits<M>::vector_type>::value_type>
// M cube(FT _size=1)
// {
//     using P = typename Traits<M>::vector_type;
//     using vertex_index = typename Traits<M>::vertex_index;
//     using Face = std::vector<vertex_index>;

//     M m;

//     FT V = _size/static_cast<FT>(2);
//     std::array<vertex_index,8> vs = {
//         m.add_vertex(P(-V,-V,-V)),
//         m.add_vertex(P(-V,-V,+V)),
//         m.add_vertex(P(-V,+V,-V)),
//         m.add_vertex(P(-V,+V,+V)),
//         m.add_vertex(P(+V,-V,-V)),
//         m.add_vertex(P(+V,-V,+V)),
//         m.add_vertex(P(+V,+V,-V)),
//         m.add_vertex(P(+V,+V,+V))
//     };
//     for (const auto& f : cube_vertex_indices) {
//         m.add_face(Face{vs[f[0]],vs[f[1]],vs[f[2]],vs[f[3]]});
//     }
//     return m;
// }

}
