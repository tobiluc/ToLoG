#pragma once
#include <ToLoG/utils/indices.hpp>
#include <ToLoG/mesh/CellComplex.hpp>

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

template<vector P,
    typename FT = typename Traits<P>::value_type>
PolyhedralMesh<P> create_cube(FT _size=1)
{
    PolyhedralMesh<P> cube;
    FT V = _size/static_cast<FT>(2);
    std::array<typename PolyhedralMesh<P>::VH,8> vhs = {
        cube.add_vertex(P(-V,-V,-V)),
        cube.add_vertex(P(-V,-V,+V)),
        cube.add_vertex(P(-V,+V,-V)),
        cube.add_vertex(P(-V,+V,+V)),
        cube.add_vertex(P(+V,-V,-V)),
        cube.add_vertex(P(+V,-V,+V)),
        cube.add_vertex(P(+V,+V,-V)),
        cube.add_vertex(P(+V,+V,+V))
    };
    std::vector<typename PolyhedralMesh<P>::HFH> hfhs;
    for (int i = 0; i < 6; ++i) {
        const auto& f = cube_vertex_indices[i];
        hfhs.push_back(cube.add_halfface({vhs[f[0]],vhs[f[1]],vhs[f[2]],vhs[f[3]]}));
    }
    cube.add_cell(hfhs);
    return cube;
}

}
