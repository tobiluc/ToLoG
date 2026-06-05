#include <gtest/gtest.h>
#include <ToLoG/mesh/CellComplex.hpp>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

using P = Point<double,3>;
using Mesh = PolyhedralMesh<P>;

TEST(MeshTest, TriangleFaceTest)
{
    Mesh m;
    Mesh::VH vh0 = m.add_vertex(P(0,0,0));
    Mesh::VH vh1 = m.add_vertex(P(0,0,1));
    Mesh::VH vh2 = m.add_vertex(P(1,0,0));
    Mesh::HEH heh01 = m.add_halfedge(vh0, vh1);
    Mesh::HEH heh12 = m.add_halfedge(vh1, vh2);
    Mesh::HEH heh20 = m.add_halfedge(vh2, vh0);
    Mesh::HFH hfh = m.add_halfface({heh01, heh12, heh20});

    EXPECT_EQ(hfh.fh(), m.find_face({vh2, vh0, vh1}));
    EXPECT_EQ(hfh, m.find_halfface({vh2, vh0, vh1}));
    EXPECT_EQ(hfh.opp(), m.find_halfface({vh2, vh1, vh0}));
    EXPECT_EQ(Mesh::FH(), m.find_face({vh2, vh1, vh1}));
    EXPECT_EQ(3, m.num_allocated_vertices());
    EXPECT_EQ(3, m.num_allocated_edges());
    EXPECT_EQ(6, m.num_allocated_halfedges());
    EXPECT_EQ(1, m.num_allocated_faces());
    EXPECT_EQ(0, m.num_allocated_cells());
    EXPECT_EQ(P(1,0,0), m.point(vh2));
    EXPECT_EQ(vh0, m.vh(heh01, 0));
    EXPECT_EQ(vh0, m.vh(heh20, 1));

    m.delete_face(hfh.fh());

    EXPECT_EQ(Mesh::FH(), m.find_face({vh2, vh0, vh1}));
    EXPECT_EQ(Mesh::HFH(), m.find_halfface({vh2, vh0, vh1}));
    EXPECT_EQ(Mesh::HFH(), m.find_halfface({vh2, vh1, vh0}));
    EXPECT_EQ(Mesh::FH(), m.find_face({vh2, vh1, vh1}));
    EXPECT_EQ(3, m.num_allocated_vertices());
    EXPECT_EQ(3, m.num_allocated_edges());
    EXPECT_EQ(6, m.num_allocated_halfedges());
    EXPECT_EQ(1, m.num_allocated_faces());
    EXPECT_EQ(1, m.num_deleted_faces());
    EXPECT_EQ(0, m.num_active_faces());
    EXPECT_EQ(0, m.num_allocated_cells());
}

}

// TEST(PolygonMeshTest, CubeTest)
// {
//     using P = ToLoG::Point<double, 3>;
//     using Mesh = ToLoG::PolygonMesh<P>;

//     Mesh cube = ToLoG::cube<Mesh>();
//     P p0(-.5,-.5,-.5);
//     EXPECT_EQ(cube.point(0), p0);
//     EXPECT_EQ(cube.n_vertices(), 8);
//     EXPECT_EQ(cube.n_edges(), 12);
//     EXPECT_EQ(cube.n_faces(), 6);

//     size_t n_v(0);
//     for (auto vv_it = cube.vv_iter(0); vv_it.is_valid(); ++vv_it) {
//         ++n_v;
//     }
//     EXPECT_EQ(n_v, 3);

//     cube = ToLoG::triangulated_faces(cube);
//     EXPECT_EQ(cube.n_vertices(), 8);
//     EXPECT_EQ(cube.n_edges(), 18);
//     EXPECT_EQ(cube.n_faces(), 12);
//     EXPECT_EQ(cube.point(0), P(-0.5,-0.5,-0.5));
//     EXPECT_EQ(cube.point(7), P(0.5,0.5,0.5));

//     n_v = 0;
//     for (auto vv_it = cube.vv_iter(0); vv_it.is_valid(); ++vv_it) {
//         ++n_v;
//     }
//     EXPECT_EQ(n_v, 6);

//     EXPECT_EQ(ToLoG::IO::write_polygon_mesh_obj(
//         std::filesystem::path(TESTS_OUTPUT_DIR)/"cube.obj", cube), 0);

//     cube.clear();

//     EXPECT_EQ(ToLoG::IO::read_polygon_mesh_obj(
//         std::filesystem::path(TESTS_OUTPUT_DIR)/"cube.obj", cube), 0);
//     EXPECT_EQ(cube.n_vertices(), 8);
//     EXPECT_EQ(cube.n_faces(), 12);

//     EXPECT_EQ(ToLoG::IO::write_polygon_mesh_obj(
//         std::filesystem::path(TESTS_OUTPUT_DIR)/"cube2.obj", cube), 0);

// }

// TEST(PolygonMeshTest, CylinderTest)
// {
//     using P = ToLoG::Point<double, 3>;
//     using Mesh = ToLoG::PolygonMesh<P>;

//     Mesh mesh = ToLoG::cylinder<Mesh>();

//     EXPECT_EQ(ToLoG::IO::write_polygon_mesh_obj(
//         std::filesystem::path(TESTS_OUTPUT_DIR)/"cylinder.obj", mesh), 0);
// }
