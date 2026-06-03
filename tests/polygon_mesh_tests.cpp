// #include "ToLoG/io/obj_reader.hpp"
// #include <gtest/gtest.h>
// #include <ToLoG/mesh/polygon_mesh.hpp>
// #include <ToLoG/Core.hpp>
// #include <ToLoG/io/obj_writer.hpp>

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
