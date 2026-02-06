#include <gtest/gtest.h>
#include <ToLoG/mesh/polygon_mesh.hpp>
#include <ToLoG/Core.hpp>
#include <ToLoG/io/obj_writer.hpp>

TEST(PolygonMeshTest, CubeTest)
{
    using P = ToLoG::Point<double, 3>;
    using Mesh = ToLoG::PolygonMesh3<P>;

    Mesh cube = ToLoG::cube<Mesh>();
    EXPECT_EQ(cube.n_vertices(), 8);
    EXPECT_EQ(cube.n_faces(), 6);
    cube = ToLoG::triangulated_faces(cube);
    EXPECT_EQ(cube.n_faces(), 12);
    EXPECT_EQ(cube.point(0), P(-0.5,-0.5,-0.5));
    EXPECT_EQ(cube.point(7), P(0.5,0.5,0.5));

    EXPECT_EQ(ToLoG::IO::write_polygon_mesh_obj(
        std::filesystem::path(TESTS_OUTPUT_DIR)/"cube.obj", cube), 0);
}

TEST(PolygonMeshTest, CylinderTest)
{
    using P = ToLoG::Point<double, 3>;
    using Mesh = ToLoG::PolygonMesh3<P>;

    Mesh cylinder = ToLoG::cylinder<Mesh>();

    EXPECT_EQ(ToLoG::IO::write_polygon_mesh_obj(
        std::filesystem::path(TESTS_OUTPUT_DIR)/"cylinder.obj", cylinder), 0);
}
