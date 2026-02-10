#include "OpenMesh/Core/IO/MeshIO.hh"
#include <gtest/gtest.h>
#include <ToLoG/utils/OM_Traits.hpp>
#include <ToLoG/Core.hpp>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <ToLoG/mesh/polygon_mesh.hpp>
#include <ToLoG/io/obj_writer.hpp>
#include <ToLoG/io/obj_reader.hpp>

TEST(OMTest, OMTest1)
{

    typedef OpenMesh::PolyMesh_ArrayKernelT<OpenMesh::DefaultTraits>  Mesh;

    OpenMesh::Vec3d p(1,2,3);
    OpenMesh::Vec3d q(-1,10,0);
    EXPECT_EQ(ToLoG::dot(p,q), p.dot(q));
    EXPECT_EQ(ToLoG::squared_distance(p,q), (p-q).sqrnorm());

    Mesh mesh = ToLoG::cube<Mesh>();
    EXPECT_EQ(ToLoG::IO::write_polygon_mesh_obj(
        std::filesystem::path(TESTS_OUTPUT_DIR)/"om_cube.obj", mesh), 0);

    mesh.clear();
    EXPECT_EQ(mesh.n_vertices(), 0);
    EXPECT_EQ(mesh.n_faces(), 0);

    EXPECT_EQ(ToLoG::IO::read_polygon_mesh_obj(
        std::filesystem::path(TESTS_OUTPUT_DIR)/"om_cube.obj", mesh), 0);

    EXPECT_EQ(mesh.n_vertices(), 8);
    EXPECT_EQ(mesh.n_faces(), 6);
}
