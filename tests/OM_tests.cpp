#include <gtest/gtest.h>
#include <ToLoG/utils/OM_Traits.hpp>
#include <ToLoG/Core.hpp>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <ToLoG/mesh/polygon_mesh.hpp>
#include <ToLoG/io/obj_writer.hpp>


TEST(OMTest, OMTest1)
{

    typedef OpenMesh::PolyMesh_ArrayKernelT<OpenMesh::DefaultTraits>  Mesh;

    OpenMesh::Vec3d p(1,2,3);
    OpenMesh::Vec3d q(-1,10,0);
    EXPECT_EQ(ToLoG::dot(p,q), p.dot(q));
    EXPECT_EQ(ToLoG::squared_distance(p,q), (p-q).sqrnorm());

    Mesh cube = ToLoG::cube<Mesh>();
    EXPECT_EQ(ToLoG::IO::write_polygon_mesh_obj(
        std::filesystem::path(TESTS_OUTPUT_DIR)/"om_cube.obj", cube), 0);

    cube.face(OpenMesh::FaceHandle(0));
    auto f = cube.face(OpenMesh::FaceHandle(0));
    //cube.vert
    //cube.faces

    for (auto f : cube.faces()) {
        f.vertices();
        for (auto v : f.vertices()) {
            v.idx();
            // v should be vertex_index
        }
    }
}
