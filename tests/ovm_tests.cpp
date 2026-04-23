#include <gtest/gtest.h>
#include <ToLoG/utils/OVM_Traits.hpp>
#include <ToLoG/Core.hpp>

TEST(OVMTest, OVMTest1)
{
    OpenVolumeMesh::Vec3d p(1,2,3);
    OpenVolumeMesh::Vec3d q(-1,10,0);
    EXPECT_EQ(ToLoG::dot(p,q), p.dot(q));
    EXPECT_EQ(ToLoG::squared_distance(p,q), (p-q).sqrnorm());

    using TetMesh = OpenVolumeMesh::GeometryKernel<OpenVolumeMesh::Vec3d, OpenVolumeMesh::TopologyKernel>;


    EXPECT_TRUE((std::is_same<ToLoG::Traits<TetMesh>::vector_type,OpenVolumeMesh::Vec3d>::value));
}

TEST(OVMTest, OVMReadTetMeshTest)
{
    using Vec3d = OpenVolumeMesh::Vec3d;
    using TetMesh = OpenVolumeMesh::TetrahedralGeometryKernel<Vec3d, OpenVolumeMesh::TetrahedralMeshTopologyKernel>;
    TetMesh mesh;

    ToLoG::Traits<TetMesh>::vertex_index vh;

    // EXPECT_EQ(ToLoG::IO::read_polyhedral_mesh_medit(
    //     "/Users/tobiaskohler/Desktop/tetmesh.mesh", mesh), 0);

}
