#include <gtest/gtest.h>
#include <ToLoG/utils/OVM_Traits.hpp>
#include <ToLoG/Core.hpp>

TEST(OVMTest, OVMTest1)
{
    OpenVolumeMesh::Vec3d p(1,2,3);
    OpenVolumeMesh::Vec3d q(-1,10,0);
    EXPECT_EQ(ToLoG::dot(p,q), p.dot(q));
    EXPECT_EQ(ToLoG::squared_distance(p,q), (p-q).sqrnorm());
}
