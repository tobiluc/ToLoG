#include <gtest/gtest.h>
#include <ToLoG/unstable/Octree.hpp>

namespace ToLoG
{

TEST(OctreeTest, Test1)
{
    AABB<Point<double,3>> aabb({{0,0,0}, {1,1,1}});
    Octree3d tree(aabb, 2, 5);
    EXPECT_EQ(2*2*2, tree.n_nodes());
    EXPECT_EQ(UINT32_MAX, tree.locate(Point<double,3>(-1,0,0)));
}

}
