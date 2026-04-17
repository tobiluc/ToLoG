#include <gtest/gtest.h>
#include <ToLoG/tree/KDTree.hpp>

namespace ToLoG
{

TEST(KDTreeTest, Simple2dTest)
{
    using P = Point<double,2>;
    using Tree = KDTree<P>;

    std::vector<P> points = {
        {0,0}, {1,0}, {1,1}, {0,1}
    };
    Tree tree(points, 1);

    std::vector<uint32_t> inds;
    tree.k_nearest_neighbors(P(0.3,0.2), 2, inds);
    EXPECT_EQ(inds.size(), 2);
    EXPECT_EQ(inds[0], 1);
    EXPECT_EQ(inds[1], 0);

    tree.radius_search(P(2,2), 2.5, inds);
    EXPECT_EQ(inds.size(), 3);
    EXPECT_EQ(std::find(inds.begin(),inds.end(),0), inds.end());
    EXPECT_NE(std::find(inds.begin(),inds.end(),1), inds.end());
    EXPECT_NE(std::find(inds.begin(),inds.end(),2), inds.end());
    EXPECT_NE(std::find(inds.begin(),inds.end(),3), inds.end());
}

}
