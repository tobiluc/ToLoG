#include "ToLoG/AABBTree.hpp"
#include <gtest/gtest.h>
#include <ToLoG/utils/EigenTraits.hpp>
#include <ToLoG/Core.hpp>

TEST(EigenTest, EigenTest1)
{
    Eigen::Vector3d p(1,2,3);
    Eigen::Vector3d q(-1,10,0);
    EXPECT_EQ(ToLoG::dot(p,q), p.dot(q));
    EXPECT_EQ(ToLoG::squared_distance(p,q), (p-q).squaredNorm());

    ToLoG::AABB<Eigen::Vector3d> bbox({p,q});
    EXPECT_EQ(bbox.min(), Eigen::Vector3d(-1,2,0));
    EXPECT_EQ(bbox.max(), Eigen::Vector3d(1,10,3));

    std::vector<Eigen::Vector3d> points = {p,q};
    ToLoG::AABBTree<Eigen::Vector3d> tree(points);
    EXPECT_EQ(tree.k_nearest_neighbors(p, 2)[0], 0);
    EXPECT_EQ(tree.k_nearest_neighbors(q, 2)[0], 1);
    auto is = tree.intersecting(p);
    EXPECT_EQ(is.size(), 1);
    EXPECT_EQ(is[0], 0);
    is = tree.intersecting(ToLoG::Sphere<Eigen::Vector3d>({0,0,0}, 100));
    EXPECT_EQ(is.size(), 2);

    ToLoG::Sphere<Eigen::Vector2f> circle({1,1}, 5.0f);
    EXPECT_EQ(circle.squared_radius(), 25.0f);
}
