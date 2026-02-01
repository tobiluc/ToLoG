#include "ToLoG/mesh/delauney_triangulation.hpp"
#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>
#include <ToLoG/predicates/contains_point.hpp>

namespace ToLoG
{

TEST(PredicatesTest, SignOri2dTest1)
{
    Point<double,2> p0(0,0);
    Point<double,2> p1(1,0);
    Point<double,2> p2(0,1);
    ASSERT_EQ(ORI::CW, sign_orient2d(p0.data(), p1.data(), p2.data()));
    ASSERT_EQ(ORI::CCW, sign_orient2d(p2.data(), p1.data(), p0.data()));
    ASSERT_EQ(ORI::ZERO, sign_orient2d(p0.data(), p1.data(), Point<double,2>(10,0).data()));
    ASSERT_EQ(ORI::ZERO, sign_orient2d(p0.data(), p2.data(), Point<double,2>(0,-7.5).data()));
}

TEST(PredicatesTest, SignOri3dTest1)
{
    Point<double,3> p0(0,0,0);
    Point<double,3> p1(0,0,1);
    Point<double,3> p2(1,0,0);
    Point<double,3> p3(0,1,0);
    ASSERT_EQ(ORI::CCW, sign_orient3d(p0.data(), p1.data(), p2.data(), p3.data()));
    ASSERT_EQ(ORI::CW, sign_orient3d(p0.data(), p2.data(), p1.data(), p3.data()));
    ASSERT_EQ(ORI::ZERO, sign_orient3d(p0.data(), p1.data(), p2.data(), Point<double,3>(10,0,-10).data()));
}

TEST(PredicatesTest, PointInTriangle2dTest)
{
    using Point = Point<double,2>;
    using Triangle = Triangle<Point>;

    Point t0(0,0);
    Point t1(0,5);
    Point t2(9,-4);
    Triangle tri(t0,t1,t2);

    EXPECT_TRUE(contains_point(tri, t0));
    EXPECT_TRUE(contains_point(tri, t1));
    EXPECT_TRUE(contains_point(tri, t2));
    EXPECT_TRUE(contains_point(tri, Point(0,2.1)));
    EXPECT_FALSE(contains_point(tri, Point(0,6)));
}

TEST(PredicatesTest, DelauneyTest)
{
    using Point = Point<double,2>;

    std::vector<Point> points = {Point(0,0)};
    delauney_triangulation(points);
}

}
