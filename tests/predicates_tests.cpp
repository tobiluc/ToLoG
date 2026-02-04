#include "ToLoG/mesh/delauney_triangulation.hpp"
#include "ToLoG/predicates/derived_predicates.hpp"
#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

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

    EXPECT_TRUE(intersects(tri, t0));
    EXPECT_TRUE(intersects(tri, t1));
    EXPECT_TRUE(intersects(tri, t2));
    EXPECT_TRUE(intersects(tri, Point(0,2.1)));
    EXPECT_FALSE(intersects(tri, Point(0,6)));
}

TEST(PredicatesTest, InCircleTest)
{
    using Point = Point<double,2>;
    Point a(-1,-1);
    Point b(1,-1);
    Point c(0,1);
    EXPECT_GT(point_incircle(a,b,c,Point(0,0)), 0.0); // in circle
    EXPECT_GT(point_incircle(a,b,c,Point(0,-1.1)), 0.0); // in circle
    EXPECT_LT(point_incircle(a,b,c,Point(0,1.1)), 0.0); // outside circle
}

TEST(PredicatesTest, ExactSimplexInTetTest)
{
    using P = Point<double,3>;
    using T = Tetrahedron<P>;

    T tet(P(0,0,0), P(0,0,1), P(1,0,0), P(0,1,0));
    std::vector<int> i;

    i = ToLoG::exact_simplex_in_tet(tet, P(-1,0,0));
    EXPECT_TRUE(i.empty());

    i = ToLoG::exact_simplex_in_tet(tet, P(0,0,0));
    EXPECT_EQ(i.size(), 1);
    EXPECT_EQ(i[0], 0);
    i = ToLoG::exact_simplex_in_tet(tet, P(0,0,1));
    EXPECT_EQ(i.size(), 1);
    EXPECT_EQ(i[0], 1);
    i = ToLoG::exact_simplex_in_tet(tet, P(1,0,0));
    EXPECT_EQ(i.size(), 1);
    EXPECT_EQ(i[0], 2);
    i = ToLoG::exact_simplex_in_tet(tet, P(0,1,0));
    EXPECT_EQ(i.size(), 1);
    EXPECT_EQ(i[0], 3);

    i = ToLoG::exact_simplex_in_tet(tet, P(0,0,0.5));
    EXPECT_EQ(i.size(), 2);
    i = ToLoG::exact_simplex_in_tet(tet, P(0.5,0,0));
    EXPECT_EQ(i.size(), 2);
    i = ToLoG::exact_simplex_in_tet(tet, P(0,0.5,0));
    EXPECT_EQ(i.size(), 2);
    i = ToLoG::exact_simplex_in_tet(tet, P(0.5,0,0.5));
    EXPECT_EQ(i.size(), 2);

    i = ToLoG::exact_simplex_in_tet(tet, P(0.1,0,0.1));
    EXPECT_EQ(i.size(), 3);

    i = ToLoG::exact_simplex_in_tet(tet, P(0.1,0.1,0.1));
    EXPECT_EQ(i.size(), 4);
}

TEST(PredicatesTest, DelauneyTest)
{
    using Point = Point<double,2>;

    std::vector<Point> points = {
        Point(0,2),
        Point(-5,5),
        Point(5,5),
        Point(-2,3),
        Point(3,1),
        Point(-4,-1),
        Point(1,-2),
        Point(-6,-4),
        Point(5,-4)
    };
    // std::vector<Point> points = {
    //     Point(-1,-1),
    //     Point(1,-1),
    //     Point(0,1)
    // };

    // points.clear();
    // for (int x = 0; x < 2; ++x) {
    //     for (int y = 0; y < 2; ++y) {
    //         points.emplace_back(x,y);
    //     }
    // }
    // std::vector<Point> points = {
    //     Point(0,0),
    //     Point(-1,-1),
    //     Point(1,-1),
    //     Point(0,1)
    // };

    delauney_triangulation(points);
}

}
