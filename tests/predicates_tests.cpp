#include "ToLoG/predicates/derived_predicates_3d.hpp"
#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

TEST(PredicatesTest, SignOri2dTest1)
{
    using P = ToLoG::Point<double,2>;
    P p0(0,0);
    P p1(1,0);
    P p2(0,1);
    EXPECT_EQ(ToLoG::ORI::CW, ToLoG::sign_orient2d(p0.data(), p1.data(), p2.data()));
    EXPECT_EQ(ToLoG::ORI::CCW, ToLoG::sign_orient2d(p2.data(), p1.data(), p0.data()));
    EXPECT_EQ(ToLoG::ORI::ZERO, ToLoG::sign_orient2d(p0.data(), p1.data(), P(10,0).data()));
    EXPECT_EQ(ToLoG::ORI::ZERO, ToLoG::sign_orient2d(p0.data(), p2.data(), P(0,-7.5).data()));
}

TEST(PredicatesTest, SignOri3dTest1)
{
    using P = ToLoG::Point<double,3>;
    P p0(0,0,0);
    P p1(0,0,1);
    P p2(1,0,0);
    P p3(0,1,0);
    EXPECT_EQ(ToLoG::ORI::CCW, ToLoG::sign_orient3d(p0.data(), p1.data(), p2.data(), p3.data()));
    EXPECT_EQ(ToLoG::ORI::CW, ToLoG::sign_orient3d(p0.data(), p2.data(), p1.data(), p3.data()));
    EXPECT_EQ(ToLoG::ORI::ZERO, ToLoG::sign_orient3d(p0.data(), p1.data(), p2.data(), P(10,0,-10).data()));
}

TEST(PredicatesTest, PointInTriangle2dTest)
{
    using Point = ToLoG::Point<double,2>;
    using Triangle = ToLoG::Triangle<Point>;

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
    using Point = ToLoG::Point<double,2>;
    Point a(-1,-1);
    Point b(1,-1);
    Point c(0,1);
    EXPECT_GT(ToLoG::point_incircle(a,b,c,Point(0,0)), 0.0); // in circle
    EXPECT_GT(ToLoG::point_incircle(a,b,c,Point(0,-1.1)), 0.0); // in circle
    EXPECT_LT(ToLoG::point_incircle(a,b,c,Point(0,1.1)), 0.0); // outside circle
}

TEST(PredicatesTest, PointSupportingSimplexInTetTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;

    T tet(P(0,0,0), P(0,0,1), P(1,0,0), P(0,1,0));
    ToLoG::SimplexIndices i;

    i = ToLoG::supporting_simplex_in_tet(tet, P(-1,0,0));
    EXPECT_TRUE(i.empty());

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,0));
    EXPECT_TRUE(i.is_point());
    EXPECT_EQ(i[0], 0);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,1));
    EXPECT_TRUE(i.is_point());
    EXPECT_EQ(i[0], 1);

    i = ToLoG::supporting_simplex_in_tet(tet, P(1,0,0));
    EXPECT_TRUE(i.is_point());
    EXPECT_EQ(i[0], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,1,0));
    EXPECT_TRUE(i.is_point());
    EXPECT_EQ(i[0], 3);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,0.5));
    EXPECT_TRUE(i.is_segment());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 1);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.5,0,0));
    EXPECT_EQ(i.size(), 2);
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0.5,0));
    EXPECT_TRUE(i.is_segment());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 3);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.5,0,0.5));
    EXPECT_TRUE(i.is_segment());
    EXPECT_EQ(i[0], 1);
    EXPECT_EQ(i[1], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.1,0,0.1));
    EXPECT_TRUE(i.is_triangle());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 1);
    EXPECT_EQ(i[2], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.1,0.1,0.1));
    EXPECT_TRUE(i.is_tet());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 1);
    EXPECT_EQ(i[2], 2);
    EXPECT_EQ(i[3], 3);
}

TEST(PredicatesTest, PointSupportingSimplexInInvertedTetTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;

    T tet(P(0,0,0), P(1,0,0), P(0,0,1), P(0,1,0));
    ToLoG::SimplexIndices i;

    i = ToLoG::supporting_simplex_in_tet(tet, P(-1,0,0));
    EXPECT_TRUE(i.empty());

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,0));
    EXPECT_TRUE(i.is_point());
    EXPECT_EQ(i[0], 0);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,1));
    EXPECT_TRUE(i.is_point());
    EXPECT_EQ(i[0], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, P(1,0,0));
    EXPECT_TRUE(i.is_point());
    EXPECT_EQ(i[0], 1);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,1,0));
    EXPECT_TRUE(i.is_point());
    EXPECT_EQ(i[0], 3);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,0.5));
    EXPECT_TRUE(i.is_segment());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.5,0,0));
    EXPECT_EQ(i.size(), 2);
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 1);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0.5,0));
    EXPECT_TRUE(i.is_segment());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 3);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.5,0,0.5));
    EXPECT_TRUE(i.is_segment());
    EXPECT_EQ(i[0], 1);
    EXPECT_EQ(i[1], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.1,0,0.1));
    EXPECT_TRUE(i.is_triangle());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 1);
    EXPECT_EQ(i[2], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.1,0.1,0.1));
    EXPECT_TRUE(i.is_tet());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 1);
    EXPECT_EQ(i[2], 2);
    EXPECT_EQ(i[3], 3);
}

TEST(PredicatesTest, ExactSimplexInTetRayTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;
    using S = ToLoG::Segment<P>;

    T tet(P(0,0,0), P(0,0,1), P(1,0,0), P(0,1,0));
    ToLoG::SimplexIndices i;

    i = ToLoG::supporting_simplex_in_tet(tet, S(P(0,0,0), P(1,0,0)));
    EXPECT_TRUE(i.is_segment());
    EXPECT_EQ(i[0], 0);
    EXPECT_EQ(i[1], 2);

    i = ToLoG::supporting_simplex_in_tet(tet, S(P(0,0,0), P(100,0,100)));
    EXPECT_TRUE(i.is_triangle());

    i = ToLoG::supporting_simplex_in_tet(tet, S(P(0.2,0.1,0.3), P(-1,10,14)));
    EXPECT_TRUE(i.is_tet());
}

TEST(PredicatesTest, SegmentThroughTetTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;
    using S = ToLoG::Segment<P>;
    const double eps = std::numeric_limits<double>::epsilon();

    // Classic Tet
    T tet(P(0,0,0), P(0,0,1), P(1,0,0), P(0,1,0));

    // Tet is oriented ccw
    EXPECT_EQ(ToLoG::sign_orient3d(tet), ToLoG::ORI::CCW);

    // Segment coming from pos z to neg z should leave through origin
    S s(P(0,0,4), P(0,0,-1));
    auto i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.is_point());
    EXPECT_EQ(i.simplex[0], 0);
    EXPECT_EQ(i.point, P(0,0,0));

    // Segment leaving through face on z plane
    s = S(P(0.2,0.2,4), P(0.1,0.1,-1));
    i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.is_triangle());
    EXPECT_EQ(i.simplex[0], 0);
    EXPECT_EQ(i.simplex[1], 2);
    EXPECT_EQ(i.simplex[2], 3);

    // Endpoint is contained within tet
    s = S(P(-1,-1,-1), P(0.1,0.1,0.1));
    i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.empty());

    // Same direction as before, but we go further and leave through the diagonal face
    s = S(P(-1,-1,-1), P(1,1,1));
    i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.is_triangle());
    EXPECT_EQ(i.simplex[0], 1);
    EXPECT_EQ(i.simplex[1], 3);
    EXPECT_EQ(i.simplex[2], 2);
    EXPECT_NEAR(ToLoG::squared_distance(i.point, P(1./3.,1./3.,1./3.)), 0, eps);

    // Opposite direction from previous case, entering through diagonal face
    // and leaving through vertex at origin
    s = S(P(1,1,1), P(-3,-3,-3));
    i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.is_point());
    EXPECT_EQ(i.simplex[0], 0);
    EXPECT_EQ(i.point, P(0,0,0));

    // Segment is exactly on diagonal face -> no leaving simplex
    s = S(P(0,0.5,0.5), P(0.5,0.5,0));
    i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.empty());

    // Same direction as before but we go further, leaving
    // through the edge shared by the diagonal face and the z plane
    s = S(P(0,0.5,0.5), P(0.75,0.5,-0.25));
    i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.is_segment());
    EXPECT_NEAR(ToLoG::squared_distance(i.point, P(0.5,0.5,0)), 0, eps);
}

TEST(PredicatesTest, SegmentThroughInvertedTetTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;
    using S = ToLoG::Segment<P>;

    T tet(P(0,0,0), P(1,0,0), P(0,0,1), P(0,1,0));

    EXPECT_EQ(ToLoG::sign_orient3d(tet), ToLoG::ORI::CW);

    S s(P(0,0,4), P(0,0,-1));
    auto i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.is_point());
    EXPECT_EQ(i.simplex[0], 0);
    EXPECT_EQ(i.point, P(0,0,0));

    s = S(P(0.2,0.2,4), P(0.1,0.1,-1));
    i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.is_triangle());
    EXPECT_EQ(i.simplex[0], 0);
    EXPECT_EQ(i.simplex[1], 3);
    EXPECT_EQ(i.simplex[2], 1);

    s = S(P(1,1,1), P(-3,-3,-3));
    i = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(i.simplex.is_point());
    EXPECT_EQ(i.simplex[0], 0);
    EXPECT_EQ(i.point, P(0,0,0));
}
