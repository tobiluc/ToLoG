#include <ToLoG/geometry/predicates/simplex_in_tet.hpp>
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
    const ToLoG::ORI o = ToLoG::sign_orient2d(a, b, c);
    EXPECT_EQ(ToLoG::sign_incircle(a,b,c,Point(0,0)), o);
    EXPECT_EQ(ToLoG::sign_incircle(a,b,c,Point(0,-1.1)), o);
    EXPECT_NE(ToLoG::sign_incircle(a,b,c,Point(0,1.1)), o);
}

TEST(PredicatesTest, PointSupportingSimplexInTetTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;

    T tet(P(0,0,0), P(0,0,1), P(1,0,0), P(0,1,0));
    using TT = ToLoG::TetTopology;
    TT::VAR s;

    s = ToLoG::supporting_simplex_in_tet(tet, P(-1,0,0));
    EXPECT_EQ(s, TT::VAR::O);

    s = ToLoG::supporting_simplex_in_tet(tet, P(0,0,0));
    EXPECT_EQ(s, TT::VAR::A);

    s = ToLoG::supporting_simplex_in_tet(tet, P(0,0,1));
    EXPECT_EQ(s, TT::VAR::B);

    s = ToLoG::supporting_simplex_in_tet(tet, P(1,0,0));
    EXPECT_EQ(s, TT::VAR::C);

    s = ToLoG::supporting_simplex_in_tet(tet, P(0,1,0));
    EXPECT_EQ(s, TT::VAR::D);

    s = ToLoG::supporting_simplex_in_tet(tet, P(0,0,0.5));
    EXPECT_TRUE(TT::is_same_edge(static_cast<TT::HE>(s), TT::HE::AB));

    s = ToLoG::supporting_simplex_in_tet(tet, P(0.5,0,0));
    EXPECT_TRUE(TT::is_same_edge(static_cast<TT::HE>(s), TT::HE::AC));

    s = ToLoG::supporting_simplex_in_tet(tet, P(0,0.5,0));
    EXPECT_TRUE(TT::is_same_edge(static_cast<TT::HE>(s), TT::HE::AD));

    s = ToLoG::supporting_simplex_in_tet(tet, P(0.5,0,0.5));
    EXPECT_TRUE(TT::is_same_edge(static_cast<TT::HE>(s), TT::HE::BC));

    s = ToLoG::supporting_simplex_in_tet(tet, P(0.1,0,0.1));
    EXPECT_TRUE(TT::is_same_halfface(static_cast<TT::HF>(s), TT::HF::ABC));

    s = ToLoG::supporting_simplex_in_tet(tet, P(0.1,0.1,0.1));
    EXPECT_TRUE(TT::is_tet(s));
}

TEST(PredicatesTest, PointSupportingSimplexInInvertedTetTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;
    using TT = ToLoG::TetTopology;

    T tet(P(0,0,0), P(1,0,0), P(0,0,1), P(0,1,0));

    auto i = ToLoG::supporting_simplex_in_tet(tet, P(-1,0,0));
    EXPECT_EQ(i, TT::VAR::O);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,0));
    EXPECT_EQ(i, TT::VAR::A);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,1));
    EXPECT_EQ(i, TT::VAR::C);

    i = ToLoG::supporting_simplex_in_tet(tet, P(1,0,0));
    EXPECT_EQ(i, TT::VAR::B);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,1,0));
    EXPECT_EQ(i, TT::VAR::D);

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0,0.5));
    EXPECT_TRUE(TT::is_same_edge(TT::he(i), TT::HE::AC));

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.5,0,0));
    EXPECT_TRUE(TT::is_same_edge(TT::he(i), TT::HE::AB));

    i = ToLoG::supporting_simplex_in_tet(tet, P(0,0.5,0));
    EXPECT_TRUE(TT::is_same_edge(TT::he(i), TT::HE::AD));

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.5,0,0.5));
    EXPECT_TRUE(TT::is_same_edge(TT::he(i), TT::HE::BC));

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.1,0,0.1));
    EXPECT_TRUE(TT::is_same_halfface(TT::hf(i), TT::HF::ABC));

    i = ToLoG::supporting_simplex_in_tet(tet, P(0.1,0.1,0.1));
    EXPECT_EQ(i, TT::VAR::ABCD);
}

TEST(PredicatesTest, ExactSimplexInTetRayTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;
    using S = ToLoG::Segment<P>;
    using TT = ToLoG::TetTopology;

    T tet(P(0,0,0), P(0,0,1), P(1,0,0), P(0,1,0));

    auto i2 = ToLoG::supporting_simplex_in_tet(tet, S(P(0,0,0), P(1,0,0)));
    EXPECT_TRUE(TT::is_halfedge(i2));
    EXPECT_TRUE(TT::is_same_edge(static_cast<TT::HE>(i2), TT::HE::AC));

    i2 = ToLoG::supporting_simplex_in_tet(tet, S(P(0,0,0), P(100,0,100)));
    EXPECT_TRUE(TT::is_halfface(i2));
    EXPECT_TRUE(TT::is_same_halfface(static_cast<TT::HF>(i2), TT::HF::ABC));

    i2 = ToLoG::supporting_simplex_in_tet(tet, S(P(0.2,0.1,0.3), P(-1,10,14)));
    EXPECT_EQ(i2, TT::VAR::ABCD);
}

TEST(PredicatesTest, SegmentThroughTetTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;
    using TT = ToLoG::TetTopology;
    using S = ToLoG::Segment<P>;
    const double eps = std::numeric_limits<double>::epsilon();

    // Classic Tet
    T tet(P(0,0,0), P(0,0,1), P(1,0,0), P(0,1,0));

    // Tet is oriented ccw
    EXPECT_EQ(ToLoG::sign_orient3d(tet), ToLoG::ORI::CCW);

    // Segment coming from pos z to neg z should leave through origin
    S s(P(0,0,4), P(0,0,-1));
    auto i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_EQ(i2.simplex, TT::VAR::A);
    EXPECT_EQ(i2.point, P(0,0,0));

    // Segment leaving through face on z plane
    s = S(P(0.2,0.2,4), P(0.1,0.1,-1));
    i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_EQ(i2.simplex, TT::VAR::CDA);
    EXPECT_EQ(i2.pierce_face, TT::HF::CDA);

    // Endpoint is contained within tet
    s = S(P(-1,-1,-1), P(0.1,0.1,0.1));
    i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_FALSE(TT::is_valid(i2.simplex));

    // Same direction as before, but we go further and leave through the diagonal face
    s = S(P(-1,-1,-1), P(1,1,1));
    i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_EQ(i2.simplex, TT::VAR::BDC);
    EXPECT_NEAR(ToLoG::squared_distance(i2.point, P(1./3.,1./3.,1./3.)), 0, eps);

    // Opposite direction from previous case, entering through diagonal face
    // and leaving through vertex at origin
    s = S(P(1,1,1), P(-3,-3,-3));
    i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_EQ(i2.simplex, TT::VAR::A);
    EXPECT_EQ(i2.point, P(0,0,0));

    // Segment is exactly on diagonal face -> no leaving simplex
    s = S(P(0,0.5,0.5), P(0.5,0.5,0));
    i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_FALSE(TT::is_valid(i2.simplex));

    // Same direction as before but we go further, leaving
    // through the edge shared by the diagonal face and the z plane
    s = S(P(0,0.5,0.5), P(0.75,0.5,-0.25));
    i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_TRUE(TT::is_halfedge(i2.simplex));
    EXPECT_TRUE(TT::is_same_edge(static_cast<TT::HE>(i2.simplex), TT::HE::CD));
}

TEST(PredicatesTest, SegmentThroughInvertedTetTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Tetrahedron<P>;
    using S = ToLoG::Segment<P>;
    using TT = ToLoG::TetTopology;

    T tet(P(0,0,0), P(1,0,0), P(0,0,1), P(0,1,0));

    EXPECT_EQ(ToLoG::sign_orient3d(tet), ToLoG::ORI::CW);

    S s(P(0,0,4), P(0,0,-1));
    auto i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_EQ(i2.simplex, TT::VAR::A);
    EXPECT_EQ(i2.point, P(0,0,0));

    s = S(P(0.2,0.2,4), P(0.1,0.1,-1));
    i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_EQ(i2.simplex, TT::VAR::DBA);

    s = S(P(1,1,1), P(-3,-3,-3));
    i2 = ToLoG::exiting_simplex_in_tet(tet, s);
    EXPECT_EQ(i2.simplex, TT::VAR::A);
    EXPECT_EQ(i2.point, P(0,0,0));
}
