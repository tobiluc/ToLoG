#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

TEST(GeometryCoreTest, TraitsTest3d)
{
    using Point = ToLoG::Point<double, 3>;
    using Segment = ToLoG::Segment<Point>;
    using Triangle = ToLoG::Triangle<Point>;
    using Sphere = ToLoG::Sphere<Point>;
    using AABB = ToLoG::AABB<Point>;

    EXPECT_EQ((ToLoG::Traits<Point>::dim), 3);
    EXPECT_TRUE((std::is_same<ToLoG::Traits<Point>::value_type, double>::value));
    EXPECT_TRUE((std::is_same<ToLoG::Traits<Point>::vector_type, Point>::value));

    EXPECT_TRUE((ToLoG::is_vector_type<Point>::value));
    EXPECT_FALSE((ToLoG::is_vector_type<AABB>::value));
    EXPECT_FALSE((ToLoG::is_vector_type<Triangle>::value));
    EXPECT_FALSE((ToLoG::is_vector_type<Segment>::value));
    EXPECT_FALSE((ToLoG::is_vector_type<Sphere>::value));
}

TEST(GeometryCoreTest, PointTest)
{
    using Point = ToLoG::Point<double, 3>;
    Point p(3,4,-5);

    EXPECT_EQ(max(p), 4);
    EXPECT_EQ(min(p), -5);
    EXPECT_EQ(max(abs(p)), 5);

    ASSERT_EQ(3, p[0]);
    ASSERT_EQ(4, p[1]);
    ASSERT_EQ(-5, p[2]);
    int d = ToLoG::Traits<Point>::dim;
    ASSERT_EQ(3, d);

    ASSERT_EQ(p, ToLoG::aabb(p).min());
    ASSERT_EQ(p, ToLoG::aabb(p).max());
    ASSERT_EQ(p, ToLoG::centroid(p));

    ASSERT_LT(Point(2,5,1), p);
    ASSERT_LT(p, Point(3,4,-4));

    p = Point(3.2, 4.9, -5.6);
    EXPECT_EQ(ToLoG::rounded(p), Point(3,5,-6));
}

TEST(GeometryCoreTest, AABBTest)
{
    using Point = ToLoG::Point<double, 3>;
    using Segment = ToLoG::Segment<Point>;
    using Triangle = ToLoG::Triangle<Point>;
    using Sphere = ToLoG::Sphere<Point>;
    using AABB = ToLoG::AABB<Point>;

    AABB aabb;
    Point a(3,4,-5);
    Point b(1,0,5);
    Point c(1,0,5);
    Point d(1,0,5);
    aabb = ToLoG::aabb(a);
    Segment seg(a, b);
    aabb = ToLoG::aabb(seg);
    Triangle tri(a, b, c);
    aabb = ToLoG::aabb(tri);
    Sphere sphere(a, 4);
    aabb = ToLoG::aabb(sphere);
}

TEST(CoreTest, BarycentricTest)
{
    using P = ToLoG::Point<double, 3>;
    using Tri = ToLoG::Triangle<P>;
    using Tet = ToLoG::Tetrahedron<P>;

    Tri tri(P(0,0.5,0), P(0,0,1), P(2,0,0));
    for (int i = 0; i < 3; ++i) {
        auto b = ToLoG::barycentric_coordinates(tri[i], tri);
        for (int j = 0; j < 3; ++j) {
            EXPECT_EQ(b[j], (i==j)? 1.0 : 0.0);
        }
    }

    Tet tet(P(0,0.25,0), P(0,0,3.5), P(10,0,0), P(0,9.5,0));
    for (int i = 0; i < 4; ++i) {
        auto b = ToLoG::barycentric_coordinates(tet[i], tet);
        for (int j = 0; j < 4; ++j) {
            EXPECT_EQ(b[j], (i==j)? 1.0 : 0.0);
        }
    }
}

TEST(GeometryCoreTest, CentroidTest)
{
    using Point = ToLoG::Point<double, 3>;
    using Segment = ToLoG::Segment<Point>;
    using Triangle = ToLoG::Triangle<Point>;
    using Sphere = ToLoG::Sphere<Point>;
    using AABB = ToLoG::AABB<Point>;

    Point center;
    Point a(3,4,-5);
    Point b(1,0,5);
    Point c(1,0,5);
    Point d(1,0,5);
    center = ToLoG::centroid(a);
    Segment seg(a, b);
    center = ToLoG::centroid(seg);
    Triangle tri(a, b, c);
    center = ToLoG::centroid(tri);
    Sphere sphere(a, 4);
    center = ToLoG::centroid(sphere);
}

TEST(GeometryCoreTest, Triangle2dTest)
{
    using Point = ToLoG::Point<double,2>;
    using Triangle = ToLoG::Triangle<Point>;
    using AABB = ToLoG::AABB<Point>;
    using Segment = ToLoG::Segment<Point>;

    Point a(0,0);
    Point b(3,0);
    Point c(0,4);
    Triangle tri(a,b,c);
    EXPECT_EQ(AABB({Point(0,0),Point(3,4)}), ToLoG::aabb(tri));
    EXPECT_EQ(6, area(tri));
    EXPECT_EQ(12, circumference(tri));
    EXPECT_EQ(Segment(a,b), tri.segment(0));
    EXPECT_EQ(Segment(b,c), tri.segment(1));
    EXPECT_EQ(Segment(c,a), tri.segment(2));
}

TEST(GeometryCoreTest, AABBPointsTest)
{
    using Point = ToLoG::Point<double, 3>;
    Point p1(-1,-3,10);
    Point p2(4,-3,0);
    ToLoG::AABB<Point> aabb({p1, p2});

    EXPECT_EQ(Point(-1,-3,0), aabb.min());
    EXPECT_EQ(Point(4,-3,10), aabb.max());
    EXPECT_EQ(Point(1.5,-3,5), ToLoG::centroid(aabb));

    auto corners = aabb.corners();
    EXPECT_EQ(8, corners.size());
    EXPECT_EQ(corners[0], Point(-1,-3,0));
    EXPECT_EQ(corners[1], Point(-1,-3,10));
    EXPECT_EQ(corners[2], Point(-1,-3,0));
    EXPECT_EQ(corners[3], Point(-1,-3,10));
    EXPECT_EQ(corners[4], Point(4,-3,0));
    EXPECT_EQ(corners[5], Point(4,-3,10));
    EXPECT_EQ(corners[6], Point(4,-3,0));
    EXPECT_EQ(corners[7], Point(4,-3,10));
}

TEST(CoreTest, TriangleArea3dTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Triangle<P>;

    auto area3d = [](const P& A, const P& B, const P& C) -> double {
        return 0.5 * ToLoG::norm(ToLoG::cross((C-A),(B-A)));
    };
    constexpr double eps = std::numeric_limits<double>::epsilon();

    P a(1,2,3);
    P b(0,-14,31.123);
    P c(-9,0,-3);

    EXPECT_NEAR(ToLoG::area(T(a,b,c)), area3d(a,b,c), eps);
    EXPECT_NEAR(ToLoG::area(T(a,c,b)), area3d(a,c,b), eps);
}

TEST(GeometryCoreTest, PointDistanceTest2d)
{
    using Point = ToLoG::Point<double, 2>;
    using Segment = ToLoG::Segment<Point>;
    using Triangle = ToLoG::Triangle<Point>;
    using Sphere = ToLoG::Sphere<Point>;
    using AABB = ToLoG::AABB<Point>;

    Point q(-10,4);

    Point p(1.5, 3);
    EXPECT_EQ(133.25, ToLoG::squared_distance(q, p));

    Segment seg(Point(-12,6), Point(5,6));
    EXPECT_EQ(4, ToLoG::squared_distance(q, seg));
    seg = Segment(Point(1,6), Point(5,6));
    EXPECT_EQ(125, ToLoG::squared_distance(q, seg));
    seg = Segment(Point(-11,0), Point(-9,8));
    EXPECT_EQ(0, ToLoG::squared_distance(q, seg));
}

TEST(GeometryCoreTest, TetrahedronTest)
{
    using Point = ToLoG::Point<double,3>;

    Point a(0,0,0);
    Point b(0,0,1);
    Point c(1,0,0);
    Point d(0,1,0);

    ToLoG::Tetrahedron<Point> tet(a,b,c,d);
}

TEST(GeometryCoreTest, AABBDistanceTest)
{
    using Point = ToLoG::Point<double,2>;
    using AABB = ToLoG::AABB<Point>;

    AABB b0({Point(0,0), Point(1,1)});
    AABB b1({Point(1.5,1), Point(2,2)});
    AABB b2({Point(0.5,0.5), Point(2,1)});
    AABB b3({Point(-10,-10), Point(-8,10)});

    constexpr double eps = std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(ToLoG::squared_distance(b0, b1), 0.25, eps);
    EXPECT_EQ(ToLoG::squared_distance(b0, b2), 0);
    EXPECT_EQ(ToLoG::squared_distance(b1, b2), 0);
    EXPECT_NEAR(ToLoG::squared_distance(b0, b3), 64, eps);
}
