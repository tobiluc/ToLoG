#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

TEST(GeometryCoreTest, PointTest)
{
    using Point = ToLoG::Point<double, 3>;
    Point p(3,4,-5);
    ASSERT_EQ(3, p[0]);
    ASSERT_EQ(4, p[1]);
    ASSERT_EQ(-5, p[2]);
    int d = ToLoG::Traits<Point>::dim;
    ASSERT_EQ(3, d);

    ASSERT_EQ(p, p.aabb().min());
    ASSERT_EQ(p, p.aabb().max());
    ASSERT_EQ(p, p.centroid());

    ASSERT_LT(Point(2,5,1), p);
    ASSERT_LT(p, Point(3,4,-4));
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
    EXPECT_EQ(AABB({Point(0,0),Point(3,4)}), tri.aabb());
    EXPECT_EQ(6, tri.area());
    EXPECT_EQ(12, tri.circumference());
    EXPECT_EQ(Segment(a,b), tri.edge(0));
    EXPECT_EQ(Segment(b,c), tri.edge(1));
    EXPECT_EQ(Segment(c,a), tri.edge(2));
}

TEST(GeometryCoreTest, AABBPointsTest)
{
    using Point = ToLoG::Point<double, 3>;
    Point p1(-1,-3,10);
    Point p2(4,-3,0);
    ToLoG::AABB<Point> aabb({p1, p2});

    EXPECT_EQ(Point(-1,-3,0), aabb.min());
    EXPECT_EQ(Point(4,-3,10), aabb.max());
    EXPECT_EQ(Point(1.5,-3,5), aabb.centroid());

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

TEST(GeometryCoreTest, PointDistanceTest2d)
{
    using Point = ToLoG::Point<double, 2>;
    using Segment = ToLoG::Segment<Point>;
    using Triangle = ToLoG::Triangle<Point>;
    using Sphere = ToLoG::Sphere<Point>;
    using AABB = ToLoG::AABB<Point>;

    Point q(-10,4);

    Point p(1.5, 3);
    EXPECT_EQ(133.25, ToLoG::point_squared_distance(q, p));

    Segment seg(Point(-12,6), Point(5,6));
    EXPECT_EQ(4, ToLoG::point_squared_distance(q, seg));
    seg = Segment(Point(1,6), Point(5,6));
    EXPECT_EQ(125, ToLoG::point_squared_distance(q, seg));
    seg = Segment(Point(-11,0), Point(-9,8));
    EXPECT_EQ(0, ToLoG::point_squared_distance(q, seg));
}

