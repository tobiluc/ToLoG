#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

constexpr double eps = 1e-12;

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

}
