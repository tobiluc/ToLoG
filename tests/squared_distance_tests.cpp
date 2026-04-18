#include "ToLoG/math/epsilon.hpp"
#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

TEST(SquaredDistanceTest, PointPoint)
{
    using Point = ToLoG::Point<double, 2>;
    using Segment = ToLoG::Segment<Point>;
    using Triangle = ToLoG::Triangle<Point>;
    using Sphere = ToLoG::Sphere<Point>;
    using AABB = ToLoG::AABB<Point>;

    Point q(-10,4);
    Point p(1.5, 3);

    EXPECT_EQ(133.25, ToLoG::squared_distance(q, p));
}

TEST(SquaredDistanceTest, PointSegment)
{
    using Point = ToLoG::Point<double, 2>;
    using Segment = ToLoG::Segment<Point>;
    using Triangle = ToLoG::Triangle<Point>;
    using Sphere = ToLoG::Sphere<Point>;
    using AABB = ToLoG::AABB<Point>;

    Point q(-10,4);

    Segment seg(Point(-12,6), Point(5,6));
    EXPECT_EQ(4, ToLoG::squared_distance(q, seg));
    seg = Segment(Point(1,6), Point(5,6));
    EXPECT_EQ(125, ToLoG::squared_distance(q, seg));
    seg = Segment(Point(-11,0), Point(-9,8));
    EXPECT_EQ(0, ToLoG::squared_distance(q, seg));
}

TEST(SquaredDistanceTest, AABBAABB)
{
    using Point = ToLoG::Point<double,2>;
    using AABB = ToLoG::AABB<Point>;

    AABB b0({Point(0,0), Point(1,1)});
    AABB b1({Point(1.5,1), Point(2,2)});
    AABB b2({Point(0.5,0.5), Point(2,1)});
    AABB b3({Point(-10,-10), Point(-8,10)});

    EXPECT_NEAR(ToLoG::squared_distance(b0, b1), 0.25, epsilon);
    EXPECT_EQ(ToLoG::squared_distance(b0, b2), 0);
    EXPECT_EQ(ToLoG::squared_distance(b1, b2), 0);
    EXPECT_NEAR(ToLoG::squared_distance(b0, b3), 64, epsilon);
}

TEST(SquaredDistanceTest, PointTriangle)
{
    using FT = double;
    using Point = ToLoG::Point<FT,3>;
    using Triangle = ToLoG::Triangle<Point>;

    Point p(1,1,1);

    Triangle tri(Point(0,0,0),Point(0,0,1),Point(1,0,0));

    Point q(0.5,0,0.5);
    FT d2 = squared_distance(p, q);

    EXPECT_NEAR(ToLoG::squared_distance(p, tri), d2, epsilon);

    p = Point(0.2,0,0.3);
    auto bary = barycentric_coordinates(p, tri);
    EXPECT_GE(bary[0], 0);
    EXPECT_GE(bary[1], 0);
    EXPECT_GE(bary[2], 0);
    EXPECT_NEAR(ToLoG::squared_distance(p, tri), 0, epsilon);
}

}
