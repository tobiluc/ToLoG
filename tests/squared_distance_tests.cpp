#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

constexpr double eps = std::numeric_limits<double>::epsilon();

TEST(SquaredDistanceTest, PointDistanceTest2d)
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

TEST(SquaredDistanceTest, AABBTest)
{
    using Point = ToLoG::Point<double,2>;
    using AABB = ToLoG::AABB<Point>;

    AABB b0({Point(0,0), Point(1,1)});
    AABB b1({Point(1.5,1), Point(2,2)});
    AABB b2({Point(0.5,0.5), Point(2,1)});
    AABB b3({Point(-10,-10), Point(-8,10)});

    EXPECT_NEAR(ToLoG::squared_distance(b0, b1), 0.25, eps);
    EXPECT_EQ(ToLoG::squared_distance(b0, b2), 0);
    EXPECT_EQ(ToLoG::squared_distance(b1, b2), 0);
    EXPECT_NEAR(ToLoG::squared_distance(b0, b3), 64, eps);
}

}
