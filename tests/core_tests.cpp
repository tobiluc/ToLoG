#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

TEST(CoreTest, TraitsTest3d)
{
    using Point = ToLoG::Point<double, 3>;
    using Segment = ToLoG::Segment<Point>;
    using Triangle = ToLoG::Triangle<Point>;
    using Sphere = ToLoG::Sphere<Point>;
    using AABB = ToLoG::AABB<Point>;

    static_assert(ToLoG::Traits<Point>::dim == 3);
    static_assert(std::is_same<ToLoG::Traits<Point>::value_type, double>::value);
    static_assert(std::is_same<ToLoG::Traits<Point>::vector_type, Point>::value);
}

TEST(CoreTest, PointAssignmentTest)
{
    using Point4d = ToLoG::Point<double,4>;
    using Point4f = ToLoG::Point<float,4>;
    using Point4i = ToLoG::Point<int,4>;

    Point4i a(1,2,3,4);
    Point4d b;
    b = a;
    Point4f c = b;

    EXPECT_EQ(a, b);
    EXPECT_EQ(a, c);
    EXPECT_EQ(b, c);

    b = a + c*3;
    EXPECT_EQ(b[0], 4);
    EXPECT_EQ(b[1], 8);
    EXPECT_EQ(b[2], 12);
    EXPECT_EQ(b[3], 16);
}

TEST(CoreTest, Point1)
{
    using P = Point<int,1>;
    P p(3);
    EXPECT_EQ(p[0], 3);
    P q(p);
    EXPECT_EQ(p, q);
}

}
