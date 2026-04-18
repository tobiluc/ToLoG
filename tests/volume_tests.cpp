#include "ToLoG/math/epsilon.hpp"
#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

TEST(VolumeTest, Triangle2dTest)
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

TEST(VolumeTest, TriangleArea3dTest)
{
    using P = ToLoG::Point<double,3>;
    using T = ToLoG::Triangle<P>;

    auto area3d = [](const P& A, const P& B, const P& C) -> double {
        return 0.5 * ToLoG::norm(ToLoG::cross((C-A),(B-A)));
    };

    P a(1,2,3);
    P b(0,-14,31.123);
    P c(-9,0,-3);

    EXPECT_NEAR(ToLoG::area(T(a,b,c)), area3d(a,b,c), epsilon);
    EXPECT_NEAR(ToLoG::area(T(a,c,b)), area3d(a,c,b), epsilon);
}

}
