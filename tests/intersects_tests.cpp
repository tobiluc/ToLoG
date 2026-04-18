#include "epsilon.hpp"
#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

TEST(IntersectsTest, EllipsoidTest)
{
    using Point = ToLoG::Point<double,3>;
    using Axis = Point;
    using Ellipsoid = ToLoG::Ellipsoid<Point>;
    using AABB = ToLoG::AABB<Point>;

    Ellipsoid e(Point(1,0,0), {Axis(1,0,0),Axis(0,2,0),Axis(0,0,3)});
    EXPECT_NEAR(e.radius(0), 1, epsilon);
    EXPECT_NEAR(e.radius(1), 2, epsilon);
    EXPECT_NEAR(e.radius(2), 3, epsilon);
    EXPECT_TRUE(ToLoG::intersects(e, e.center()));
    EXPECT_TRUE(ToLoG::intersects(e, Point(0.1,0.2,0.3)));
    EXPECT_FALSE(ToLoG::intersects(e, Point(0,2,3)));

    AABB bbox = aabb(e);
    EXPECT_NEAR(bbox.min()[0], 0, epsilon);
    EXPECT_NEAR(bbox.max()[0], 2, epsilon);
    EXPECT_NEAR(bbox.min()[1], -2, epsilon);
    EXPECT_NEAR(bbox.max()[1], 2, epsilon);
    EXPECT_NEAR(bbox.min()[2], -3, epsilon);
    EXPECT_NEAR(bbox.max()[2], 3, epsilon);
}

TEST(IntersectsTest, PointSegment3d)
{
    using P = ToLoG::Point<double,3>;
    using S = ToLoG::Segment<P>;

    S seg(P(1,2,3), P(2,3,4));
    EXPECT_TRUE(intersects(seg, P(1.5,2.5,3.5)));
    EXPECT_TRUE(intersects(seg, P(1,2,3)));
    EXPECT_TRUE(intersects(seg, P(2,3,4)));
    EXPECT_FALSE(intersects(seg, P(1.6,2.5,3.4)));
    EXPECT_FALSE(intersects(seg, P(4,6,8)));
}

}
