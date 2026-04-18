#include "ToLoG/math/epsilon.hpp"
#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

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

TEST(GeometryCoreTest, TetrahedronTest)
{
    using Point = ToLoG::Point<double,3>;

    Point a(0,0,0);
    Point b(0,0,1);
    Point c(1,0,0);
    Point d(0,1,0);

    ToLoG::Tetrahedron<Point> tet(a,b,c,d);

    ToLoG::incenter(a, b, c, d);

    EXPECT_NEAR(ToLoG::dihedral_angle(a, b, c, d), 0.5*M_PI, epsilon);
    EXPECT_NEAR(ToLoG::dihedral_angle(c, a, b, d), 0.5*M_PI, epsilon);
    EXPECT_NEAR(ToLoG::dihedral_angle(d, a, c, b), 0.5*M_PI, epsilon);

    EXPECT_NEAR(ToLoG::angle(b-a, c-a), 0.5*M_PI, epsilon);
}

}
