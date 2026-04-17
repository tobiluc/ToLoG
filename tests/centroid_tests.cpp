#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

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

}
