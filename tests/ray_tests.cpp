#include <gtest/gtest.h>
#include <ToLoG/Ray.hpp>

TEST(RayTest, RayTest1)
{
    using Point = ToLoG::Point<float,3>;
    using Ray = ToLoG::Ray<Point>;
    using Sphere = ToLoG::Sphere<Point>;

    Ray ray(Point(0,0,0), Point(1,0,0));
    EXPECT_EQ(ray.point(2.5), Point(2.5,0,0));

    Sphere sphere(Point(100,0,0), 1.5);
    const auto ts = ToLoG::ray_intersection_times(ray, sphere);
    EXPECT_EQ(98.5, ts[0]);
    EXPECT_EQ(101.5, ts[1]);
}
