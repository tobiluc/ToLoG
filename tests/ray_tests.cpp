#include <gtest/gtest.h>
#include <ToLoG/Ray.hpp>

TEST(RayTest, RayTest1)
{
    using Point = ToLoG::Point<float,3>;
    using Ray = ToLoG::Ray<Point>;

    Ray ray(Point(0,0,0), Point(1,0,0));
    EXPECT_EQ(ray.point(2.5), Point(2.5,0,0));
}

TEST(RayTest, RaySphereTest)
{
    using Point = ToLoG::Point<float,3>;
    using Ray = ToLoG::Ray<Point>;
    using Sphere = ToLoG::Sphere<Point>;

    Ray ray(Point(0,0,0), Point(1,0,0));

    // 2 intersections
    Sphere sphere(Point(100,0,0), 1.5);
    auto ts = ToLoG::ray_intersection_times(ray, sphere);
    EXPECT_EQ(ts.size(), 2);
    EXPECT_EQ(98.5, ts[0]);
    EXPECT_EQ(101.5, ts[1]);

    // 1 intersection
    sphere = Sphere(Point(100,10,0), 10);
    ts = ToLoG::ray_intersection_times(ray, sphere);
    EXPECT_EQ(ts.size(), 1);
    EXPECT_EQ(100.0, ts[0]);

    // 0 intersections
    sphere = Sphere(Point(100,10,0), 9);
    ts = ToLoG::ray_intersection_times(ray, sphere);
    EXPECT_EQ(ts.size(), 0);
}

TEST(RayTest, RayAABBTest)
{
    using Point = ToLoG::Point<float,3>;
    using Ray = ToLoG::Ray<Point>;
    using AABB = ToLoG::AABB<Point>;

    Ray ray(Point(0,0,0), Point(1,0,0));

    // 2 from outside
    AABB aabb({Point(50,-5,-5), Point(80, 5, 5)});
    auto ts = ToLoG::ray_intersection_times(ray, aabb);
    EXPECT_EQ(ts.size(), 2);
    EXPECT_EQ(50.0, ts[0]);
    EXPECT_EQ(80.0, ts[1]);

    // 2 from inside
    aabb = AABB({Point(-10,-5,-5), Point(20, 5, 5)});
    ts = ToLoG::ray_intersection_times(ray, aabb);
    EXPECT_EQ(ts.size(), 2);
    EXPECT_EQ(-10.0, ts[0]);
    EXPECT_EQ(20.0, ts[1]);

    // no intersection
    aabb = AABB({Point(50,3,-5), Point(80, 5, 5)});
    ts = ToLoG::ray_intersection_times(ray, aabb);
    EXPECT_EQ(ts.size(), 0);
}
