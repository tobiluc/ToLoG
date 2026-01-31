#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>
#include <ToLoG/math/winding_number.hpp>
#include <ToLoG/predicates/ExactPredicates.hpp>

TEST(GeometryCoreTest, WindingNumber3dTest)
{
    using Point = ToLoG::Point<double,3>;
    using Triangle = ToLoG::Triangle<Point>;

    // Tetrahedron vertices
    Point v0(0,0,0);
    Point v1(0,0,1);
    Point v2(1,0,0);
    Point v3(0,1,0);

    // Oriented faces (outward)
    std::vector<Triangle> tris = {
        Triangle(v0, v1, v2),
        Triangle(v0, v3, v1),
        Triangle(v0, v2, v3),
        Triangle(v1, v3, v2)
    };

    EXPECT_EQ(ToLoG::sign_orient3d(v0.data(),v1.data(),v2.data(),v3.data()), ToLoG::ORI::CCW);
    EXPECT_EQ(ToLoG::sign_orient3d(v0.data(),v3.data(),v1.data(),v2.data()), ToLoG::ORI::CCW);
    EXPECT_EQ(ToLoG::sign_orient3d(v0.data(),v2.data(),v3.data(),v1.data()), ToLoG::ORI::CCW);
    EXPECT_EQ(ToLoG::sign_orient3d(v1.data(),v3.data(),v2.data(),v0.data()), ToLoG::ORI::CCW);

    auto wn = [&](const Point& _p) {
        double wn(0);
        for (const auto& t : tris) {
            wn += winding_number(_p, t);
        }
        return std::abs(wn); //bool inside = std::abs(wn) > 0.5;
    };
    constexpr double eps = std::numeric_limits<double>::epsilon();


    EXPECT_NEAR(wn(Point(0.1,0.1,0.1)), 1.0, eps); // Inside
    EXPECT_NEAR(wn(Point(5,5,5)), 0.0, eps); // Outside
    EXPECT_NEAR(wn(Point(-99,0,-99)), 0.0, eps); // Outside

}

TEST(GeometryCoreTest, WindingNumber2dTest)
{
    using Point = ToLoG::Point<double,2>;
    using Segment = ToLoG::Segment<Point>;

    // Square vertices
    Point v0(0,0);
    Point v1(1,0);
    Point v2(1,1);
    Point v3(0,1);

    // Square
    std::vector<Segment> square = {
        Segment(v0, v1),
        Segment(v1, v2),
        Segment(v2, v3),
        Segment(v3, v0)
    };

    auto wn = [&](const Point& _p) {
        double wn(0);
        for (const auto& s : square) {
            wn += winding_number(_p, s);
        }
        return std::abs(wn); //bool inside = std::abs(wn) > 0.5;
    };
    constexpr double eps = std::numeric_limits<double>::epsilon();

    EXPECT_NEAR(wn(Point(0.5,0.5)), 1.0, eps); // Inside
    EXPECT_NEAR(wn(Point(0.1,0.9)), 1.0, eps); // Inside
    EXPECT_NEAR(wn(Point(5,5)), 0.0, eps); // Outside
    EXPECT_NEAR(wn(Point(-1000,0.5)), 0.0, eps); // Outside
    EXPECT_NEAR(wn(Point(0.5,1.01)), 0.0, eps); // Outside
}
