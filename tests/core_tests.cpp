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

}
