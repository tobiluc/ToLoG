#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

constexpr double eps = std::numeric_limits<double>::epsilon();

TEST(CoreTest, BarycentricTest)
{
    using P = ToLoG::Point<double, 3>;
    using Tri = ToLoG::Triangle<P>;
    using Tet = ToLoG::Tetrahedron<P>;

    Tri tri(P(0,0.5,0), P(0,0,1), P(2,0,0));
    for (int i = 0; i < 3; ++i) {
        auto b = ToLoG::barycentric_coordinates(tri[i], tri);
        for (int j = 0; j < 3; ++j) {
            EXPECT_EQ(b[j], (i==j)? 1.0 : 0.0);
        }
    }

    Tet tet(P(0,0.25,0), P(0,0,3.5), P(10,0,0), P(0,9.5,0));
    for (int i = 0; i < 4; ++i) {
        auto b = ToLoG::barycentric_coordinates(tet[i], tet);
        for (int j = 0; j < 4; ++j) {
            EXPECT_EQ(b[j], (i==j)? 1.0 : 0.0);
        }
    }
}

}
