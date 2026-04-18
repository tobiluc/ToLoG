#include "ToLoG/math/epsilon.hpp"
#include <gtest/gtest.h>
#include <ToLoG/Core.hpp>

namespace ToLoG
{

TEST(BarycentricTest, CornerBaryTest)
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

TEST(BarycentricTest, PointInTriangleTest)
{
    using P = ToLoG::Point<double, 3>;
    using Tri = ToLoG::Triangle<P>;

    Tri tri(P(0,0.5,0), P(0,0,1), P(2,0,0));
    P p = tri[0]*1.2 + tri[1]*0.3 + tri[2]*-0.5;

    auto bary = ToLoG::barycentric_coordinates(p, tri);
    EXPECT_NEAR(bary[0], 1.2, epsilon);
    EXPECT_NEAR(bary[1], 0.3, epsilon);
    EXPECT_NEAR(bary[2], -0.5, epsilon);

    P q = tri[0]*bary[0] + tri[1]*bary[1] + tri[2]*bary[2];
    EXPECT_NEAR(p[0], q[0], epsilon);
    EXPECT_NEAR(p[1], q[1], epsilon);
    EXPECT_NEAR(p[2], q[2], epsilon);
}

}
