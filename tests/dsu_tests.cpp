#include <gtest/gtest.h>
#include <ToLoG/DSU.hpp>

using namespace ToLoG;

TEST(DSUTest, EachToTheirOwnTest)
{
    size_t n = 1000;
    DSU dsu(n);
    EXPECT_EQ(n, dsu.n_connected_components());
    for (uint32_t i = 0; i < n; ++i) {
        EXPECT_EQ(dsu.parent(i), i);
        EXPECT_EQ(dsu.root(i), i);
        EXPECT_EQ(dsu.size(i), 1);
    }
}

TEST(DSUTest, EvenAndOddTest)
{
    size_t n = 10000;
    DSU dsu(n);
    for (int i = 0; i < n; ++i) {
        (i%2)? dsu.unite(1,i) : dsu.unite(0,i);
    }
    EXPECT_EQ(2, dsu.n_connected_components());
    for (uint32_t i = 0; i < n; ++i) {
        EXPECT_EQ(dsu.size(i), n/2);
    }
}
