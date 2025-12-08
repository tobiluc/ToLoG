#include <gtest/gtest.h>
#include <ToLoG/enumerate.hpp>

using namespace ToLoG;

TEST(EnumerateTest, Test1)
{
    std::vector<std::string> strings = {
        "Apple", "Banana", "Coconut", "Door", "Euphemism", "Fragility"
    };
    int idx = 0;
    for (const auto& [i,s] : ToLoG::enumerate(strings)) {
        EXPECT_EQ(i, idx);
        EXPECT_EQ(strings[i], s);
        idx += 1;
    }
    EXPECT_EQ(strings.size(), idx);
}
