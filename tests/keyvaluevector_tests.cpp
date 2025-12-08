#include <gtest/gtest.h>
#include <ToLoG/KeyValueVector.hpp>

using namespace ToLoG;

TEST(KeyValueVectorTest, Test1)
{
    // 10 -> "Hello World"
    // 0 -> ""
    // 10 -> "Ouagadougou"
    KeyValueVector<int,std::string> kvs;
    EXPECT_TRUE(kvs.empty());
    kvs.set(10, "Hello World");
    EXPECT_EQ(1, kvs.size());
    EXPECT_EQ("Hello World", kvs.at(10));
    EXPECT_EQ(10, kvs.at_index(0).first);
    EXPECT_EQ("Hello World", kvs.at_index(0).second);
    EXPECT_EQ("", kvs[0]); // implicitly add new element
    EXPECT_EQ(2, kvs.size());
    kvs[10] = "Ouagadougou"; // overwrite
    EXPECT_EQ(2, kvs.size());
    EXPECT_EQ("Ouagadougou", kvs[10]);
    EXPECT_EQ(10, kvs.at_index(0).first);
    EXPECT_EQ("Ouagadougou", kvs.at_index(0).second);
    EXPECT_EQ(0, kvs.at_index(1).first);
    EXPECT_EQ("", kvs.at_index(1).second);
    kvs.clear();
    EXPECT_TRUE(kvs.empty());
}
