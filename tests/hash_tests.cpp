#include <gtest/gtest.h>
#include <ToLoG/HashMap.hpp>
#include <random>

namespace ToLoG
{

TEST(HashMapTest, Test1)
{
    HashMap<std::string, std::string> map;
    EXPECT_TRUE(map.empty());
    map["One"] = "Hello World";
    EXPECT_EQ(map.size(), 1);
    EXPECT_EQ(map.get("One")->get(), "Hello World");
    EXPECT_FALSE(map.erase("Two"));
    EXPECT_EQ(map.size(), 1);
    EXPECT_EQ(map.get("One")->get(), "Hello World");
    EXPECT_TRUE(map.erase("One"));
    EXPECT_TRUE(map.empty());
    EXPECT_FALSE(map.get("One").has_value());
}

TEST(HashMapTest, PerformanceTest)
{
    std::chrono::steady_clock::time_point t1, t2;
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> rand(-9999.0, 9999.0);
    size_t n = 10000000;

    HashMap<double, double> map1(n);
    std::unordered_map<double,double> map2(n);

    // Insertion
    std::vector<std::pair<double,double>> keyvalues;
    keyvalues.reserve(n);
    long long dt1 = 0, dt2 = 0;
    for (int i = 0; i < n; ++i) {
        double k = rand(gen);
        double v = rand(gen);
        keyvalues.emplace_back(k, v);

        t1 = std::chrono::steady_clock::now();
        map1[k] = v;
        t2 = std::chrono::steady_clock::now();
        dt1 += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        map2[k] = v;
        t2 = std::chrono::steady_clock::now();
        dt2 += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

    }
    EXPECT_EQ(n, map1.size());
    EXPECT_EQ(n, map2.size());

    std::cerr << "HashMap Insertion:" << std::endl;
    std::cerr << dt1 << "[µs] (HashMap)" << std::endl;
    std::cerr << dt2 << "[µs] (std::unordered_map)" << std::endl;

    // Find
    dt1 = dt2 = 0;
    for (int i = 0; i < n; ++i) {
        double k = keyvalues[i].first;

        t1 = std::chrono::steady_clock::now();
        double v = map1.get(k)->get();
        EXPECT_EQ(v, keyvalues[i].second);
        t2 = std::chrono::steady_clock::now();
        dt1 += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        v = map2.at(k);
        EXPECT_EQ(v, keyvalues[i].second);
        t2 = std::chrono::steady_clock::now();
        dt2 += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

    }

    std::cerr << "HashMap Finding:" << std::endl;
    std::cerr << dt1 << "[µs] (HashMap)" << std::endl;
    std::cerr << dt2 << "[µs] (std::unordered_map)" << std::endl;

    // Deletion
    dt1 = dt2 = 0;
    for (int i = 0; i < n; ++i) {
        double k = keyvalues[i].first;

        t1 = std::chrono::steady_clock::now();
        map1.erase(k);
        t2 = std::chrono::steady_clock::now();
        dt1 += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        map2.erase(k);
        t2 = std::chrono::steady_clock::now();
        dt2 += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

    }

    std::cerr << "HashMap Deletion:" << std::endl;
    std::cerr << dt1 << "[µs] (HashMap)" << std::endl;
    std::cerr << dt2 << "[µs] (std::unordered_map)" << std::endl;

    // Check empty
    EXPECT_TRUE(map1.empty());
    EXPECT_TRUE(map2.empty());
}

}
