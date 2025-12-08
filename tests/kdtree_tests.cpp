#include "naive_knn.hpp"
#include <gtest/gtest.h>
#include <ToLoG/unstable/KDTree.hpp>
#include <random>

namespace ToLoG
{

TEST(TreeTest, KDTReeNNTest)
{
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> rand(-10.0, 10.0);

    std::vector<Point<double,3>> pts;
    for (int i = 0; i < 1000; ++i) {
        pts.emplace_back(rand(gen), rand(gen), rand(gen));
    }

    KDTree<Point<double,3>> tree(pts, 8);

    uint32_t k = 30;
    long long dtnaive = 0, dtaabb = 0;
    std::chrono::steady_clock::time_point t1, t2;
    for (uint32_t i = 0; i < pts.size(); ++i) {
        Point<double,3> q = pts[i];

        t1 = std::chrono::steady_clock::now();
        std::vector<uint32_t> treeres;
        tree.k_nearest_neighbors(q.data(), k, treeres);
        EXPECT_EQ(k, treeres.size());
        t2 = std::chrono::steady_clock::now();
        dtaabb += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

        t1 = std::chrono::steady_clock::now();
        std::vector<uint32_t> naiveres;
        naive_knn_search(pts, q, k, naiveres);
        EXPECT_EQ(k, naiveres.size());
        t2 = std::chrono::steady_clock::now();
        dtnaive += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

        EXPECT_EQ(treeres, naiveres);
    }

    EXPECT_LT(dtaabb, dtnaive);
    std::cerr << "KDTree KNN: " << dtaabb << "[µs]" << std::endl;
    std::cerr << "Naive KNN: " << dtnaive << "[µs]" << std::endl;
}

}
