#include <gtest/gtest.h>
#include <ToLoG/AABBTree.h>
#include <random>

namespace ToLoG
{
void naive_knn_search(const std::vector<Point<double,3>>& _pts,
                      const Point<double,3>& _q, const uint32_t _k,
                      std::vector<uint32_t>& _res)
{
    // Store all distances
    std::vector<double> d;
    d.reserve(_pts.size());
    for (const auto& p : _pts) {
        d.push_back((p - _q).squared_norm());
    }

    // Sort points
    std::vector<uint32_t> tmp(_pts.size());
    std::iota(tmp.begin(), tmp.end(), 0u);
    std::sort(tmp.begin(), tmp.end(), [&](const uint32_t& _i, const uint32_t& _j) {
        return d[_i] < d[_j];
    });

    // Returns k best
    _res.clear();
    _res.reserve(_k);
    for (uint32_t i = 0; i < _k; ++i) {
        _res.push_back(tmp[i]);
    }
}

TEST(TreeTest, KDTReeNNTest)
{
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> rand(-10.0, 10.0);

    std::vector<Point<double,3>> pts;
    for (int i = 0; i < 1000; ++i) {
        pts.emplace_back(rand(gen), rand(gen), rand(gen));
    }

    AABBTree<Point<double,3>> tree(pts, 8);

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
    std::cerr << "AABBTree KNN: " << dtaabb << "[µs]" << std::endl;
    std::cerr << "Naive KNN: " << dtnaive << "[µs]" << std::endl;
}

TEST(TreeTest, TreeTest2i)
{
    size_t n_points_sqrt = 6;
    std::vector<Point<int,2>> pts;
    for (int x = 0; x < n_points_sqrt; ++x) {
        for (int y = 0; y < n_points_sqrt; ++y) {
            pts.emplace_back(x, y);
        }
    }

    // Tree init
    AABBTree<Point<int,2>> tree(pts, 48);

    // Queries
    std::vector<uint32_t> res;
    for (int x = 1; x < n_points_sqrt-1; ++x) {
        for (int y = 1; y < n_points_sqrt-1; ++y) {
            Point<int,2> q(x,y);
            tree.k_nearest_neighbors(q, 9, res);
            for (int j = 0; j < res.size(); ++j) {
                EXPECT_LE(std::abs(pts[res[j]][0] - q[0]), 1);
                EXPECT_LE(std::abs(pts[res[j]][1] - q[1]), 1);
                //std::cerr << q.transpose() << ": " << pts[res[j]].transpose() << std::endl;
            }
        }

    }
}

TEST(TreeTest, PointBetweenTwoSpheresTest)
{
    using Point = Point<double,3>;
    using Sphere = Sphere<Point>;
    using Tree = AABBTree<Sphere>;

    Sphere s1(Point(-101,0,0), 100);
    Sphere s2(Point(5,0,0),2);

    EXPECT_EQ(Point(-201,-100,-100), s1.aabb().min());
    EXPECT_EQ(Point(-1,100,100), s1.aabb().max());
    EXPECT_EQ(s1.centroid(), s1.aabb().centroid());
    EXPECT_EQ(Point(3,-2,-2), s2.aabb().min());
    EXPECT_EQ(Point(7,2,2), s2.aabb().max());
    EXPECT_EQ(s2.centroid(), s2.aabb().centroid());

    Point q1(0,0,0);
    EXPECT_EQ(1, point_squared_distance(q1, s1));
    EXPECT_EQ(9, point_squared_distance(q1, s2));

    std::vector<Sphere> spheres = {s1, s2};
    Tree tree(spheres);
    EXPECT_EQ(1, tree.n_nodes());
    EXPECT_EQ(2, tree.n_primitives());

    std::vector<uint32_t> res;

    tree.k_nearest_neighbors(s1.centroid(), 1, res);
    EXPECT_EQ(0, res[0]);

    tree.k_nearest_neighbors(s2.centroid(), 1, res);
    EXPECT_EQ(1, res[0]);

    tree.k_nearest_neighbors(q1, 1, res);
    EXPECT_EQ(0, res[0]);
}

}
