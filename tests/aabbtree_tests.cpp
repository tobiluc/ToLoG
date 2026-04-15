#include <gtest/gtest.h>
#include <ToLoG/tree/AABBTree.hpp>
#include <random>
#include "ToLoG/HashMap.hpp"
#include "ToLoG/io/ply_writer.hpp"
#include "naive_knn.hpp"
#include <ToLoG/io/obj_writer.hpp>

namespace ToLoG
{

TEST(TreeTest, AABBTReeNNTest)
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
        std::vector<uint32_t> treeres = tree.k_nearest_neighbors(q.data(), k);
        //tree.k_nearest_neighbors(q.data(), k, treeres);
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
    using Point = Point<int,2>;

    size_t n_points_sqrt = 6;
    std::vector<Point> pts;
    for (int x = 0; x < n_points_sqrt; ++x) {
        for (int y = 0; y < n_points_sqrt; ++y) {
            pts.emplace_back(x, y);
        }
    }

    // Tree init
    AABBTree<Point> tree(pts, 48);

    // Queries
    std::vector<uint32_t> res;
    for (int x = 1; x < n_points_sqrt-1; ++x) {
        for (int y = 1; y < n_points_sqrt-1; ++y) {
            Point q(x,y);
            tree.k_nearest_neighbors(q, 9, res);
            for (int j = 0; j < res.size(); ++j) {
                EXPECT_LE(std::abs(pts[res[j]][0] - q[0]), 1);
                EXPECT_LE(std::abs(pts[res[j]][1] - q[1]), 1);
                //std::cerr << q.transpose() << ": " << pts[res[j]].transpose() << std::endl;
            }
        }
    }

    for (int i = 0; i < pts.size(); ++i) {
        auto j = tree.locate(pts[i]);
        EXPECT_TRUE(j.has_value());
        EXPECT_EQ(j.value(), i);
    }
}

TEST(TreeTest, PointBetweenTwoSpheresTest)
{
    using Point = Point<double,3>;
    using Sphere = Sphere<Point>;
    using Tree = AABBTree<Sphere>;

    Sphere s1(Point(-101,0,0), 100);
    Sphere s2(Point(5,0,0),2);

    EXPECT_EQ(Point(-201,-100,-100), ToLoG::aabb(s1).min());
    EXPECT_EQ(Point(-1,100,100), ToLoG::aabb(s1).max());
    EXPECT_EQ(s1.center(), ToLoG::centroid(ToLoG::aabb(s1)));
    EXPECT_EQ(Point(3,-2,-2), ToLoG::aabb(s2).min());
    EXPECT_EQ(Point(7,2,2), ToLoG::aabb(s2).max());
    EXPECT_EQ(s2.center(), ToLoG::centroid(ToLoG::aabb(s2)));

    Point q1(0,0,0);
    EXPECT_EQ(1, squared_distance(q1, s1));
    EXPECT_EQ(9, squared_distance(q1, s2));

    std::vector<Sphere> spheres = {s1, s2};
    Tree tree(spheres);
    EXPECT_EQ(1, tree.n_nodes());
    EXPECT_EQ(2, tree.n_primitives());

    std::vector<uint32_t> res;

    tree.k_nearest_neighbors(s1.center(), 1, res);
    EXPECT_EQ(0, res[0]);

    tree.k_nearest_neighbors(s2.center(), 1, res);
    EXPECT_EQ(1, res[0]);

    tree.k_nearest_neighbors(q1, 1, res);
    EXPECT_EQ(0, res[0]);

    EXPECT_TRUE(tree.locate(Point(-3,0,0)).has_value());
    EXPECT_EQ(tree.locate(Point(-3,0,0)).value(), 0);
    EXPECT_TRUE(tree.locate(Point(4,0,0)).has_value());
    EXPECT_EQ(tree.locate(Point(4,0,0)).value(), 1);
    EXPECT_FALSE(tree.locate(Point(0,0,0)).has_value());
}

TEST(TreeTest, TriangleTreeAndSegmentsTest)
{
    using Point = Point<double,3>;
    using Segment = Segment<Point>;
    using Triangle = Triangle<Point>;
    using Tree = AABBTree<Triangle>;

    Tree tree;
    std::vector<Triangle> tris;
    tris.emplace_back(Point(0,0,0),Point(0,0,1),Point(1,0,0));
    tris.emplace_back(Point(0,0,1),Point(1,0,0),Point(1,0,1));
    tris.emplace_back(Point(0,3,0),Point(0,3,1),Point(1,3,0));
    tree.build_tree(tris);

    Segment seg(Point(0.5,2,0.5), Point(0.5,-2,0.5));

    EXPECT_TRUE(intersects(tris[0], seg));
    EXPECT_TRUE(intersects(tris[1], seg));
    EXPECT_FALSE(intersects(tris[2], seg));

    auto inds = tree.intersecting(seg);
    EXPECT_EQ(inds.size(), 2);
}

}
