#include "ToLoG/tree/AABBTree.hpp"
#include <filesystem>
#include <gtest/gtest.h>
#include <ToLoG/tree/Octree.hpp>
#include <ToLoG/io/obj_writer.hpp>

namespace ToLoG
{

TEST(OctreeTest, BitShiftTest)
{
    EXPECT_FALSE(0b00100000&(1<<(7-0)));
    EXPECT_FALSE(0b00100000&(1<<(7-1)));
    EXPECT_TRUE(0b00100000&(1<<(7-2)));
    EXPECT_FALSE(0b00100000&(1<<(7-3)));
    EXPECT_FALSE(0b00100000&(1<<(7-4)));
    EXPECT_FALSE(0b00100000&(1<<(7-5)));
    EXPECT_FALSE(0b00100000&(1<<(7-6)));
    EXPECT_FALSE(0b00100000&(1<<(7-7)));
}

TEST(OctreeTest, InitialTreeTest)
{
    using Point = Point<float,3>;
    using Octree = Octree<Point>;

    AABB<Point> aabb({{0,0,0}, {1,1,1}});
    Octree tree(aabb, 2, 5);
    EXPECT_EQ(2*2*2, tree.n_nodes());
    EXPECT_FALSE(tree.locate(Point(-1,0,0)).has_value());
    EXPECT_EQ(tree.locate(Point(0.1,0.1,0.1)).value(), 0);
}

TEST(OctreeTest, ChildOrderTest)
{
    using Point = Point<float,3>;
    using Octree = Octree<Point>;
    using AABB = AABB<Point>;

    AABB aabb({{0,0,0}, {2,2,2}});
    Octree tree(aabb, 1, 5);
    std::vector<Octree::NodeSplit> splits;
    splits.push_back({.node_idx_ = 0, .children_mask_ = 0b11111111});
    tree.refine_tree(splits);
    EXPECT_EQ(tree.n_nodes(), 8);
    for (int i = 0; i < 8; ++i) {
        std::cerr << "Octree Child "<< i<<": "
            << tree.node_aabb(tree.node_coords(i)).min() << std::endl;
    }
}

TEST(OctreeTest, SimpleRefinementTest)
{
    using Point = Point<float,3>;
    using Octree = Octree<Point>;
    using AABB = AABB<Point>;

    AABB aabb({{0,0,0}, {1,1,1}});
    Octree tree(aabb, 2, 5);

    // Before the Refinement
    EXPECT_EQ(tree.node_coords(0).depth, 0);
    EXPECT_EQ(tree.node_aabb(tree.node_coords(0)), AABB({Point(0,0,0),Point(.5,.5,.5)}));
    EXPECT_TRUE(tree.locate(Point(0.3,0.3,0.3)).has_value());
    EXPECT_TRUE(tree.locate(Point(0.9,0.9,0.9)).has_value());

    // Split only the first node into 4 children.
    // Keep the next 6 nodes, but delete completeley the last node
    std::vector<Octree::NodeSplit> splits;
    splits.push_back({.node_idx_ = 0, .children_mask_ = 0b11110000});
    splits.push_back({.node_idx_ = 1, .children_mask_ = 0b00000000});
    splits.push_back({.node_idx_ = 2, .children_mask_ = 0b00000000});
    splits.push_back({.node_idx_ = 3, .children_mask_ = 0b00000000});
    splits.push_back({.node_idx_ = 4, .children_mask_ = 0b00000000});
    splits.push_back({.node_idx_ = 5, .children_mask_ = 0b00000000});
    splits.push_back({.node_idx_ = 6, .children_mask_ = 0b00000000});
    tree.refine_tree(splits);

    // We should now have
    // 4 + 6 = 10 nodes
    EXPECT_EQ(10, tree.n_nodes());

    // After the Refinement
    EXPECT_EQ(tree.node_coords(0).depth, 1);
    EXPECT_EQ(tree.node_aabb(tree.node_coords(0)), AABB({{0,0,0},{.25,.25,.25}}));

    EXPECT_FALSE(tree.locate(Point(0.3,0.3,0.3)).has_value());
    EXPECT_FALSE(tree.locate(Point(0.9,0.9,0.9)).has_value());
}

TEST(OctreeTest, SphereRefinementTest)
{
    using Point = ToLoG::Point<float,3>;
    using Octree = Octree<Point>;
    using AABB = ToLoG::AABB<Point>;

    ToLoG::Sphere<Point> sphere({0,0,0}, 1);

    uint32_t max_depth = 3;
    ToLoG::Octree tree(ToLoG::aabb(sphere).scaled(1.1), 3, max_depth);
    uint32_t depth = 0;
    while (depth < max_depth)
    {
        // Refine all Nodes inside a sphere
        std::vector<Octree::NodeSplit> splits;
        for (uint32_t i = 0; i < tree.n_nodes(); ++i) {
            uint8_t children_mask = 0;
            for (int j = 0; j < 8; ++j) {
                AABB child_aabb = tree.node_aabb(tree.node_child_coords(tree.node_coords(i), j));
                for (const auto& child_corner : child_aabb.corners()) {
                    if (squared_norm(child_corner) < 1.0f) {
                        children_mask |= (0b10000000>>j);
                        break;
                    }
                }
            }
            if (children_mask != 0) {
                splits.push_back({
                    .node_idx_=i,
                    .children_mask_=children_mask
                });
            }
        }
        tree.refine_tree(splits);
        ++depth;
    }

    // using Mesh = ToLoG::QuadMesh<Point>;
    // Mesh mesh = ToLoG::octree_to_polygon_mesh<Point>(tree);
    // ToLoG::IO::write_polygon_mesh_obj(
    //     std::filesystem::path(TESTS_OUTPUT_DIR)/"octree_sphere.obj", mesh);
}

TEST(OctreeTest, AdaptiveTest)
{
    using Point = ToLoG::Point<float,3>;
    using Octree = Octree<Point>;
    using AABB = ToLoG::AABB<Point>;

    uint32_t max_depth = 6;
    AABB aabb({{0,0,0},{1,2,1}});
    Octree tree(aabb, 1, max_depth);
    uint32_t depth = 0;
    while (depth < max_depth)
    {
        std::vector<Octree::NodeSplit> splits;
        float h = 2.0f / (1<<depth);

        for (uint32_t i = 0; i < tree.n_nodes(); ++i) {
            if (ToLoG::centroid(tree.node_aabb(tree.node_coords(i)))[1] < h) {
                splits.push_back({.node_idx_=i,.children_mask_=0b11111111});
            } else {
                splits.push_back({.node_idx_=i,.children_mask_=0});
            }
        }
        tree.refine_tree(splits);
        depth += 1;
    }
    // using Mesh = ToLoG::QuadMesh<Point>;
    // Mesh mesh = ToLoG::octree_to_polygon_mesh<Point>(tree);
    // ToLoG::IO::write_polygon_mesh_obj(
    //     std::filesystem::path(TESTS_OUTPUT_DIR)/"octree_adaptive.obj", mesh);
}

}
