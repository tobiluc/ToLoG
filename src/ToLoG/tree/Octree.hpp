#pragma once

#include <ToLoG/HashMap.hpp>
#include <ToLoG/Core.hpp>
#include <cassert>
#include <cstdint>
#include <vector>

namespace ToLoG
{

/*
 * Sparse Octree. Modification of version by @sraimondi in Snail Field
 */
class Octree
{
public:
    // Encodes position and depth
    using NodeCode = uint64_t;

    using NodeIndex = uint32_t;

    // The Node is split according to the children mask.
    // The i-th bit of the mask tells us if the i-th child exists.
    // If the mask is 0, the node is kept as-is
    struct NodeSplit {
        NodeIndex node_idx_;
        uint8_t children_mask_ = 0b11111111;
    };

    typedef struct {
        uint32_t x;
        uint32_t y;
        uint32_t z;
        uint32_t depth;
    } NodeCoords;

public:
    using FT = float;
    using Point = Point<FT, 3>;
    using AABB = AABB<Point>;

private:
    uint32_t max_depth_ = 5;
    uint32_t initial_resolution_ = 2;
    std::vector<NodeCode> codes_;
    HashMap<NodeCode, NodeIndex> code_index_map_;
    AABB tree_bounds_;

public:
    Octree(AABB _tree_bounds, uint32_t _initial_resolution = 2, uint32_t _max_depth = 5);

    Octree() : Octree(AABB({filled<Point>(0), filled<Point>(1)})) {}

    /*
     * Refines the tree according to the given node splits.
     * Nodes that are not present in the splits are deleted. If a node
     * should be kept, but not split, set its mask to 0.
     */
    void refine_tree(
        const std::vector<NodeSplit>& _splits
    );

    std::optional<NodeIndex> locate(Point _q) const;

    AABB node_aabb(const NodeCoords& _coords) const;

    NodeCoords node_coords(NodeIndex _idx) const;

    // Integer Coordinates of a nodes' child
    static Octree::NodeCoords node_child_coords(
        const Octree::NodeCoords& _node_coords,
        Octree::NodeIndex _child_idx);

    Point node_size_at_depth(uint32_t depth) const;

    inline uint32_t max_depth() const {
        return max_depth_;
    }

    inline size_t n_nodes() const {
        return codes_.size();
    }

    inline FT node_scale_at_depth(uint32_t depth) const {
        return static_cast<FT>(1.0) / (initial_resolution_ << depth); // resolution * 2^depth
    }
};

}

