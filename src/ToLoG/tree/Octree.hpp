#pragma once

#include "ToLoG/HashMap.hpp"
#include <ToLoG/Core.hpp>
#include <cassert>
#include <cstdint>
#include <vector>

namespace ToLoG
{

/*
 * Sparse Octree
 */
class Octree
{
public:
    // Encodes position and depth
    using NodeCode = uint64_t;
    using NodeIndex = uint32_t;

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

    inline uint32_t max_depth() const {
        return max_depth_;
    }

    inline FT node_scale_at_depth(uint32_t depth) const {
        return static_cast<FT>(1.0) / (initial_resolution_ << depth); // resolution * 2^depth
    }

    inline Point node_size_at_depth(uint32_t depth) const {
        FT scale = node_scale_at_depth(depth);
        return {
            scale * (tree_bounds_.max()[0] - tree_bounds_.min()[0]),
            scale * (tree_bounds_.max()[1] - tree_bounds_.min()[1]),
            scale * (tree_bounds_.max()[2] - tree_bounds_.min()[2])
        };
    }

    inline size_t n_nodes() const {
        return codes_.size();
    }

    /*
     * Refines
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
};

}

