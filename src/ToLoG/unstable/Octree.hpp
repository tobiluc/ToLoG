#pragma once

#include <ToLoG/Core.hpp>
#include <cassert>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <array>

namespace ToLoG
{

using u8 = uint8_t;
using u32 = uint32_t;
using u64 = uint64_t;

// Encodes position and depth
using OctreeNodeCode = u64;

struct OctreeNode {
    OctreeNodeCode code;
    bool refined;
    std::array<u32, 8> children = {}; // UINT32_MAX means no child

    OctreeNode(OctreeNodeCode code) : code(code), refined(false) {
        children.fill(UINT32_MAX);
    }

};

class Octree3d
{
public:
    using FT = double;
    using Point = Point<FT, 3>;
    using AABB = AABB<Point>;

private:
    u32 max_depth_ = 5;
    u32 initial_resolution_ = 2;
    std::vector<OctreeNode> nodes_;
    std::unordered_map<OctreeNodeCode, u32> cube_index_map_;
    AABB bounds_;

public:
    Octree3d(AABB bounds, u32 initialResolution = 2, u32 maxDepth = 5);

    Octree3d() : Octree3d(AABB({Point::filled(0), Point::filled(1)})) {}

    inline bool isRefined(u32 idx) const {
        return nodes_[idx].refined;
    }

    inline OctreeNode& node(u32 idx) {
        return nodes_[idx];
    }

    u32 node_depth(u32 idx) const;

    inline u32 max_depth() const {
        return max_depth_;
    }

    inline bool contains_node(OctreeNodeCode code) const {
        return cube_index_map_.find(code) != cube_index_map_.end();
    }

    inline double node_scale_at_depth(u32 depth) const {
        return 1.0 / (initial_resolution_ << depth); // resolution * 2^depth
    }

    inline Point node_size_at_depth(u32 depth) const {
        double scale = node_scale_at_depth(depth);
        return {
            scale * (bounds_.max()[0] - bounds_.min()[0]),
            scale * (bounds_.max()[1] - bounds_.min()[1]),
            scale * (bounds_.max()[2] - bounds_.min()[2])
        };
    }

    inline const u32 node_index_from_code(OctreeNodeCode code) const {
        auto it = cube_index_map_.find(code);
        return (it != cube_index_map_.end())? it->second : UINT32_MAX;
    }

    inline u32 n_nodes() const {
        return nodes_.size();
    }

    /// Splits a Node into 8 children
    void refine_node(u32 idx);

    u32 locate(Point _q) const;

    AABB node_bounding_box(u32 idx) const;
};

}

