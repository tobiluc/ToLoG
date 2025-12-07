#pragma once

#include "IGRec/geometry/BoundingBox.h"
#include "IGRec/typedefs/TypedefsEigen.h"
#include "absl/container/flat_hash_map.h"
#include <cassert>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <array>

namespace IGRec::Geometry
{

using u8 = uint8_t;
using u32 = uint32_t;
using u64 = uint64_t;

// Encodes position and depth
using OctreeNodeCode = u64;

struct OctreeNode {
    OctreeNodeCode code;
    bool refined;
    bool active = true;
    std::array<u32, 8> children = {}; // UINT32_MAX means no child

    OctreeNode(OctreeNodeCode code) : code(code), refined(false), active(true) {
        children.fill(UINT32_MAX);
    }

};

class Octree
{
private:
    u32 max_depth_ = 5;
    u32 initial_resolution_ = 2;
    std::vector<OctreeNode> nodes_;
    absl::flat_hash_map<OctreeNodeCode, u32> cube_index_map_;
    BoundingBox bounds_;

public:
    Octree(BoundingBox bounds, u32 initialResolution = 2, u32 maxDepth = 5);

    Octree() : Octree(BoundingBox(0,0,0,1,1,1)) {}

    inline bool isRefined(u32 idx) const {
        return nodes_[idx].refined;
    }

    inline bool isActive(u32 idx) const {
        return nodes_[idx].active;
    }

    inline void deactivate(u32 idx) {
        nodes_[idx].active = false;
    }

    inline OctreeNode& node(u32 idx) {
        return nodes_[idx];
    }

    u32 depth(u32 idx) const;

    inline u32 max_depth() const {
        return max_depth_;
    }

    inline bool containsNode(OctreeNodeCode code) const {
        return cube_index_map_.find(code) != cube_index_map_.end();
    }

    inline double getNodeScaleAtDepth(u32 depth) const {
        return 1.0 / (initial_resolution_ << depth); // resolution * 2^depth
    }

    inline std::array<double,3> getNodeSizeAtDepth(u32 depth) const {
        double scale = getNodeScaleAtDepth(depth);
        auto treeSize = bounds_.size<Eig::Vec3d>();
        return {
            scale * treeSize[0],
            scale * treeSize[1],
            scale * treeSize[2]
        };
    }

    inline const u32 getNodeIndex(OctreeNodeCode code) const {
        auto it = cube_index_map_.find(code);
        return (it != cube_index_map_.end())? it->second : UINT32_MAX;
    }

    inline u32 n_nodes() const {
        return nodes_.size();
    }

    /// Splits a Node into 8 children
    void refineNode(u32 idx);

    u32 locate(double px, double py, double pz) const;

    BoundingBox node_bounding_box(u32 idx) const;
};

}

