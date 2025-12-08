#include <ToLoG/unstable/Octree.hpp>
#include <iostream>

namespace ToLoG
{

#define EPS 1e-12

#define NUM_NODE_CHILDREN 8
#define NUM_NODE_NEIGHBORS 6

#define NUM_DEPTH_BITS 6
#define NUM_MORTON_BITS 58

u64 expandBits(u32 v) {
    v = (v | (v << 16)) & 0x030000FF;
    v = (v | (v << 8)) & 0x0300F00F;
    v = (v | (v << 4)) & 0x030C30C3;
    v = (v | (v << 2)) & 0x09249249;
    return v;
}

OctreeNodeCode encode(u32 x, u32 y, u32 z, u32 depth) {
    assert(depth < (1u << NUM_DEPTH_BITS));

    u64 morton = (expandBits(x) << 0)
                 | (expandBits(y) << 1)
                 | (expandBits(z) << 2);
    return ((u64)(depth) << NUM_MORTON_BITS)
           | (morton & ((1ULL << NUM_MORTON_BITS) - 1));
}

u32 compactBits(u64 v) {
    v &= 0x09249249;
    v = (v ^ (v >> 2)) & 0x030C30C3;
    v = (v ^ (v >> 4)) & 0x0300F00F;
    v = (v ^ (v >> 8)) & 0x030000FF;
    v = (v ^ (v >> 16)) & 0x000003FF;
    return (u32)v;
}

void decode(OctreeNodeCode code, u32& x, u32& y, u32& z, u32& depth) {
    u64 morton = code & ((1ULL << NUM_MORTON_BITS) - 1);
    x = compactBits(morton >> 0);
    y = compactBits(morton >> 1);
    z = compactBits(morton >> 2);
    depth = (u32)(code >> NUM_MORTON_BITS);
}

Octree3d::Octree3d(AABB bounds, u32 initialResolution, u32 maxDepth)
    : bounds_(bounds), max_depth_(maxDepth),
    initial_resolution_(initialResolution)
{
    assert((initialResolution << maxDepth) <= 1024);

    // Initialize root nodes
    u32 capacity = pow(initialResolution,3);
    nodes_.reserve(capacity);
    cube_index_map_.reserve(capacity);
    for (u32 z = 0; z < initialResolution; ++z) {
        for (u32 y = 0; y < initialResolution; ++y) {
            for (u32 x = 0; x < initialResolution; ++x) {
                OctreeNodeCode code = encode(x, y, z, 0);
                nodes_.emplace_back(code);
                cube_index_map_[code] = nodes_.size()-1;
            }
        }
    }
}

void Octree3d::refine_node(u32 idx) {
    if (idx == UINT32_MAX) {return;}

    OctreeNodeCode code = nodes_[idx].code;
    u32 x, y, z, depth;
    decode(code, x, y, z, depth);
    if (depth >= max_depth_) {return;}

    u32 childDepth = depth + 1;
    for (int i = 0; i < NUM_NODE_CHILDREN; ++i) {
        u32 cx = x * 2 + ((i >> 0) & 1);
        u32 cy = y * 2 + ((i >> 1) & 1);
        u32 cz = z * 2 + ((i >> 2) & 1);
        OctreeNodeCode childCode = encode(cx, cy, cz, childDepth);
        nodes_.emplace_back(childCode);
        u32 childIdx = nodes_.size()-1;
        nodes_[idx].children[i] = childIdx;
        cube_index_map_[childCode] = childIdx;
    }
    nodes_[idx].refined = true;
}

u32 Octree3d::locate(Point _q) const {

    // Normalize within tree bounds
    _q[0] = (_q[0] - bounds_.min()[0]) / (bounds_.max()[0]-bounds_.min()[0]);
    _q[1] = (_q[1] - bounds_.min()[1]) / (bounds_.max()[1]-bounds_.min()[1]);
    _q[2] = (_q[2] - bounds_.min()[2]) / (bounds_.max()[2]-bounds_.min()[2]);

    // Check if outside tree bounds
    if (_q[0] < -EPS || _q[0] > 1.0+EPS
        || _q[1] < -EPS || _q[1] > 1.0+EPS
        || _q[2] < -EPS || _q[2] > 1.0+EPS) {
        return UINT32_MAX;
    }

    // Clamp
    _q[0] = std::clamp(_q[0], 0.0, 1.0);
    _q[1] = std::clamp(_q[1], 0.0, 1.0);
    _q[2] = std::clamp(_q[2], 0.0, 1.0);

    // Find which root cell this falls into
    u32 x = (u32)(_q[0] * initial_resolution_);
    u32 y = (u32)(_q[1] * initial_resolution_);
    u32 z = (u32)(_q[2] * initial_resolution_);

    OctreeNodeCode code = encode(x, y, z, 0);
    u32 nodeIdx = node_index_from_code(code);

    if (nodeIdx == UINT32_MAX) {return UINT32_MAX;}

    // Descend as far as possible
    while (true) {
        const OctreeNode& node = nodes_[nodeIdx];

        u32 cx, cy, cz, depth;
        decode(node.code, cx, cy, cz, depth);

        if (depth >= max_depth_) {break;}

        // Compute the relative position of the point in this node
        double scale = node_scale_at_depth(depth);
        double localX = (_q[0] - cx * scale) / scale;
        double localY = (_q[1] - cy * scale) / scale;
        double localZ = (_q[2] - cz * scale) / scale;

        u32 childIndex =
            ((localZ >= 0.5) << 2) |
            ((localY >= 0.5) << 1) |
            ((localX >= 0.5) << 0);

        u32 childIdx = node.children[childIndex];
        if (childIdx == UINT32_MAX) {break;}

        nodeIdx = childIdx;
    }

    return nodeIdx;
}

u32 Octree3d::node_depth(u32 idx) const
{
    u32 x, y, z, d;
    decode(nodes_[idx].code, x, y, z, d);
    return d;
}

Octree3d::AABB Octree3d::node_bounding_box(u32 idx) const {
    if (idx == UINT32_MAX) {return AABB();}
    u32 cx, cy, cz, d;
    decode(nodes_[idx].code, cx, cy, cz, d);
    Point size = node_size_at_depth(d);
    Point origin(
        bounds_.min()[0] + cx * size[0],
        bounds_.min()[1] + cy * size[1],
        bounds_.min()[2] + cz * size[2]
    );
    return AABB({origin, origin+size});
}

}
