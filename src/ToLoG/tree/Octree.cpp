#include <ToLoG/tree/Octree.hpp>

namespace ToLoG
{

#define EPS 1e-12

#define NUM_NODE_CHILDREN 8
#define NUM_NODE_NEIGHBORS 6

#define NUM_DEPTH_BITS 6
#define NUM_MORTON_BITS 58

// If Shifts are 2,1,0, children are sorted lexographically
// matching AABB.corners()
// For 0,1,2 they are sorted from the right instead
static constexpr inline uint8_t MORTON_SHIFTS[3] = {2,1,0};

static constexpr inline uint64_t expand_bits(uint32_t v)
{
    v = (v | (v << 16)) & 0x030000FF;
    v = (v | (v << 8)) & 0x0300F00F;
    v = (v | (v << 4)) & 0x030C30C3;
    v = (v | (v << 2)) & 0x09249249;
    return v;
}

static constexpr inline Octree::NodeCode encode(const Octree::NodeCoords& _coords)
{
    assert(depth < (1u << NUM_DEPTH_BITS));

    uint64_t morton = (expand_bits(_coords.x) << MORTON_SHIFTS[0])
                    | (expand_bits(_coords.y) << MORTON_SHIFTS[1])
                    | (expand_bits(_coords.z) << MORTON_SHIFTS[2]);
    return ((uint64_t)(_coords.depth) << NUM_MORTON_BITS)
           | (morton & ((1ULL << NUM_MORTON_BITS) - 1));
}

static constexpr inline uint32_t compact_bits(uint64_t v)
{
    v &= 0x09249249;
    v = (v ^ (v >> 2)) & 0x030C30C3;
    v = (v ^ (v >> 4)) & 0x0300F00F;
    v = (v ^ (v >> 8)) & 0x030000FF;
    v = (v ^ (v >> 16)) & 0x000003FF;
    return (uint32_t)v;
}

static constexpr inline Octree::NodeCoords decode(Octree::NodeCode code)
{
    uint64_t morton = code & ((1ULL << NUM_MORTON_BITS) - 1);
    return {
        .x = compact_bits(morton >> MORTON_SHIFTS[0]),
        .y = compact_bits(morton >> MORTON_SHIFTS[1]),
        .z = compact_bits(morton >> MORTON_SHIFTS[2]),
        .depth = (uint32_t)(code >> NUM_MORTON_BITS)
    };
}

Octree::Octree(AABB _tree_bounds, uint32_t _initial_resolution, uint32_t _max_depth)
    : tree_bounds_(_tree_bounds), max_depth_(_max_depth),
    initial_resolution_(_initial_resolution)
{
    assert((_initial_resolution << _max_depth) <= 1024);

    // Initialize root nodes
    uint32_t capacity = pow(_initial_resolution,3);
    codes_.reserve(capacity);
    code_index_map_.reserve(capacity);
    for (uint32_t z = 0; z < _initial_resolution; ++z) {
        for (uint32_t y = 0; y < _initial_resolution; ++y) {
            for (uint32_t x = 0; x < _initial_resolution; ++x) {
                NodeCode code = encode(NodeCoords{.x=x, .y=y, .z=z, .depth=0});
                code_index_map_[code] = codes_.size();
                codes_.emplace_back(code);
            }
        }
    }
}

void Octree::refine_tree(
    const std::vector<NodeSplit>& _splits
    )
{
    std::vector<NodeCode> new_codes;
    new_codes.reserve(_splits.size()*NUM_NODE_CHILDREN);

    for (const auto& split : _splits)
    {
        // No new children, we just keep the node
        if (split.children_mask_ == 0) {
            new_codes.emplace_back(codes_[split.node_idx_]);
            continue;
        }

        // Get Node coordinates
        NodeCode code = codes_[split.node_idx_];
        NodeCoords coords = decode(code);

        // If the max depth has been reached,
        // we keep the node as-is
        if (coords.depth >= max_depth_) {
            new_codes.emplace_back(code);
            continue;
        }

        // Add a Child Node for each set bit in the children mask
        for (uint8_t i = 0; i < NUM_NODE_CHILDREN; ++i) {
            // Only add child if mask bit is 1
            bool has_child = split.children_mask_ & (1<<(NUM_NODE_CHILDREN-1-i));
            if (!has_child) {continue;}

            // Remember the new child node
            NodeCode child_code = encode(node_child_coords(coords, i));
            new_codes.emplace_back(child_code);
        }
    }

    // Rebuild tree
    codes_ = std::move(new_codes);
    code_index_map_.clear();
    code_index_map_.reserve(codes_.size());
    for (NodeIndex i = 0; i < codes_.size(); ++i) {
        code_index_map_[codes_[i]] = i;
    }
}

std::optional<Octree::NodeIndex> Octree::locate(Point _q) const
{
    // Normalize within tree bounds
    _q[0] = (_q[0] - tree_bounds_.min()[0]) / (tree_bounds_.max()[0]-tree_bounds_.min()[0]);
    _q[1] = (_q[1] - tree_bounds_.min()[1]) / (tree_bounds_.max()[1]-tree_bounds_.min()[1]);
    _q[2] = (_q[2] - tree_bounds_.min()[2]) / (tree_bounds_.max()[2]-tree_bounds_.min()[2]);

    // Check if outside tree bounds
    if (_q[0] < -EPS || _q[0] > 1.0+EPS
        || _q[1] < -EPS || _q[1] > 1.0+EPS
        || _q[2] < -EPS || _q[2] > 1.0+EPS) {
        return std::nullopt;
    }

    // Clamp
    _q[0] = std::clamp<FT>(_q[0], 0.0, 1.0);
    _q[1] = std::clamp<FT>(_q[1], 0.0, 1.0);
    _q[2] = std::clamp<FT>(_q[2], 0.0, 1.0);

    // Find which root cell this falls into
    NodeCoords node_coords = {
        .x = static_cast<uint32_t>(_q[0] * initial_resolution_),
        .y = static_cast<uint32_t>(_q[1] * initial_resolution_),
        .z = static_cast<uint32_t>(_q[2] * initial_resolution_),
        .depth = 0
    };
    NodeCode code = encode(node_coords);
    auto node_idx_opt = code_index_map_.get(code);
    if (node_idx_opt.has_value()) {return node_idx_opt.value();}

    // If there is a node at this depth, we descend the tree
    while (node_coords.depth < max_depth_)
    {
        node_coords.depth += 1;
        uint32_t resolution = initial_resolution_ << node_coords.depth;

        // Compute integer coordinates at this depth
        node_coords.x = static_cast<uint32_t>(_q[0] * resolution);
        node_coords.y = static_cast<uint32_t>(_q[1] * resolution);
        node_coords.z = static_cast<uint32_t>(_q[2] * resolution);

        NodeCode child_code = encode(node_coords);

        auto child_idx_opt = code_index_map_.get(child_code);
        if (child_idx_opt.has_value()) {
            return child_idx_opt.value();
        }
    }

    return std::nullopt;
}

Octree::AABB Octree::node_aabb(const NodeCoords& _coords) const
{
    Octree::Point size = node_size_at_depth(_coords.depth);
    Octree::Point origin = {
        tree_bounds_.min()[0] + _coords.x * size[0],
        tree_bounds_.min()[1] + _coords.y * size[1],
        tree_bounds_.min()[2] + _coords.z * size[2]
    };
    return AABB({origin,origin+size});
}

Octree::NodeCoords Octree::node_coords(Octree::NodeIndex _idx) const
{
    return decode(codes_[_idx]);
}

Octree::NodeCoords Octree::node_child_coords(
    const Octree::NodeCoords& _node_coords,
    Octree::NodeIndex _child_idx)
{
    return Octree::NodeCoords{
        .x = _node_coords.x * 2 + ((_child_idx >> MORTON_SHIFTS[0]) & 1),
        .y = _node_coords.y * 2 + ((_child_idx >> MORTON_SHIFTS[1]) & 1),
        .z = _node_coords.z * 2 + ((_child_idx >> MORTON_SHIFTS[2]) & 1),
        .depth = _node_coords.depth + 1
    };
}

Octree::Point Octree::node_size_at_depth(uint32_t depth) const
{
    FT scale = node_scale_at_depth(depth);
    return {
        scale * (tree_bounds_.max()[0] - tree_bounds_.min()[0]),
        scale * (tree_bounds_.max()[1] - tree_bounds_.min()[1]),
        scale * (tree_bounds_.max()[2] - tree_bounds_.min()[2])
    };
}

}
