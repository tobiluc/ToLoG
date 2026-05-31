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
template<vector_of_dim<3> Point>
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

private:
    using FT = Traits<Point>::value_type;
    using AABB = AABB<Point>;

    static constexpr FT eps = 1e-10;
    static constexpr uint32_t num_node_children = 8;
    static constexpr uint32_t num_node_neighbours = 6;
    static constexpr uint32_t num_depth_bits = 6;
    static constexpr uint32_t num_morton_bits = 58;

    // If Shifts are 2,1,0, children are sorted lexographically
    // matching AABB.corners()
    // For 0,1,2 they are sorted from the right instead
    static constexpr uint8_t morton_shifts[3] = {2,1,0};

    static constexpr uint64_t expand_bits(uint32_t v)
    {
        v = (v | (v << 16)) & 0x030000FF;
        v = (v | (v << 8)) & 0x0300F00F;
        v = (v | (v << 4)) & 0x030C30C3;
        v = (v | (v << 2)) & 0x09249249;
        return v;
    }

    static constexpr Octree::NodeCode encode(const NodeCoords& _coords)
    {
        assert(_coords.depth < (1u << NUM_DEPTH_BITS));
        uint64_t morton = (expand_bits(_coords.x) << morton_shifts[0])
                          | (expand_bits(_coords.y) << morton_shifts[1])
                          | (expand_bits(_coords.z) << morton_shifts[2]);
        return ((uint64_t)(_coords.depth) << num_morton_bits)
               | (morton & ((1ULL << num_morton_bits) - 1));
    }

    static constexpr uint32_t compact_bits(uint64_t v)
    {
        v &= 0x09249249;
        v = (v ^ (v >> 2)) & 0x030C30C3;
        v = (v ^ (v >> 4)) & 0x0300F00F;
        v = (v ^ (v >> 8)) & 0x030000FF;
        v = (v ^ (v >> 16)) & 0x000003FF;
        return (uint32_t)v;
    }

    static constexpr NodeCoords decode(NodeCode code)
    {
        uint64_t morton = code & ((1ULL << num_morton_bits) - 1);
        return {
            .x = compact_bits(morton >> morton_shifts[0]),
            .y = compact_bits(morton >> morton_shifts[1]),
            .z = compact_bits(morton >> morton_shifts[2]),
            .depth = (uint32_t)(code >> num_morton_bits)
        };
    }

private:
    uint32_t max_depth_ = 5;
    uint32_t initial_resolution_ = 2;
    std::vector<NodeCode> codes_;
    HashMap<NodeCode, NodeIndex> code_index_map_;
    AABB tree_bounds_;

public:
    Octree(AABB _tree_bounds, uint32_t _initial_resolution = 2, uint32_t _max_depth = 5)
        :
        tree_bounds_(_tree_bounds),
        max_depth_(_max_depth),
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


    Octree() : Octree(AABB({filled<Point>(0), filled<Point>(1)}))
    {}

    /*
     * Refines the tree according to the given node splits.
     * Nodes that are not present in the splits are deleted. If a node
     * should be kept, but not split, set its mask to 0.
     */
    void refine_tree(
        const std::vector<NodeSplit>& _splits
    )
    {
        std::vector<NodeCode> new_codes;
        new_codes.reserve(_splits.size()*num_node_children);

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
            for (uint8_t i = 0; i < num_node_children; ++i) {
                // Only add child if mask bit is 1
                bool has_child = split.children_mask_ & (1<<(num_node_children-1-i));
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

    std::optional<NodeIndex> locate(Point _q) const
    {
        // Normalize within tree bounds
        _q[0] = (_q[0] - tree_bounds_.min()[0]) / (tree_bounds_.max()[0]-tree_bounds_.min()[0]);
        _q[1] = (_q[1] - tree_bounds_.min()[1]) / (tree_bounds_.max()[1]-tree_bounds_.min()[1]);
        _q[2] = (_q[2] - tree_bounds_.min()[2]) / (tree_bounds_.max()[2]-tree_bounds_.min()[2]);

        // Check if outside tree bounds
        if (_q[0] < -eps || _q[0] > 1.0+eps
            || _q[1] < -eps || _q[1] > 1.0+eps
            || _q[2] < -eps || _q[2] > 1.0+eps) {
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

    FT node_scale_at_depth(uint32_t _depth) const
    {
        return static_cast<FT>(1.0) / (initial_resolution_ << _depth); // resolution * 2^depth
    }

    Point node_size_at_depth(uint32_t _depth) const
    {
        FT scale = node_scale_at_depth(_depth);
        return {
            scale * (tree_bounds_.max()[0] - tree_bounds_.min()[0]),
            scale * (tree_bounds_.max()[1] - tree_bounds_.min()[1]),
            scale * (tree_bounds_.max()[2] - tree_bounds_.min()[2])
        };
    }

    Point node_center(const NodeCoords& _coords) const
    {
        Point size = node_size_at_depth(_coords.depth);
        return Point{
            tree_bounds_.min()[0] + _coords.x * size[0],
            tree_bounds_.min()[1] + _coords.y * size[1],
            tree_bounds_.min()[2] + _coords.z * size[2]
        };
    }

    AABB node_aabb(const NodeCoords& _coords) const
    {
        Point size = node_size_at_depth(_coords.depth);
        Point center = {
            tree_bounds_.min()[0] + _coords.x * size[0],
            tree_bounds_.min()[1] + _coords.y * size[1],
            tree_bounds_.min()[2] + _coords.z * size[2]
        };
        return AABB({center,center+size});
    }

    NodeCoords node_coords(NodeIndex _idx) const
    {
        return decode(codes_[_idx]);
    }

    // Integer Coordinates of a nodes' child
    static NodeCoords node_child_coords(
        const NodeCoords& _node_coords,
        NodeIndex _child_idx)
    {
        return NodeCoords{
            .x = _node_coords.x * 2 + ((_child_idx >> morton_shifts[0]) & 1),
            .y = _node_coords.y * 2 + ((_child_idx >> morton_shifts[1]) & 1),
            .z = _node_coords.z * 2 + ((_child_idx >> morton_shifts[2]) & 1),
            .depth = _node_coords.depth + 1
        };
    }

    uint32_t max_depth() const {
        return max_depth_;
    }

    size_t n_nodes() const {
        return codes_.size();
    }
};

}

