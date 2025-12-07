#pragma once

#include <cassert>
#include <cstdint>
#include <vector>

namespace IGRec::Geometry
{

template<typename FT, int DIM>
class KDTree
{
private:
    struct Node {
        // Left children index in nodes_ (right = left+1).
        // If leaf: left==right==UINT32_MAX
        uint32_t left = UINT32_MAX;

        // Range [start, end) in points_idx_buffer_.
        // If inner: start==end==UINT32_MAX
        uint32_t start = UINT32_MAX;
        uint32_t end = UINT32_MAX;

        // AABB (min, max)
        FT bbox_min[DIM];
        FT bbox_max[DIM];

        // Split axis and value (for inner nodes)
        uint32_t split_axis = 0;
        FT split_value = 0.0;
    };

public:
    KDTree(size_t _n_points, const FT* _points_data, size_t _leaf_size = 32) :
        points_data_(_points_data),
        n_points_(_n_points),
        leaf_size_(_leaf_size)
    {
        build_tree();
    }

    void k_nearest_neighbors(const FT* _q, const uint32_t _k, std::vector<uint32_t>& _res) const;

private:
    const FT* points_data_; // DIM x n_points
    size_t n_points_;
    std::vector<Node> nodes_;
    std::vector<uint32_t> points_idx_buffer_;
    size_t leaf_size_ = 32;

    void build_tree();

    inline size_t n_nodes() const {
        return nodes_.size();
    }

    inline size_t n_points() const {
        return n_points_;
    }

    inline bool is_leaf_node(uint32_t _node_idx) const {
        assert(_node_idx < n_nodes());
        return nodes_[_node_idx].left == UINT32_MAX;
    }

    inline const Node& node(uint32_t _node_idx) const {
        assert(_node_idx < n_nodes());
        return nodes_[_node_idx];
    }

    /// Returns a Pointer to the _pidx-th point
    inline const FT* point(const uint32_t& _pidx) const {
        assert(_pidx < n_points_);
        return &points_data_[DIM*_pidx+0];
    }

    /// Squared Distance from Query Point Q to pidx-th Point
    inline FT point_squared_distance(const FT* _q, const uint32_t& _pidx) const {
        assert(_pidx < n_points_);
        FT res(0);
        for (int i = 0; i < DIM; ++i) {
            const FT d = _q[i]-point(_pidx)[i];
            res += d*d;
        }
        return res;
    }

    /// Squared Distance Lower Bound from Query Point Q to Node (based on bbox)
    inline FT node_squared_distance_lower_bound(const FT* _q, const uint32_t& _node_idx) const {
        assert(_node_idx < n_nodes());
        FT res(0);
        for (int i = 0; i < DIM; ++i) {
            const FT d = std::max(std::max(node(_node_idx).bbox_min[i] - _q[i], FT(0)), _q[i] - node(_node_idx).bbox_max[i]);
            res += d*d;
        }
        return res;
    };
};

}
