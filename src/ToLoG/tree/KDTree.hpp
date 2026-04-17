#pragma once
#include <ToLoG/Core.hpp>
#include <numeric>
#include <span>
#include <stack>

namespace ToLoG
{

template<vector P>
class KDTree
{
private:
    using FT = Traits<P>::value_type;
    using Range = std::pair<uint32_t,uint32_t>;
    struct Split {
        uint32_t axis_;
        FT value_;
    };
    std::span<const P> points_;
    uint32_t leaf_size_ = 16;
    std::vector<Split> splits_;
    std::vector<Range> ranges_; // Range in indices per node
    std::vector<uint32_t> left_; // left children
    inline uint32_t left_child(uint32_t _node_idx) const {return left_[_node_idx];}
    inline uint32_t right_child(uint32_t _node_idx) const {return left_[_node_idx]+1;}
    inline bool is_leaf(uint32_t _node_idx) const {return left_[_node_idx]==UINT32_MAX;}

    std::vector<uint32_t> indices; // to points

    uint32_t add_empty_node()
    {
        uint32_t node_idx = ranges_.size();
        ranges_.emplace_back();
        left_.emplace_back(UINT32_MAX);
        splits_.emplace_back();
        return node_idx;
    }

public:
    KDTree() {}

    KDTree(const std::vector<P>& _points, const uint32_t _leaf_size = 32)
    {
        build(_points, _leaf_size);
    }

    void clear()
    {
        splits_.clear();
        ranges_.clear();
        left_.clear();
        indices.clear();
    }

    void build(const std::vector<P>& _points, const uint32_t _leaf_size = 32)
    {
        clear();
        points_ = _points;
        leaf_size_ = _leaf_size;

        indices.resize(_points.size());
        std::iota(std::begin(indices), std::end(indices), 0);
        struct Item {
            uint32_t node_idx;
            Range range;
        };
        std::stack<Item> stack;
        // Push root node
        stack.emplace(Item{
            .node_idx=add_empty_node(),
            .range={0u,static_cast<uint32_t>(_points.size())}});
        while (!stack.empty())
        {
            Item item = stack.top();
            stack.pop();

            // Is Leaf?
            if ((item.range.second-item.range.first) <= leaf_size_) {
                ranges_[item.node_idx] = item.range;
                left_[item.node_idx] = UINT32_MAX;
                splits_.emplace_back(Split{.axis_=UINT32_MAX,.value_=FT(0)});
                continue;
            }

            // Choose Split Axis (Max. extent)
            AABB<P> bbox;
            for (uint32_t i = item.range.first; i < item.range.second; ++i) {
                bbox.expand(_points[indices[i]]);
            }
            Split split;
            split.axis_ = argmax(static_cast<P>(bbox.max()-bbox.min()));

            // Partition Points (Median)
            uint32_t mid = item.range.first + ((item.range.second-item.range.first) / 2);
            std::nth_element(indices.begin() + item.range.first,
                             indices.begin() + mid,
                             indices.begin() + item.range.second,
                             [&split,&_points](const uint32_t& _i0, const uint32_t& _i1) {
                                 return _points[_i0][split.axis_] < _points[_i1][split.axis_];
                             });
            split.value_ = _points[indices[mid]][split.axis_];

            // Create Internal Node
            uint32_t left_idx = add_empty_node();
            uint32_t right_idx = add_empty_node();
            ranges_[item.node_idx] = item.range;
            splits_[item.node_idx] = std::move(split);
            left_[item.node_idx] = left_idx;

            stack.emplace(Item{right_idx, {mid, item.range.second}});
            stack.emplace(Item{left_idx, {item.range.first, mid}});
        }
    }

    void k_nearest_neighbors(
        const P& _query,
        const uint32_t _k,
        std::vector<uint32_t>& _res) const
    {
        struct Candidate {
            uint32_t point_idx;
            FT dist2;
            inline bool operator<(const Candidate& _p) const {
                return dist2 < _p.dist2;
            }
        };

        // Max-heap to store the k closest points
        std::priority_queue<Candidate> pq;

        // Traversal stack
        std::stack<uint32_t> stack;
        stack.push(0); // Push root
        while (!stack.empty())
        {
            uint32_t node_idx = stack.top();
            stack.pop();

            // Leaf Node?
            if (left_[node_idx] == UINT32_MAX) {
                const Range& r = ranges_[node_idx];
                for (uint32_t i = r.first; i < r.second; ++i) {
                    uint32_t pt_idx = indices[i];
                    FT d2 = squared_distance(_query, points_[pt_idx]);
                    if (pq.size() < _k) {
                        pq.push({pt_idx, d2});
                    } else if (d2 < pq.top().dist2) {
                        pq.pop();
                        pq.push({pt_idx, d2});
                    }
                }
                continue;
            }

            // Internal Node?
            uint32_t left_idx = left_[node_idx];
            uint32_t right_idx = left_idx + 1;

            FT d = _query[splits_[node_idx].axis_] - splits_[node_idx].value_;
            FT d2 = d * d;

            // Determine near and far child
            uint32_t near = (d <= 0)? left_idx : right_idx;
            uint32_t far = (d <= 0)? right_idx : left_idx;

            // If the far side could potentially have a better point
            // push it to stack as well
            if (pq.size() < _k || d2 < pq.top().dist2) {
                stack.push(far);
            }
            // Always push the near side last so it's processed first
            stack.push(near);
        }

        // Convert heap to sorted result vector
        _res.clear();
        _res.reserve(_k);
        while (!pq.empty()) {
            _res.push_back(pq.top().point_idx);
            pq.pop();
        }
    }

    void radius_search(
        const P& _query,
        const FT _radius,
        std::vector<uint32_t>& _res) const
    {
        _res.clear();
        const FT r_squared = _radius*_radius;

        struct Candidate {
            uint32_t point_idx;
            FT dist2;
            inline bool operator<(const Candidate& _p) const {
                return dist2 < _p.dist2;
            }
        };

        // Traversal stack
        std::stack<uint32_t> stack;
        stack.push(0); // Push root
        while (!stack.empty())
        {
            uint32_t node_idx = stack.top();
            stack.pop();

            // Leaf Node?
            if (left_[node_idx] == UINT32_MAX) {
                const Range& r = ranges_[node_idx];
                for (uint32_t i = r.first; i < r.second; ++i) {
                    uint32_t pt_idx = indices[i];
                    FT d2 = squared_distance(_query, points_[pt_idx]);
                    if (d2 <= r_squared) {
                        _res.push_back(pt_idx);
                    }
                }
                continue;
            }

            // Internal Node?
            uint32_t left_idx = left_[node_idx];
            uint32_t right_idx = left_idx + 1;

            FT d = _query[splits_[node_idx].axis_] - splits_[node_idx].value_;
            FT d2 = d * d;

            // Determine near and far child
            uint32_t near = (d <= 0)? left_idx : right_idx;
            uint32_t far = (d <= 0)? right_idx : left_idx;

            // If the far side could potentially have a better point
            // push it to stack as well
            if (d2 <= r_squared) {
                stack.push(far);
            }
            // Always push the near side last so it's processed first
            stack.push(near);
        }
    }
};

}
