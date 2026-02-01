#pragma once

#include <ToLoG/Core.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <ostream>
#include <span>
#include <stack>
#include <vector>
#include <numeric>
#include <queue>
#include <ToLoG/predicates/contains_point.hpp>

namespace ToLoG
{

template<typename T>
class DynamicAABBTree
{
public:
    using Primitive = T;
    using Point = Traits<T>::vector_type;
    using FT = Traits<Point>::value_type;
    constexpr static int DIM = Traits<Point>::dim;
    using AABB = AABB<Point>;

protected:
    struct Node {
        uint32_t left = UINT32_MAX; // right = left+1
        std::vector<uint32_t> prim_indices;
        AABB aabb;
    };
public:
    DynamicAABBTree() {}

    DynamicAABBTree(std::span<const Primitive> _primitives, size_t _leaf_size = 32) :
        primitives_(_primitives),
        leaf_size_(_leaf_size)
    {
        build_tree();
    }

    void split_node(uint32_t _node_i)
    {
        std::stack<uint32_t> node_stack;
        node_stack.push(_node_i);

        while (!node_stack.empty())
        {
            _node_i = node_stack.top();
            node_stack.pop();

            // Small enough
            if (node_size(_node_i) <= leaf_size_) {continue;}

            // Determine split axis (max extent of node bbox)
            uint32_t split_axis = argmax(node_aabb(_node_i).max() - node_aabb(_node_i).min());

            // Median Split
            auto m = nodes_[_node_i].prim_indices.begin() + nodes_[_node_i].prim_indices.size() / 2;
            std::nth_element(
                nodes_[_node_i].prim_indices.begin(),
                m,
                nodes_[_node_i].prim_indices.end(),
                [&](const uint32_t& a, const uint32_t& b){
                    return centroid(prim_bboxes_[a])[split_axis]
                           < centroid(prim_bboxes_[b])[split_axis];
                }
                );

            // Create Left Child
            nodes_[_node_i].left = nodes_.size();
            nodes_.emplace_back();
            nodes_.back().prim_indices.reserve(leaf_size_);
            for (auto it = nodes_[_node_i].prim_indices.begin(); it <= m; ++it) {
                nodes_.back().prim_indices.push_back(*it);
                nodes_.back().aabb.expand(prim_bboxes_[*it]);
            }

            // Create Right Child
            nodes_.emplace_back();
            nodes_.back().prim_indices.reserve(leaf_size_);
            for (auto it = m; it < nodes_[_node_i].prim_indices.end(); ++it) {
                nodes_.back().prim_indices.push_back(*it);
                nodes_.back().aabb.expand(prim_bboxes_[*it]);
            }

            node_stack.push(nodes_.size()-2);
            node_stack.push(nodes_.size()-1);
        }
    }

    void build_tree()
    {
        // Setup
        nodes_.clear();
        nodes_.reserve(2*n_primitives()/leaf_size_);
        prim_idx_buffer_.clear();
        prim_idx_buffer_.reserve(n_primitives());

        if (n_primitives() == 0) {
            std::cerr << "Warning: Building AABBTree with no primitives" << std::endl;
        }

        // Cache Primitive AABBs
        prim_bboxes_.clear();
        prim_bboxes_.reserve(n_primitives());
        for (uint32_t i = 0; i < n_primitives(); ++i) {
            prim_bboxes_.push_back(aabb(primitive(i)));
        }

        // Create the root Box
        nodes_.emplace_back();
        nodes_.back().prim_indices.resize(n_primitives());
        std::iota(nodes_.back().prim_indices.begin(), nodes_.back().prim_indices.end(), 0u);
        nodes_.back().aabb = prim_bboxes_[0];
        for (uint32_t i = 1; i < n_primitives(); ++i) {
            nodes_.back().aabb.expand(prim_bboxes_[i]);
        }

        split_node(0);
    }

    void k_nearest_neighbors(const Point& _q,
                             const uint32_t _k,
                             std::vector<uint32_t>& _res) const
    {
        _res.clear();
        _res.reserve(_k);
        if (_k == 0 || nodes_.empty()) {return;}

        struct IdxD {
            uint32_t idx;
            FT d;
            inline bool operator<(const IdxD& _id) const {return d < _id.d;}
            inline bool operator>(const IdxD& _id) const {return d > _id.d;}
        };

        // Point & Distance
        std::priority_queue<IdxD,std::vector<IdxD>,std::less<IdxD>> best;

        // Node & Distance Lower Bound
        std::priority_queue<IdxD,std::vector<IdxD>,std::greater<IdxD>> pq;

        pq.push({0, FT(0)}); // push root

        while(!pq.empty())
        {
            auto [node_idx, lb] = pq.top();
            pq.pop();

            // We already found k points and the node cannot contain closer points?
            if(best.size()==_k && lb >= best.top().d) {
                break;
            }

            const Node& N = nodes_[node_idx];

            if(is_leaf_node(node_idx)) {
                for(uint32_t pid : nodes_[node_idx].prim_indices) {
                    const FT d = point_squared_distance(_q, primitives_[pid]);
                    if(best.size()<_k) {
                        best.push({pid, d});
                    } else if(d < best.top().d) {
                        // Replace top
                        best.pop();
                        best.push({pid, d});
                    }
                }
            } else {
                FT lbL = point_squared_distance(_q, nodes_[N.left].aabb);
                if(best.size()<_k || lbL < best.top().d) {
                    pq.push({N.left, lbL}); // left child
                }

                FT lbR = point_squared_distance(_q, nodes_[N.left+1].aabb);
                if(best.size()<_k || lbR < best.top().d) {
                    pq.push({N.left+1, lbR}); // right child
                }
            }
        }

        // Extract result
        _res.resize(best.size());
        uint32_t i = best.size();
        while(!best.empty()){
            _res[--i] = best.top().idx;
            best.pop();
        }
    }

    std::vector<uint32_t> k_nearest_neighbors(const Point& _q,
                                              const uint32_t _k) const
    {
        std::vector<uint32_t> res;
        k_nearest_neighbors(_q, _k, res);
        return res;
    }

    /// Locates the primitive which contains the query point q
    std::optional<uint32_t> locate(const Point& _q) const
    {
        // First, locate the node
        uint32_t node_i(0);
        while (true) {
            if (is_leaf_node(node_i)) {break;}
            if (contains_point(nodes_[nodes_[node_i].left].aabb, _q)) {
                node_i = nodes_[node_i].left;
            } else if (contains_point(nodes_[nodes_[node_i].left+1].aabb, _q)) {
                node_i = nodes_[node_i].left+1;
            } else {
                return std::nullopt;
            }
        }

        // Locate the primitive within the node
        for(uint32_t i=nodes_[node_i].start; i < nodes_[node_i].end; ++i) {
            if (contains_point(primitives_[prim_idx_buffer_[i]], _q)) {
                return prim_idx_buffer_[i];
            }
        }
        return std::nullopt;
    }

    /*
    bool insert_inside_leaf(const std::vector<Primitive>& _new_prims)
    {
        if (_new_prims.empty()) {return false;}

        // Cache new prims bboxes and compute total bbox of new prims
        std::vector<AABB> new_prims_boxes;
        new_prims_boxes.reserve(_new_prims.size());
        AABB total_box;
        for (const auto& new_prim : _new_prims) {
            AABB box = aabb(new_prim);
            total_box.expand(box);
            new_prims_boxes.emplace_back(std::move(box));
        }

        // Find leaf node containing the total box
        auto node_i_opt = locate_leaf(total_box);
        if (!node_i_opt.has_value()) {return false;}
        uint32_t node_i = node_i_opt.value();

        //TODO
        return false;
    }
    */

    inline size_t n_nodes() const {
        return nodes_.size();
    }

    inline size_t n_primitives() const {
        return primitives_.size();
    }

    inline const AABB& node_aabb(uint32_t _node_i) const {
        return nodes_[_node_i].aabb;
    }

    inline bool is_leaf_node(uint32_t _node_idx) const {
        return nodes_[_node_idx].left == UINT32_MAX;
    }

    const Primitive& primitive(uint32_t _prim_idx) const {
        return primitives_[_prim_idx];
    }

    inline size_t leaf_size() const {
        return leaf_size_;
    }

protected:
    std::span<const Primitive> primitives_;
    std::vector<AABB> prim_bboxes_;
    std::vector<Node> nodes_;
    std::vector<uint32_t> prim_idx_buffer_;
    size_t leaf_size_ = 32;

    /// Locates the leaf node, starting from node_i, which contains the given box
    std::optional<uint32_t> locate_leaf(uint32_t _node_i, const AABB& _box) const
    {
        if (!nodes_[nodes_[_node_i]].aabb.contains(_box)) {return std::nullopt;}
        while (!is_leaf_node(_node_i)) {
            if (nodes_[nodes_[_node_i].left].aabb.contains(_box)) {
                _node_i = nodes_[_node_i].left;
            } else if (nodes_[nodes_[_node_i].left+1].aabb.contains(_box)) {
                _node_i = nodes_[_node_i].left+1;
            } else {
                return std::nullopt;
            }
        }
        return _node_i;
    }

    size_t node_size(uint32_t _node_i) const {
        return nodes_[_node_i].prim_indices.size();
    }
};

}
