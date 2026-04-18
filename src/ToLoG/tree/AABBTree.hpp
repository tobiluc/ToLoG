#pragma once

#include <ToLoG/geometry/aabb_from_shape.hpp>
#include <ToLoG/geometry/squared_distance.hpp>
#include <ToLoG/geometry/predicates/intersects.hpp>
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

namespace ToLoG
{

template<typename T>
class AABBTree
{
protected:
    using Primitive = T;
    using Point = Traits<T>::vector_type;
    using FT = Traits<Point>::value_type;
    constexpr static int DIM = Traits<Point>::dim;
    using AABB = AABB<Point>;

    struct Node {
        uint32_t left = UINT32_MAX;
        uint32_t start = UINT32_MAX;
        uint32_t end = UINT32_MAX;
        AABB aabb;
    };
public:
    AABBTree() {}

    AABBTree(std::span<const Primitive> _primitives, size_t _leaf_size = 32) :
        primitives_(_primitives),
        leaf_size_(_leaf_size)
    {
        build_tree(_primitives, _leaf_size);
    }

    void build_tree(std::span<const Primitive> _primitives, size_t _leaf_size = 32)
    {
        leaf_size_ = _leaf_size;
        primitives_ = _primitives;

        // Setup
        nodes_.clear();
        nodes_.reserve(2*n_primitives()/leaf_size_);
        prim_idx_buffer_.clear();
        prim_idx_buffer_.reserve(n_primitives());

        if (n_primitives() == 0) {
            std::cerr << "Warning: Building AABBTree with no primitives" << std::endl;
        }

        // Cache Primitive AABBs
        std::vector<AABB> prim_aabbs;
        prim_aabbs.reserve(n_primitives());
        for (uint32_t i = 0; i < n_primitives(); ++i) {
            prim_aabbs.push_back(aabb(primitive(i)));
        }

        struct NodeTask {
            uint32_t node_idx;
            uint32_t* begin;
            size_t n;
        };
        std::vector<uint32_t> idx(n_primitives());
        std::iota(idx.begin(), idx.end(), 0u); // idx[i] = i

        std::vector<NodeTask> stack;
        stack.reserve(64);
        nodes_.emplace_back(); // Create root node
        stack.push_back({0, idx.data(), idx.size()});

        while (!stack.empty()) {
            NodeTask t = stack.back();
            stack.pop_back();
            Node& node = nodes_.at(t.node_idx);

            // Compute Node bbox
            if (t.n > 0) {
                node.aabb = prim_aabbs[t.begin[0]];
                for (uint32_t i = 1; i < t.n; ++i) {
                    node.aabb.expand(prim_aabbs[t.begin[i]]);
                }
            }

            // leaf?
            if(t.n <= leaf_size_) {
                nodes_[t.node_idx].start = prim_idx_buffer_.size();
                nodes_[t.node_idx].end = nodes_[t.node_idx].start + t.n;
                for(uint32_t i=0;i<t.n;i++) {
                    prim_idx_buffer_.push_back(t.begin[i]);
                }
                continue;
            }

            // compute centroids and choose split axis = longest axis of node bbox
            uint32_t split_axis = argmax(static_cast<Point>(node.aabb.max()-node.aabb.min()));

            // compute median by nth_element using centroid of prim AABBs
            // nth_element on the subrange prim_indices_[t.begin .. t.begin+t.n)
            uint32_t mid = t.n / 2;
            std::nth_element(
                t.begin,
                t.begin + mid,
                t.begin + t.n,
                [&](const uint32_t& a, const uint32_t& b){
                    return centroid(prim_aabbs[a])[split_axis]
                           < centroid(prim_aabbs[b])[split_axis];
                }
            );

            // create children nodes
            uint32_t l = nodes_.size();
            node.left = l;
            nodes_.emplace_back();
            nodes_.emplace_back();

            // push children tasks
            stack.push_back({l+1, t.begin + mid, t.n - mid});
            stack.push_back({l, t.begin, mid});
        }
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
                for(uint32_t i=N.start;i<N.end;i++){
                    uint32_t pid = prim_idx_buffer_[i];
                    const FT d = squared_distance(_q, primitives_[pid]);
                    if(best.size()<_k) {
                        best.push({pid, d});
                    } else if(d < best.top().d) {
                        // Replace top
                        best.pop();
                        best.push({pid, d});
                    }
                }
            } else {
                FT lbL = squared_distance(_q, nodes_[N.left].aabb);
                if(best.size()<_k || lbL < best.top().d) {
                    pq.push({N.left, lbL}); // left child
                }

                FT lbR = squared_distance(_q, nodes_[N.left+1].aabb);
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

    uint32_t nearest_neighbor(const Point& _q) const
    {
        return k_nearest_neighbors(_q, 1).at(0);
    }

    void radius_search(const Point& _q,
                    const FT _radius,
                std::vector<uint32_t>& _res) const
    {
        _res.clear();
        if (nodes_.empty()) {return;}
        const FT r_squared = _radius*_radius;

        struct IdxD {
            uint32_t idx;
            FT d;
            inline bool operator<(const IdxD& _id) const {return d < _id.d;}
            inline bool operator>(const IdxD& _id) const {return d > _id.d;}
        };

        // Node & Distance Lower Bound
        std::priority_queue<IdxD,std::vector<IdxD>,std::greater<IdxD>> pq;

        pq.push({0, FT(0)}); // push root

        while(!pq.empty())
        {
            auto [node_idx, lb] = pq.top();
            pq.pop();

            if(lb > r_squared) {
                break;
            }

            const Node& N = nodes_[node_idx];

            if(is_leaf_node(node_idx)) {
                for(uint32_t i=N.start;i<N.end;i++){
                    uint32_t pid = prim_idx_buffer_[i];
                    const FT d = squared_distance(_q, primitives_[pid]);
                    if(d <= r_squared) {
                        _res.push_back(pid);
                    }
                }
            } else {
                FT lbL = squared_distance(_q, nodes_[N.left].aabb);
                if(lbL <= r_squared) {
                    pq.push({N.left, lbL}); // left child
                }

                FT lbR = squared_distance(_q, nodes_[N.left+1].aabb);
                if(lbR <= r_squared) {
                    pq.push({N.left+1, lbR}); // right child
                }
            }
        }
    }

    std::vector<uint32_t> radius_search(
        const Point& _q,
        const FT _radius) const
    {
        std::vector<uint32_t> res;
        radius_search(_q, _radius, res);
        return res;
    }

    template<typename PrimT>
    requires(std::is_same<typename Traits<PrimT>::vector_type,Point>::value)
    bool intersecting(const PrimT& _q,
        std::vector<uint32_t>& _res) const
    {
        _res.clear();
        AABB q_aabb = aabb(_q);
        if (n_nodes()==0 || !intersects(q_aabb,nodes_[0].aabb)) {return false;}

        std::stack<uint32_t> node_stack;
        node_stack.push(0);

        while (!node_stack.empty())
        {
            uint32_t node_i = node_stack.top();
            node_stack.pop();

            if (is_leaf_node(node_i)) {
                // Check all primitives in node
                for(uint32_t i=nodes_[node_i].start; i < nodes_[node_i].end; ++i) {
                    if (intersects(primitives_[prim_idx_buffer_[i]], _q)) {
                        _res.push_back(prim_idx_buffer_[i]);
                    }
                }
            } else {
                // Push children with intersecting boxes
                if (intersects(q_aabb,nodes_[nodes_[node_i].left].aabb)) {
                    node_stack.push(nodes_[node_i].left);
                }
                if (intersects(q_aabb,nodes_[nodes_[node_i].left+1].aabb)) {
                    node_stack.push(nodes_[node_i].left+1);
                }
            }
        }
        return !_res.empty();
    }

    template<typename PrimT>
        requires(std::is_same<typename Traits<PrimT>::vector_type,Point>::value)
    std::vector<uint32_t> intersecting(const PrimT& _q) const
    {
        std::vector<uint32_t> res;
        intersecting(_q, res);
        return res;
    }

    /// Locates the first primitive which contains the query point q
    std::optional<uint32_t> locate(const Point& _q) const
    {
        std::stack<uint32_t> stack;
        stack.push(0);

        while (!stack.empty())
        {
            uint32_t node_i = stack.top();
            stack.pop();

            // If the point is not in the bounding box, it will not
            // be in any primitive
            if (!intersects(nodes_[node_i].aabb, _q)) {
                continue;
            }

            if (is_leaf_node(node_i)) {
                // If we have no more children, we look through all the primitives in the node
                for (uint32_t i = nodes_[node_i].start; i < nodes_[node_i].end; ++i) {
                    uint32_t prim = prim_idx_buffer_[i];
                    if (intersects(primitives_[prim], _q)) {
                        return prim;
                    }
                }
            } else {
                // Search both children
                uint32_t left  = nodes_[node_i].left;
                uint32_t right = left + 1;

                stack.push(left);
                stack.push(right);
            }
        }

        return std::nullopt;
    }

    inline constexpr size_t n_nodes() const {
        return nodes_.size();
    }

    inline constexpr size_t n_primitives() const {
        return primitives_.size();
    }

    inline const AABB& node_aabb(uint32_t _node_i) const {
        return nodes_[_node_i].aabb;
    }

    inline constexpr bool is_leaf_node(uint32_t _node_idx) const {
        return nodes_[_node_idx].left == UINT32_MAX;
    }

    const Primitive& primitive(uint32_t _prim_idx) const {
        return primitives_[_prim_idx];
    }

    inline constexpr size_t leaf_size() const {
        return leaf_size_;
    }

protected:
    std::span<const Primitive> primitives_;
    std::vector<Node> nodes_;
    std::vector<uint32_t> prim_idx_buffer_;
    size_t leaf_size_ = 32;
};

}
