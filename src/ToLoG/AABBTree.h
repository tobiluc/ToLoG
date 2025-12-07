#pragma once

#include <ToLoG/Core.h>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <ostream>
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
    using FT = Traits<T>::value_type;
    constexpr static int DIM = Traits<T>::dim;
    using Point = Traits<T>::vector_type;
    using AABB = AABB<Point>;

    struct Node {
        uint32_t left = UINT32_MAX;
        uint32_t start = UINT32_MAX;
        uint32_t end = UINT32_MAX;
        AABB aabb;
    };
public:
    AABBTree() {}

    AABBTree(std::vector<Primitive>& _primitives, size_t _leaf_size = 32) :
        primitives_(_primitives),
        leaf_size_(_leaf_size)
    {
        build_tree();
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
        prim_aabbs_.clear();
        prim_aabbs_.reserve(n_primitives());
        for (uint32_t i = 0; i < n_primitives(); ++i) {
            prim_aabbs_.push_back(primitive(i).aabb());
        }

        struct BuildTask {
            uint32_t node_idx;
            uint32_t* begin;
            size_t n;
        };
        std::vector<uint32_t> idx(n_primitives());
        std::iota(idx.begin(), idx.end(), 0u); // idx[i] = i

        std::vector<BuildTask> stack;
        stack.reserve(64);
        nodes_.emplace_back(); // Create root node
        stack.push_back({0, idx.data(), idx.size()});

        while (!stack.empty()) {
            BuildTask t = stack.back();
            stack.pop_back();
            Node& node = nodes_[t.node_idx];

            // compute bbox
            compute_node_aabb(node, t.begin, t.n);

            // leaf?
            if(t.n <= leaf_size_) {
                nodes_[t.node_idx].start = prim_idx_buffer_.size();
                nodes_[t.node_idx].end = nodes_[t.node_idx].start + t.n;
                for(size_t i=0;i<t.n;i++) {
                    prim_idx_buffer_.push_back(t.begin[i]);
                }
                continue;
            }

            // compute centroids and choose split axis = longest axis of node bbox
            uint32_t split_axis = (node.aabb.max()-node.aabb.min()).argmax();

            // compute median by nth_element using centroid of prim AABBs
            // nth_element on the subrange prim_indices_[t.begin .. t.begin+t.n)
            uint32_t mid = t.n / 2;
            std::nth_element(
                t.begin,
                t.begin + mid,
                t.begin + t.n,
                [&](const uint32_t& a, const uint32_t& b){
                    return prim_aabbs_[a].centroid()[split_axis]
                           < prim_aabbs_[b].centroid()[split_axis];
                }
            );

            // create children nodes
            node.left = nodes_.size();
            nodes_.emplace_back();
            nodes_.emplace_back();

            // push children tasks
            stack.push_back({node.left+1, t.begin + mid, t.n - mid});
            stack.push_back({node.left, t.begin, mid});
        }
    }

    void k_nearest_neighbors(const Point& _q,
                             const uint32_t _k,
                             std::vector<uint32_t>& _res) const
    {
        _res.clear();
        _res.reserve(_k);
        if (_k == 0 || nodes_.empty()) {return;}

        struct PointIdxDist {
            uint32_t idx;
            FT d;
            inline bool operator<(const PointIdxDist& _pilb) const {
                return d < _pilb.d;
            }
        };

        // Helper: Holds Node Index and Squared Distance Lower Bound
        struct NodeIdxLowerBound {
            uint32_t idx;
            FT lb; // lower bound on distance
            inline bool operator>(const NodeIdxLowerBound& _nilb) const {
                return lb > _nilb.lb;
            }
        };

        std::priority_queue<PointIdxDist,std::vector<PointIdxDist>,std::less<PointIdxDist>> best;
        std::priority_queue<NodeIdxLowerBound,std::vector<NodeIdxLowerBound>,std::greater<NodeIdxLowerBound>> pq;

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

    size_t n_nodes() const {
        return nodes_.size();
    }

    size_t n_primitives() const {
        return primitives_.size();
    }

protected:
    std::vector<T>& primitives_; // n_primitives_
    std::vector<AABB> prim_aabbs_;
    std::vector<Node> nodes_;
    std::vector<uint32_t> prim_idx_buffer_;
    size_t leaf_size_ = 32;

    const Primitive& primitive(uint32_t _prim_idx) const {
        return primitives_[_prim_idx];
    }

    inline bool is_leaf_node(uint32_t _node_idx) const {
        return nodes_[_node_idx].left == UINT32_MAX;
    }

    inline void compute_node_aabb(Node& _node, uint32_t* _begin, uint32_t _count)
    {
        if (_count==0) {return;}
        _node.aabb = prim_aabbs_[_begin[0]];
        for (uint32_t i = 1; i < _count; ++i) {
            _node.aabb.expand(prim_aabbs_[_begin[i]]);
        }
    }
};

}
