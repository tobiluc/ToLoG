#include <IGRec/geometry/KDTree.h>
#include <iostream>
#include <numeric>
#include <ostream>
#include <queue>
#include <stack>

namespace IGRec::Geometry
{

template<typename FT, int DIM>
void KDTree<FT,DIM>::build_tree()
{
    nodes_.clear();
    nodes_.reserve(2*n_points()/leaf_size_);
    points_idx_buffer_.clear();
    points_idx_buffer_.reserve(n_points());
    if (n_points() == 0) {
        std::cerr << "Warning: Building KDTree with no points" << std::endl;
        return;
    }

    struct BuildTask {
        uint32_t node_idx;
        uint32_t* begin;
        size_t n;
    };

    std::vector<uint32_t> idx(n_points());
    std::iota(idx.begin(), idx.end(), 0u); // idx[i] = i

    // Create root node
    nodes_.emplace_back();
    std::stack<BuildTask> s;
    s.push({0, idx.data(), idx.size()});

    // Build
    while(!s.empty())
    {
        BuildTask t = s.top();
        s.pop();

        // Compute bounding box
        for (int j = 0; j < DIM; ++j) {
            nodes_[t.node_idx].bbox_min[j] = point(t.begin[0])[j];
            nodes_[t.node_idx].bbox_max[j] = point(t.begin[0])[j];
        }
        for(size_t i=1;i<t.n;i++){
            for (int j = 0; j < DIM; ++j) {
                nodes_[t.node_idx].bbox_min[j] = std::min(point(t.begin[i])[j], nodes_[t.node_idx].bbox_min[j]);
                nodes_[t.node_idx].bbox_max[j] = std::max(point(t.begin[i])[j], nodes_[t.node_idx].bbox_max[j]);
            }
        }

        // Leaf?
        if(t.n <= leaf_size_) {
            nodes_[t.node_idx].start = points_idx_buffer_.size();
            nodes_[t.node_idx].end = nodes_[t.node_idx].start + t.n;
            for(size_t i=0;i<t.n;i++) {
                points_idx_buffer_.push_back(t.begin[i]);
            }
            continue;
        }

        // Determine Split Axis
        {
            FT d_max(0);
            for (int i = 0; i < DIM; ++i) {
                FT d(nodes_[t.node_idx].bbox_max[i]-nodes_[t.node_idx].bbox_min[i]);
                if (d > d_max) {
                    d_max = d;
                    nodes_[t.node_idx].split_axis = i;
                }
            }
        }

        // Median split
        uint32_t m = t.n / 2;
        std::nth_element(
            t.begin,
            t.begin + m,
            t.begin + t.n,
            [&](uint32_t a, uint32_t b){
                return point(a)[nodes_[t.node_idx].split_axis] < point(b)[nodes_[t.node_idx].split_axis];
            }
        );
        nodes_[t.node_idx].split_value = point(t.begin[m])[nodes_[t.node_idx].split_axis];

        // Create children
        nodes_[t.node_idx].left = nodes_.size();
        nodes_.emplace_back(); // left
        nodes_.emplace_back(); // right

        // Push tasks
        s.push({nodes_[t.node_idx].left+1, t.begin+m, t.n-m});
        s.push({nodes_[t.node_idx].left, t.begin, m});
    }
    // Debug Check
#ifndef NDEBUG
    std::cerr << "--------------------" << std::endl;
    std::cerr << "KD Tree Construction" << std::endl;
    std::cerr << "Node Memory: " << (nodes_.size()*sizeof(Node)*1e-6) << " MB" << std::endl;
    std::cerr << "--------------------" << std::endl;
    size_t n_points_in_nodes = 0;
    for (uint32_t i = 0; i < n_nodes(); ++i) {
        if (is_leaf(i)) {
            assert(node(i).left == UINT32_MAX);
            assert(node(i).start < node(i).end);
            assert(node(i).start < points_idx_buffer_.size());
            assert(node(i).end <= points_idx_buffer_.size());
            n_points_in_nodes += node(i).end-node(i).start;
        } else {
            assert(node(i).left < n_nodes());
            assert(node(i).start == UINT32_MAX);
            assert(node(i).end == UINT32_MAX);
        }
    }
    assert(n_points_in_nodes == points_.size());
#endif
}

template<typename FT, int DIM>
void KDTree<FT,DIM>::k_nearest_neighbors(const FT *_q, const uint32_t _k, std::vector<uint32_t>& _res) const
{
    // Helper: Holds Point Index and Squared Distance
    struct PointIdxD2
    {
        uint32_t idx;
        FT d2;
        inline bool operator<(const PointIdxD2& _pilb) const {
            return d2 < _pilb.d2;
        }
    };

    // Helper: Holds Node Index and Squared Distance Lower Bound
    struct NodeIdxLB
    {
        uint32_t idx;
        FT lb; // lower bound on squared distance
        inline bool operator>(const NodeIdxLB& _nilb) const {
            return lb > _nilb.lb;
        }
    };

    std::priority_queue<PointIdxD2,std::vector<PointIdxD2>,std::less<PointIdxD2>> best;
    std::priority_queue<NodeIdxLB,std::vector<NodeIdxLB>,std::greater<NodeIdxLB>> pq;

    pq.push({0, FT(0)}); // push root
    while(!pq.empty())
    {
        auto [node_idx, lb] = pq.top();
        pq.pop();

        // We already found k points and the node cannot contain closer points?
        if(best.size()==_k && lb >= best.top().d2) {
            break;
        }

        const Node& N = nodes_[node_idx];

        if(is_leaf_node(node_idx)){
            for(uint32_t i=N.start;i<N.end;i++){
                uint32_t pid = points_idx_buffer_[i];
                const FT d2 = point_squared_distance(_q, pid);
                if(best.size()<_k) {
                    best.push({pid, d2});
                } else if(d2 < best.top().d2) {
                    // Replace top
                    best.pop();
                    best.push({pid, d2});
                }
            }
        } else {
            FT lbL = node_squared_distance_lower_bound(_q, N.left);
            if(best.size()<_k || lbL < best.top().d2) {
                pq.push({N.left, lbL}); // left child
            }

            FT lbR = node_squared_distance_lower_bound(_q, N.left+1);
            if(best.size()<_k || lbR < best.top().d2) {
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

template class KDTree<double,3>;
template class KDTree<double,2>;
template class KDTree<double,1>;
template class KDTree<float,3>;
template class KDTree<float,2>;
template class KDTree<float,1>;
template class KDTree<int,3>;
template class KDTree<int,2>;
template class KDTree<int,1>;

}
