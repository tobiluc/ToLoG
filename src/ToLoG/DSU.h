#pragma once

namespace ToLoG
{

/// Disjoint Set Union
class DSU
{
public:
    explicit DSU(size_t n) : n_(n), parent_(n), rank_(n, 0), size_(n, 1) {
        std::iota(parent_.begin(), parent_.end(), 0); // set parent[i] = i
    }

    /// Returns the node representing the connected component in which x lies
    inline uint32_t root(uint32_t x) {
        uint32_t r = x;
        while (r != parent_[r]) {r = parent_[r];} // Wander to root
        while (x != r) { // Compress paths
            uint32_t p = parent_[x];
            parent_[x] = r;
            x = p;
        }
        return r;
    }

    /// Merge two Sets
    inline void unite(uint32_t a, uint32_t b) {
        a = root(a);
        b = root(b);
        if (a == b) {return;}
        if (rank_[a] < rank_[b]) {std::swap(a, b);}
        parent_[b] = a;
        if (rank_[a] == rank_[b]) {++rank_[a];}
        size_[a] += size_[b];
    }

    /// Returns the immediate parent of a node
    inline uint32_t parent(uint32_t v) const {
        return parent_[v];
    }

    /// Returns the size of the connected component of a node
    inline uint32_t size(uint32_t v) {
        return size_[root(v)];
    }

    /// Fills a list with all connected components
    inline void connected_components(
        std::vector<std::vector<uint32_t>>& _p
        ) {
        _p.clear();
        _p.reserve(n_);
        std::vector<int> p_idx(n_, -1);
        for (uint32_t i = 0; i < n_; ++i) {
            uint32_t r = root(i);
            if (p_idx[r] == -1) {
                p_idx[r] = _p.size();
                _p.emplace_back();
            }
            _p[p_idx[r]].push_back(i);
        }
    }

private:
    size_t n_;
    std::vector<uint32_t> parent_;
    std::vector<uint32_t> rank_;
    std::vector<uint32_t> size_;
};

}
