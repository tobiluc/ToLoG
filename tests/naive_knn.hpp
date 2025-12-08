#pragma once
#include <ToLoG/Core.hpp>

namespace ToLoG
{

template<typename Point>
void naive_knn_search(const std::vector<Point>& _pts,
                      const Point& _q, const uint32_t _k,
                      std::vector<uint32_t>& _res)
{
    // Store all distances
    std::vector<double> d;
    d.reserve(_pts.size());
    for (const auto& p : _pts) {
        d.push_back((p - _q).squared_norm());
    }

    // Sort points
    std::vector<uint32_t> tmp(_pts.size());
    std::iota(tmp.begin(), tmp.end(), 0u);
    std::sort(tmp.begin(), tmp.end(), [&](const uint32_t& _i, const uint32_t& _j) {
        return d[_i] < d[_j];
    });

    // Returns k best
    _res.clear();
    _res.reserve(_k);
    for (uint32_t i = 0; i < _k; ++i) {
        _res.push_back(tmp[i]);
    }
}

}
