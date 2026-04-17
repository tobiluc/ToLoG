#pragma once

#include <ToLoG/vector_concepts.hpp>
#include <span>

namespace ToLoG
{

template<typename P>
class AABB
{
private:
    static constexpr int DIM = Traits<P>::dim;
    using FT = Traits<P>::value_type;
public:
    AABB() {
        make_empty();
    }
    AABB(std::span<const P> _pts) {
        make_empty();
        for (const auto& _p : _pts) {
            expand(_p);
        }
    }
    AABB(std::initializer_list<const P> _pts) {
        make_empty();
        for (const auto& _p : _pts) {
            expand(_p);
        }
    }
    inline bool empty() const {
        return min_[0]>max_[0];
    }
    inline void make_empty() {
        for (int i=0;i<Traits<P>::dim;i++) {
            min_[i] = std::numeric_limits<typename Traits<P>::value_type>::infinity();
            max_[i] = -min_[i];
        }
    }
    inline void expand(const AABB<P>& _aabb) {
        for (int i=0;i<Traits<P>::dim;i++) {
            min_[i] = std::min(min_[i], _aabb.min_[i]);
            max_[i] = std::max(max_[i], _aabb.max_[i]);
        }
    }
    inline void expand(const P& _p) {
        for (int i=0;i<Traits<P>::dim;i++) {
            min_[i] = std::min(min_[i], _p[i]);
            max_[i] = std::max(max_[i], _p[i]);
        }
    }
    inline FT squared_diagonal() const {
        FT diag(0);
        for (int i=0;i<Traits<P>::dim;i++) {
            FT d = max_[i]-min_[i];
            diag += d*d;
        }
        return diag;
    }
    inline FT diagonal() const {
        return std::sqrt<FT>(squared_diagonal());
    }
    inline AABB<P> scaled(FT _s) const {
        AABB<P> sbox;
        for (int i=0;i<Traits<P>::dim;i++) {
            FT d = static_cast<FT>(0.5)*(max_[i]-min_[i])*(_s-static_cast<FT>(1.0));
            sbox.min_[i] = min_[i] - d;
            sbox.max_[i] = max_[i] + d;
        }
        return sbox;
    }
    inline const P& min() const {
        return min_;
    }
    inline const P& max() const {
        return max_;
    }
    inline P& min() {
        return min_;
    }
    inline P& max() {
        return max_;
    }
    inline bool contains(const AABB<P>& _box) const {
        for (int i = 0; i < DIM; ++i) {
            if (_box.max_[i] < min_[i] || max_[i] < _box.min_[i]) {
                return false;
            }
        }
        return true;
    }
    inline std::array<P,1<<DIM> corners() const {
        std::array<P,1<<DIM> res;
        for (int32_t mask = 0; mask < (1<<DIM); ++mask) {
            for (int d = 0; d < DIM; ++d) {
                res[mask][d] = ((mask>>(DIM-1-d))&1)? max_[d] : min_[d];
            }
        }
        return res;
    }
    inline bool operator==(const AABB<P>& _aabb) const {
        return min_ == _aabb.min_ && max_ == _aabb.max_;
    }
private:
    P min_, max_;
};

template<vector P>
struct Traits<AABB<P>>
{
    using vector_type = P;
};

}
