#pragma once
#include <array>
#include <cassert>
#include <vector>

namespace ToLoG
{

struct [[deprecated("For its specific usage, replaced by tet_topology")]] SimplexIndices
{
public:
    SimplexIndices(const std::vector<int>& _indices)
        : size_(_indices.size())
    {
        for (int i = 0; i < size_; ++i) {
            indices_[i] = _indices[i];
        }
    }
    SimplexIndices() : SimplexIndices(std::vector<int>{}) {}
    inline unsigned int size() const {return size_;}
    inline bool empty() const {return size_==0;}
    inline bool is_point() const {return size_==1;}
    inline bool is_segment() const {return size_==2;}
    inline bool is_triangle() const {return size_==3;}
    inline bool is_tet() const {return size_==4;}
    inline bool contains(int _v) const {
        for (int i = 0; i < size_; ++i) {if (indices_[i]==_v) {return true;}}
        return false;
    }
    inline int operator[](int _i) const {return indices_[_i];}
    template<int dim>
    inline std::array<int,dim> array() const {
        assert(dim==size_);
        std::array<int,dim> r;
        for (int i = 0; i < size_; ++i) {r[i] = indices_[i];}
        return r;
    }
private:
    unsigned int size_;
    std::array<int,4> indices_;
};

}
