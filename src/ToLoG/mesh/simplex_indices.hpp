#pragma once
#include <array>
#include <vector>

namespace ToLoG
{

struct SimplexIndices
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
    inline bool is_vertex() const {return size_==1;}
    inline bool is_edge() const {return size_==2;}
    inline bool is_face() const {return size_==3;}
    inline bool is_tet() const {return size_==4;}
    inline bool contains(int _v) const {
        for (int i = 0; i < size_; ++i) {if (indices_[i]==_v) {return true;}}
        return false;
    }
    inline int operator[](int _i) const {return indices_[_i];}
    template<typename T, typename Vec>
    inline std::vector<T> read(const Vec& _vec) const {
        std::vector<T> r;
        r.reserve(size_);
        for (int i = 0; i < size_; ++i) {r.push_back(_vec[indices_[i]]);}
        return r;
    }
private:
    unsigned int size_;
    std::array<int,4> indices_;
};

}
