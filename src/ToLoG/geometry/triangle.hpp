#pragma once
#include <ToLoG/geometry/Segment.hpp>

namespace ToLoG
{

template<vector P>
class Triangle
{
private:
    using FT = Traits<P>::value_type;
public:
    Triangle() {}
    Triangle(const P& _a, const P& _b, const P& _c)
    {
        t_[0] = _a;
        t_[1] = _b;
        t_[2] = _c;
    }
    inline const P& operator[](int _i) const {
        return t_[_i];
    }
    inline Segment<P> segment(int _i) const {
        return Segment<P>(t_[_i], t_[(_i+1)%3]);
    }
    inline Triangle<P> reversed() const {
        return Triangle<P>(t_[2], t_[1], t_[0]);
    }
    inline void reverse() {
        std::swap(t_[0],t_[2]);
    }
    inline bool operator==(const Triangle<P>& _tri) const {
        return t_[0] == _tri[0]
               && t_[1] == _tri[1]
               && t_[2] == _tri[2];
    }
private:
    P t_[3];
};

template<vector P>
struct Traits<Triangle<P>>
{
    using vector_type = P;
};


}
