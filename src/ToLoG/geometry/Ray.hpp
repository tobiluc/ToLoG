#pragma once
#include <ToLoG/vector_concepts.hpp>

namespace ToLoG
{

template<vector P>
class Ray
{
public:
    Ray(P _o, P _d) : o_(_o), d_(normalized(_d)) {}
    const P& origin() const {return o_;}
    const P& dir() const {return d_;}
    P point(Traits<P>::value_type _t) const {return o_ + d_*_t;}
private:
    P o_;
    P d_;
};

template<typename Point>
struct Traits<Ray<Point>>
{
    using vector_type = Point;
};

}
