#pragma once
#include <ToLoG/Traits_fwd.hpp>

namespace ToLoG
{

template<vector P>
class Ray
{
public:
    Ray(P _o, P _d) : o_(_o), d_(normalized(_d)) {}
    inline const P& origin() const {return o_;}
    inline const P& dir() const {return d_;}
    inline P point(Traits<P>::value_type _t) const {return o_ + d_*_t;}
private:
    P o_;
    P d_;
};

template<typename Point>
struct Traits<Ray<Point>>
{
    using value_type = Traits<Point>::value_type;
    using vector_type = Point;
    constexpr static int dim = Traits<Point>::dim;
};

}
