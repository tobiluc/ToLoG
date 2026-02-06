#pragma once
#include <ToLoG/Core.hpp>
#include <numbers>

namespace ToLoG
{

template<typename P> requires(Traits<P>::dim == 2)
inline typename Traits<P>::value_type
winding_number(const P& _p, const Segment<P>& _seg)
{
    using FT = typename Traits<P>::value_type;

    P v0 = _seg.start() - _p;
    P v1 = _seg.end() - _p;

    FT angle = std::atan2(v1[1], v1[0]) - std::atan2(v0[1], v0[0]);

    constexpr FT pi = std::numbers::pi_v<FT>;
    if (angle < -pi) {angle += FT(2) * pi;}
    else if (angle > pi) {angle -= FT(2) * pi;}

    return angle / (FT(2)*pi);
}

template<typename P> requires(Traits<P>::dim==3)
inline typename Traits<P>::value_type winding_number(const P& _p, const Triangle<P>& _tri)
{
    using FT = typename Traits<P>::value_type;

    P v0 = _tri[0] - _p;
    P v1 = _tri[1] - _p;
    P v2 = _tri[2] - _p;

    FT n0 = norm(v0);
    if (is_near_zero(n0)) {return FT(0);};
    FT n1 = norm(v1);
    if (is_near_zero(n1)) {return FT(0);};
    FT n2 = norm(v2);
    if (is_near_zero(n2)) {return FT(0);};

    for (int i = 0; i < 3; ++i) {
        v0[i] /= n0;
        v1[i] /= n1;
        v2[i] /= n2;
    }

    return FT(2.) * std::atan2(
       dot(v0, cross(v1, v2)),
       FT(1.) + dot(v0, v1) + dot(v0, v2) + dot(v1, v2)
    ) / (FT(4)*std::numbers::pi_v<FT>);
}

}
