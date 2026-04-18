#pragma once

namespace ToLoG
{

static constexpr double epsilon = 1e-14;

inline constexpr bool is_near_zero(double _x) {
    return _x <= epsilon && _x >= -epsilon;
}

}
