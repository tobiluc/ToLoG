#pragma once
#include <ToLoG/geometry/Shapes.hpp>
#include <ToLoG/geometry/predicates/predicates.hpp>

namespace ToLoG
{

template<vector P>
bool is_degenerate(const Segment<P>& _seg) {
    return _seg.start() == _seg.end();
}

template<vector_2_double P>
bool is_degenerate(const Triangle<P>& _tri) {
    return sign_orient2d(_tri[0], _tri[1], _tri[2]) == ORI::ZERO;
}

template<vector_3_double P>
bool is_degenerate(const Tetrahedron<P>& _tet) {
    return sign_orient3d(_tet[0], _tet[1], _tet[2], _tet[3]) == ORI::ZERO;
}

template<vector_3_double P>
bool is_degenerate(const Triangle<P>& _tri) {
    return sign_orient2d_xy(_tri[0], _tri[1], _tri[2]) == ORI::ZERO
        && sign_orient2d_yz(_tri[0], _tri[1], _tri[2]) == ORI::ZERO
        && sign_orient2d_xz(_tri[0], _tri[1], _tri[2]) == ORI::ZERO;
}

}
