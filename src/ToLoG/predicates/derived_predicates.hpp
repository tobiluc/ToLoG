#pragma once
#include <ToLoG/predicates/predicates_wrapper.hpp>
#include <ToLoG/Core.hpp>
#include <iostream>

namespace ToLoG
{

/**
 * Returns the Simplex of the Tetrahedron given by the tet's vertex indices
 * on which the query point _p lies.
 * For example, if _p lies exactly on the edge between the first and third vertex, returns {0,2}.
 */
template<typename P>
requires(is_vector_type<P>::value
        && Traits<P>::dim == 3
        && std::is_same<typename Traits<P>::value_type,double>::value)
std::vector<int> exact_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const P& _p)
{
    // Check against vertices directly
    if (_tet[0] == _p) {return {0};}
    if (_tet[1] == _p) {return {1};}
    if (_tet[2] == _p) {return {2};}
    if (_tet[3] == _p) {return {3};}

    // Get tet ori and opposite tet ori
    const ORI tet_ori = sign_orient3d(_tet[0].data(),_tet[1].data(),_tet[2].data(),_tet[3].data());
    if (tet_ori == ORI::ZERO) {
        std::cerr << "Warning: Cannot evaluate exact_simplex_in_tet in tet with volume zero" << std::endl;
        return {};
    }
    const ORI opp_ori = (tet_ori==ORI::CCW)? ORI::CW : ORI::CCW;

    // Evaluate ori against each halfface
    auto oris = std::array<ToLoG::ORI, 4>();
    oris[0] = sign_orient3d(_tet[0].data(), _tet[1].data(), _tet[2].data(), _p.data());
    if (oris[0] == opp_ori) {return {};}
    oris[1] = sign_orient3d(_tet[0].data(), _tet[2].data(), _tet[3].data(), _p.data());
    if (oris[1] == opp_ori) {return {};}
    oris[2] = sign_orient3d(_tet[0].data(), _tet[3].data(), _tet[1].data(), _p.data());
    if (oris[2] == opp_ori) {return {};}
    oris[3] = sign_orient3d(_tet[1].data(), _tet[3].data(), _tet[2].data(), _p.data());
    if (oris[3] == opp_ori) {return {};}

    // Get zero oris
    auto zeros = std::vector<char>();
    zeros.reserve(2);
    for (char i = 0; i <= 3; ++i) {if (oris[i] == ToLoG::ORI::ZERO) {zeros.push_back(i);}}

    // Determine simplex of tet which contains p
    if (zeros.size() == 3) {
        // cannot happen
        assert(false);
    }
    if (zeros.size() == 2) {
        // Edge
        if (zeros[0] == 0 && zeros[1] == 1) {return {0,2};}
        else if (zeros[0] == 0 && zeros[1] == 2) {return {0,1};}
        else if (zeros[0] == 0 && zeros[1] == 3) {return {1,2};}
        else if (zeros[0] == 1 && zeros[1] == 2) {return {0,3};}
        else if (zeros[0] == 1 && zeros[1] == 3) {return {2,3};}
        else if (zeros[0] == 2 && zeros[1] == 3) {return {1,3};}
        assert(false);
    }
    if (zeros.size() == 1) {
        // Face
        if (zeros[0] == 0) {return {0,1,2};}
        else if (zeros[0] == 1) {return {0,2,3};}
        else if (zeros[0] == 2) {return {0,3,1};}
        else if (zeros[0] == 3) {return {1,3,2};}
    }
    if (zeros.size() == 0) {
        // Cell
        return {0,1,2,3};
    }

    assert(false);
    return {};
}

}
