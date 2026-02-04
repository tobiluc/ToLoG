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
 * If _p lies exactly on the face given by the first three vertices, returns {0,1,2}
 * If _p is outside the tet, returns {}. If _p is strictly inside the tet, returns {0,1,2,3}.
 */
template<typename P>
requires(is_vector_type<P>::value
        && Traits<P>::dim == 3
        && std::is_same<typename Traits<P>::value_type,double>::value)
std::vector<int> exact_simplex_of_point_in_tet(
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
        std::cerr << "Warning: Cannot evaluate exact_simplex_of_point_in_tet in tet with volume zero" << std::endl;
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

/**
 * Returns the Simplex of the Tetrahedron given by the tet's vertex indices
 * on which the ray originating from _p and going in direction (_q - _p) lies.
 */
template<typename P>
    requires(is_vector_type<P>::value
             && Traits<P>::dim == 3
             && std::is_same<typename Traits<P>::value_type,double>::value)
std::vector<int> exact_simplex_of_ray_in_tet(
    const Tetrahedron<P>& _tet,
    const P& _p, const P& _q,
    std::optional<std::vector<int>> _p_simplex = std::nullopt)
{
    if (!_p_simplex.has_value()) {_p_simplex = exact_simplex_of_point_in_tet(_tet, _p);}
    const auto& vs = _p_simplex.value();
    if (vs.empty() || (_p==_q)) {return {};}

    if (vs.size() == 4) {
        // from tet into tet
        return {0,1,2,3};
    }

    const ORI tet_ori = sign_orient3d(_tet[0].data(),_tet[1].data(),_tet[2].data(),_tet[3].data());
    if (tet_ori == ORI::ZERO) {
        std::cerr << "Warning: Cannot evaluate exact_simplex_of_ray_in_tet in tet with volume zero" << std::endl;
        return {};
    }
    const ORI opp_ori = (tet_ori==ORI::CCW)? ORI::CW : ORI::CCW;

    if (vs.size() == 3) {
        // _p lies on a face, evaluate ori of _q w.r.t. that face
        const ORI hf_ori = sign_orient3d(
            _tet[vs[0]].data(),
            _tet[vs[1]].data(),
            _tet[vs[2]].data(),
            _q.data()
        );
        if (hf_ori == tet_ori) {return {0,1,2,3};} // into tet
        if (hf_ori == ORI::ZERO) {return vs;} // along face
        return {}; // outside
    }

    if (vs.size() == 2) {
        // _p lies on an edge, evaluate ori of _q w.r.t. the two incident faces

        // Get the two remaining tet vertices
        // s.t. vs[0], vs[1], v2 is one halfface
        // and vs[1], vs[0], v3 is the other
        int v2, v3;
        for (v2=0;v2<4;++v2) {if (vs[0]!=v2&&vs[1]!=v2) {break;}}
        for (v3=0;v3<4;++v3) {if (vs[0]!=v3&&vs[1]!=v3&&v2!=v3) {break;}}
        if (sign_orient3d(
            _tet[vs[0]].data(),
            _tet[vs[1]].data(),
            _tet[v2].data(),
            _tet[v3].data()
        ) != tet_ori) {std::swap(v2,v3);}

        const ORI o0 = sign_orient3d(
            _tet[vs[0]].data(),
            _tet[vs[1]].data(),
            _tet[v2].data(),
            _q.data()
        );
        const ORI o1 = sign_orient3d(
            _tet[vs[1]].data(),
            _tet[vs[0]].data(),
            _tet[v3].data(),
            _q.data()
        );
        if (o0 == tet_ori && o1 == tet_ori) {
            // into the tet
            return {0,1,2,3};
        }
        if (o0 == ORI::ZERO && o1 == tet_ori) {
            // along 1st face
            return {vs[0],vs[1],v2};
        }
        if (o0 == tet_ori && o1 == ORI::ZERO) {
            // along 2nd face
            return {vs[1],vs[0],v3};
        }
        if (o0 == ORI::ZERO && o1 == ORI::ZERO) {
            // along edge (should be get the order matching the direction?)
            return {vs[0],vs[1]};
        }

        // outside
        return {};
    }

    if (vs.size() == 1) {
        // _p corresponds to a vertex. Evaluate ori of _q w.r.t. the three incident faces

        // Get the three indicent faces
        std::array<std::vector<int>,3> hfs;
        if (vs[0]==0) {hfs = {{{0,1,2},{0,2,3},{0,3,1}}};}
        if (vs[0]==1) {hfs = {{{0,1,2},{0,3,1},{1,3,2}}};}
        if (vs[0]==2) {hfs = {{{0,1,2},{0,2,3},{1,3,2}}};}
        if (vs[0]==3) {hfs = {{{0,2,3},{0,3,1},{1,3,2}}};}

        // Evaluate oris
        std::array<ORI,3> oris;
        for (int i = 0; i < 3; ++i) {
            oris[i] = sign_orient3d(
                _tet[hfs[i][0]].data(),
                _tet[hfs[i][1]].data(),
                _tet[hfs[i][2]].data(),
                _q.data()
            );
            if (oris[i] == opp_ori) {
                // outside
                return {};
            }
        }
        if (oris[0]==tet_ori&&oris[1]==tet_ori&&oris[2]==tet_ori) {
            // into tet
            return {0,1,2,3};
        }
        for (int i0 = 0; i0 < 3;++i0) {
            if (oris[i0]==ORI::ZERO) {
                int i1 = (i0+1)%3;
                int i2 = (i0+2)%3;
                if (oris[i1] != ORI::ZERO && oris[i2] != ORI::ZERO) {
                    // along face
                    return hfs[i0];
                }
                if (!(oris[i1] == ORI::ZERO && oris[i2] != ORI::ZERO)) {std::swap(i1,i2);}
                if (oris[i1] == ORI::ZERO && oris[i2] != ORI::ZERO) {
                    // along two faces -> common edge
                    // find the 2nd vertex (which must not be equal to _p
                    // and be contained in both faces
                    for (int v1 : hfs[i0]) {
                        if (v1 != vs[0] &&
                            (v1==hfs[i1][0]||v1==hfs[i1][1]||v1==hfs[i1][2])) {
                            return {vs[0], v1};
                        }
                    }
                    assert(false);
                    return {};
                }
            }
        }
        assert(false);
        return {};
    }
    assert(false);
    return {};
}

template<typename P>
    requires(is_vector_type<P>::value
             && Traits<P>::dim == 3
             && std::is_same<typename Traits<P>::value_type,double>::value)
bool exact_ray_cuts_face_within_tet(
    const Tetrahedron<P>& _tet,
    const P& _p, const P& _q,
    const std::vector<int>& _f)
{
    assert(_f.size()==3);
    const ORI tet_ori = sign_orient3d(_tet[0].data(),_tet[1].data(),_tet[2].data(),_tet[3].data());
    if (tet_ori == ORI::ZERO) {
        std::cerr << "Warning: Cannot evaluate exact_ray_through_face_in_tet in tet with volume zero" << std::endl;
        return {};
    }
    const ORI opp_ori = (tet_ori==ORI::CCW)? ORI::CW : ORI::CCW;

    // Primary direction must cut through the plane given by the face
    if (sign_orient3d(_tet[_f[0]].data(), _tet[_f[1]].data(), _tet[_f[2]].data(), _p.data()) != tet_ori ||
        sign_orient3d(_tet[_f[0]].data(), _tet[_f[1]].data(), _tet[_f[2]].data(), _q.data()) != opp_ori) {
        return false;
    }

    // Get the orientations of the 3 tets formed around u, u+d1
    std::array<ORI, 3> oris = {
        sign_orient3d(_tet[_f[0]].data(), _tet[_f[1]].data(), _p.data(), _q.data()),
        sign_orient3d(_tet[_f[1]].data(), _tet[_f[2]].data(), _p.data(), _q.data()),
        sign_orient3d(_tet[_f[2]].data(), _tet[_f[0]].data(), _p.data(), _q.data())
    };

    // Check if primary direction cuts through interior/center of triangle
    if (oris[0] == oris[1] && oris[0] == oris[2]) {
        assert(oris[0] == opp_ori);
        return true;
    }

    // Check if primary direction does not cut through triangle at all
    //if (!std::any_of(oris.begin(), oris.end(), [](ORIENTATION ori){return ori == ORI_ZERO;}) {
    if (std::any_of(oris.begin(), oris.end(), [](ORI ori){return ori == ORI::CW;})
        && std::any_of(oris.begin(), oris.end(), [](ORI ori){return ori == ORI::CCW;}))
    {
        return false;
    }

    // Otherwise the primary direction cuts through the triangles boundary (edge or vertex).
    return true;
}

}
