#pragma once
#include <ToLoG/predicates/predicates_wrapper.hpp>
#include <ToLoG/Core.hpp>
#include <ToLoG/mesh/simplex_indices.hpp>
#include <ToLoG/utils/indices.hpp>
#include <iostream>

namespace ToLoG
{

template<vector_3d P>
ORI sign_orient3d(const Tetrahedron<P>& _tet)
{
    return sign_orient3d(_tet[0].data(),_tet[1].data(),_tet[2].data(),_tet[3].data());
}

template<vector_3d P>
ORI sign_orient3d(const Triangle<P>& _tri, const P& _p)
{
    return sign_orient3d(_tri[0].data(),_tri[1].data(),_tri[2].data(),_p.data());
}

/**
 * Returns the lowest dimensional Simplex of the Tetrahedron given by the tet's vertex indices
 * on which the query point _p lies.
 * For example, if _p lies exactly on the edge between the first and third vertex, returns {0,2}.
 * If _p lies exactly on the face given by the first three vertices, returns {0,1,2}
 * If _p is outside the tet, returns {}. If _p is strictly inside the tet, returns {0,1,2,3}.
 */
template<vector_3d P>
SimplexIndices supporting_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const P& _p)
{
    // Check against vertices directly
    for (int i = 0; i < 4; ++i) {if (_tet[i] == _p) {return SimplexIndices({i});}}

    // Get tet ori and opposite tet ori
    const ORI tet_ori = sign_orient3d(_tet[0].data(),_tet[1].data(),_tet[2].data(),_tet[3].data());
    if (tet_ori == ORI::ZERO) {
        std::cerr << "Warning: Cannot evaluate exact_simplex_of_point_in_tet in tet with volume zero" << std::endl;
        return SimplexIndices();
    }
    const ORI opp_ori = (tet_ori==ORI::CCW)? ORI::CW : ORI::CCW;

    // Evaluate ori against each halfface
    auto oris = std::array<ToLoG::ORI, 4>();
    for (int i = 0; i < 4; ++i) {
        const auto& f = tet_vertex_indices_ccw[i];
        oris[i] = sign_orient3d(_tet.triangle(f[0],f[1],f[2]), _p);
        if (oris[i] == opp_ori) {return SimplexIndices();}
    }

    // Get zero oris
    auto zeros = std::vector<char>();
    zeros.reserve(2);
    for (char i = 0; i <= 3; ++i) {if (oris[i] == ToLoG::ORI::ZERO) {zeros.push_back(i);}}

    // Determine simplex of tet which contains p
    if (zeros.size() == 2) { // Edge
        const auto& f1 = tet_vertex_indices_ccw[zeros[0]];
        const auto& f2 = tet_vertex_indices_ccw[zeros[1]];
        for (int i = 0; i < 3; ++i) {
            if (f1[i] != f2[0] && f1[i] != f2[1] && f1[i] != f2[2]) {
                int v0 = f1[(i+1)%3];
                int v1 = f1[(i+2)%3];
                if (v0 > v1) {std::swap(v0,v1);}
                return SimplexIndices({v0,v1});
            }
        }
        assert(false);
    }
    if (zeros.size() == 1) { // Face
        const auto& f = tet_vertex_indices_ccw[zeros[0]];
        return SimplexIndices({f[0],f[1],f[2]});
    }
    if (zeros.size() == 0) { // Tet
        return SimplexIndices({0,1,2,3});
    }

    // cannot happen
    assert(false);
    return SimplexIndices();
}

/**
 * Returns the lowest dimensional Simplex of the Tetrahedron given by the tet's vertex indices
 * on which the ray originating from _p and going in direction (_q - _p) lies.
 * Mathematically, returns the unique lowest dimensional simplex s s.t.
 * for all delta>0 there exists delta>eps>0 s.t. p + eps*(q-p) is contained in s.
 */
template<vector_3d P>
SimplexIndices supporting_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const Segment<P>& _s,
    std::optional<SimplexIndices> _p_simplex = std::nullopt)
{
    if (!_p_simplex.has_value()) {_p_simplex = supporting_simplex_in_tet(_tet, _s.start());}
    const auto& vs = _p_simplex.value();
    if (vs.empty() || (_s.start()==_s.end())) {return {};}

    if (vs.size() == 4) {
        // from tet into tet
        return SimplexIndices({0,1,2,3});
    }

    //const ORI tet_ori = sign_orient3d(_tet[0].data(),_tet[1].data(),_tet[2].data(),_tet[3].data());
    const ORI tet_ori = sign_orient3d(_tet);
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
            _s.end().data()
        );
        if (hf_ori == tet_ori) {return SimplexIndices({0,1,2,3});} // into tet
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
            _s.end().data()
        );
        const ORI o1 = sign_orient3d(
            _tet[vs[1]].data(),
            _tet[vs[0]].data(),
            _tet[v3].data(),
            _s.end().data()
        );
        if (o0 == tet_ori && o1 == tet_ori) {
            // into the tet
            return SimplexIndices({0,1,2,3});
        }
        if (o0 == ORI::ZERO && o1 == tet_ori) {
            // along 1st face
            return SimplexIndices({vs[0],vs[1],v2});
        }
        if (o0 == tet_ori && o1 == ORI::ZERO) {
            // along 2nd face
            return SimplexIndices({vs[1],vs[0],v3});
        }
        if (o0 == ORI::ZERO && o1 == ORI::ZERO) {
            // along edge
            return (vs[0]<vs[1])?
                SimplexIndices({vs[0],vs[1]}) :
                SimplexIndices({vs[1],vs[0]});
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
                _s.end().data()
            );
            if (oris[i] == opp_ori) {
                // outside
                return SimplexIndices();
            }
        }
        if (oris[0]==tet_ori&&oris[1]==tet_ori&&oris[2]==tet_ori) {
            // into tet
            return SimplexIndices({0,1,2,3});
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
                            return SimplexIndices({vs[0], v1});
                        }
                    }
                }
            }
        }
    }

    assert(false);
    return SimplexIndices();
}

template<vector_3d P>
struct TetSegmentIntersection
{
    SimplexIndices simplex; // vertex, edge or face
    P point; // intersection point of segment with piercing pierce_face
    std::array<int,3> pierce_face; // piercing face indices
};

/// Given a segment (_p, _q), finds the lowest dimensional simplex of the face of the tet
/// which contains the intersection point of the segment with the face.
/// It is expected that q lies outside the tet, and p lies on the opposite side of the face
/// compared to q.
template<vector_3d P>
TetSegmentIntersection<P> exiting_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const Segment<P>& _s, std::optional<std::array<int,3>> _exclude_f = std::nullopt)
{
    using FT = typename Traits<P>::value_type;

    const ORI tet_ori = sign_orient3d(_tet);
    if (tet_ori == ORI::ZERO) {
        std::cerr << "Warning: Cannot evaluate intersection_simplex_from_within_tet in tet with volume zero" << std::endl;
        return {};
    }
    const ORI opp_ori = (tet_ori==ORI::CCW)? ORI::CW : ORI::CCW;

    for (const auto& f : tet_vertex_indices_ccw) {
        if (_exclude_f.has_value() && f == _exclude_f.value()) {continue;}

        auto is_pt = [&]() -> P
        {
            const P d1 = _s.end() - _s.start();
            const P v1 = _tet[f[1]] - _tet[f[0]];
            const P v2 = _tet[f[2]] - _tet[f[0]];

            const P h = cross(d1, v2);
            const FT a = FT(1.0) / dot(v1, h);

            const P s = _s.start() - _tet[f[0]];
            const P q = cross(s, v1);

            const FT t = a * dot(v2, q);
            return _s.start() + d1 * t;
        };
        assert(f.size()==3);

        // Primary direction must cut through the plane given by the face
        if (sign_orient3d(_tet.triangle(f[0],f[1],f[2]), _s.start()) != tet_ori ||
            sign_orient3d(_tet.triangle(f[0],f[1],f[2]), _s.end()) != opp_ori) {
            continue;
        }

        // Get the orientations of the 3 tets formed around u, u+d1
        std::array<ORI, 3> oris = {
            sign_orient3d(_tet[f[0]].data(), _tet[f[1]].data(), _s.start().data(), _s.end().data()),
            sign_orient3d(_tet[f[1]].data(), _tet[f[2]].data(), _s.start().data(), _s.end().data()),
            sign_orient3d(_tet[f[2]].data(), _tet[f[0]].data(), _s.start().data(), _s.end().data())
        };

        // Check if primary direction cuts through interior/center of triangle
        if (oris[0] == oris[1] && oris[0] == oris[2]) {
            return {SimplexIndices({f[0],f[1],f[2]}), is_pt(), f};
        }

        // Check if primary direction does not cut through triangle at all
        //if (!std::any_of(oris.begin(), oris.end(), [](ORIENTATION ori){return ori == ORI_ZERO;}) {
        if (std::any_of(oris.begin(), oris.end(), [&](ORI ori){return ori == ORI::CCW;})
            && std::any_of(oris.begin(), oris.end(), [&](ORI ori){return ori == ORI::CW;}))
        {
            continue;
        }

        // Otherwise the primary direction cuts through the triangles boundary (edge or vertex).
        std::vector<int> zeros;
        for (int i = 0; i < 3; ++i) {
            if (oris[i] == ORI::ZERO) {zeros.push_back(i);}
        }
        assert(zeros.size() == 1 || zeros.size() ==2);
        if (zeros.size() == 1) {
            // Edge
            return {SimplexIndices({f[zeros.front()], f[(zeros.front()+1)%3]}), is_pt(), f};
        } else if (zeros.size() == 2) {
            // Vertex
            int i(-1);
            if (zeros[0] == 0 && zeros[1] == 1) {i = f[1];}
            else if (zeros[0] == 0 && zeros[1] == 2) {i = f[0];}
            else if (zeros[0] == 1 && zeros[1] == 2) {i = f[2];}
            assert(i >= 0);
            return {SimplexIndices({i}), _tet[i], f};
        }
    }
    return {};
}

template<vector_3d P>
TetSegmentIntersection<P> entering_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const Segment<P>& _s)
{
    return exiting_simplex_in_tet(_tet, _s.reversed());
}

}
