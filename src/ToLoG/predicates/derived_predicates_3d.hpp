#pragma once
#include <ToLoG/predicates/predicates_wrapper.hpp>
#include <ToLoG/Core.hpp>
#include <ToLoG/utils/indices.hpp>
#include <iostream>
#include <ToLoG/mesh/tet_topology.hpp>

namespace ToLoG
{

template<vector_3d P>
ORI sign_orient3d(const Tetrahedron<P>& _tet)
{
    return sign_orient3d(_tet[0] ,_tet[1] ,_tet[2] ,_tet[3]);
}

template<vector_3d P>
ORI sign_orient3d(const Triangle<P>& _tri, const P& _p)
{
    return sign_orient3d(_tri[0], _tri[1], _tri[2], _p);
}

template<vector_3d P>
TetTopology::VAR supporting_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const P& _p)
{
    using TT = TetTopology;
    using S = TT::VAR;

    // Check against vertices directly
    for (TT::V v : {TT::V::A,TT::V::B,TT::V::C,TT::V::D}) {
        if (_tet[v] == _p) {return TT::var(v);}
    }

    // Get tet ori and opposite tet ori
    const ORI tet_ori = sign_orient3d(_tet);
    if (tet_ori == ORI::ZERO) {
        //std::cerr << "Warning: Cannot evaluate exact_simplex_of_point_in_tet in tet with volume zero" << std::endl;
        return S::O;
    }
    const ORI opp_ori = (tet_ori==ORI::CCW)? ORI::CW : ORI::CCW;

    // Evaluate ori against each halfface
    auto zeros = std::vector<TT::HF>();
    auto oris = std::array<ToLoG::ORI, 4>();
    for (TT::V v : {TT::V::A,TT::V::B,TT::V::C,TT::V::D}) {
        const auto i = TT::i(v);
        const auto hf = TT::opp_hf(v);
        oris[i] = sign_orient3d(_tet.triangle(hf), _p);
        if (oris[i] == opp_ori) {return S::O;}
        else if (oris[i] == ToLoG::ORI::ZERO) {zeros.push_back(hf);}
    }

    // Determine simplex of tet which contains p
    if (zeros.size() == 2) {return TT::var(TT::he(zeros[0], zeros[1]));}
    else if (zeros.size() == 1) {return TT::var(zeros[0]);}
    else if (zeros.size() == 0) {return S::ABCD;}

    // cannot happen
    assert(false);
    return S::O;
}

/**
 * Returns the lowest dimensional Simplex of the Tetrahedron given by the tet's vertex indices
 * on which the ray originating from _p and going in direction (_q - _p) lies.
 * Mathematically, returns the unique lowest dimensional simplex s s.t.
 * for all delta>0 there exists delta>eps>0 s.t. p + eps*(q-p) is contained in s.
 */
template<vector_3d P>
TetTopology::VAR supporting_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const Segment<P>& _s,
    std::optional<TetTopology::VAR> _p_simplex = std::nullopt)
{
    using TT = TetTopology;
    if (!_p_simplex.has_value()) {_p_simplex = supporting_simplex_in_tet(_tet, _s.start());}
    const auto ps = _p_simplex.value();
    if (!TT::is_valid(ps) || (_s.start()==_s.end())) {return TT::VAR::O;}

    if (TT::is_tet(ps)) {return ps;} // from tet into tet

    const ORI tet_ori = sign_orient3d(_tet);
    if (tet_ori == ORI::ZERO) {
        //std::cerr << "Warning: Cannot evaluate exact_simplex_of_ray_in_tet in tet with volume zero" << std::endl;
        return {};
    }
    const ORI opp_ori = (tet_ori==ORI::CCW)? ORI::CW : ORI::CCW;

    if (TT::is_halfface(ps)) {
        // _p lies on a face, evaluate ori of _q w.r.t. that face
        const ORI hf_ori = sign_orient3d(_tet.triangle(TT::hf(ps)), _s.end());
        if (hf_ori == tet_ori) {return TT::VAR::ABCD;} // into tet
        if (hf_ori == ORI::ZERO) {return ps;} // along face
        return TT::VAR::O; // outside
    }

    if (TT::is_halfedge(ps)) {
        // _p lies on an edge, evaluate ori of _q w.r.t. the two incident faces
        const TT::HE he = TT::he(ps);
        const auto hfs = TT::incident_halffaces(TT::v0(he), TT::v1(he));

        // Evaluate oris
        std::array<ORI,2> oris;
        std::vector<TT::HF> zero_hfs;
        for (int i = 0; i < 2; ++i) {
            oris[i] = sign_orient3d(_tet.triangle(hfs[i]), _s.end());
            if (oris[i] == opp_ori) {return TT::VAR::O;} // outside
            else if (oris[i] == ORI::ZERO) {zero_hfs.push_back(hfs[i]);}
        }

        if (zero_hfs.size() == 0) {
            assert((oris[0]==tet_ori&&oris[1]==tet_ori&&oris[2]==tet_ori));
            return TT::VAR::ABCD; // into tet
        }
        if (zero_hfs.size() == 1) {
            return TT::var(zero_hfs[0]); // into face
        }
        if (zero_hfs.size() == 2) {
            return TT::var(TT::he(zero_hfs[0], zero_hfs[1])); // into edge
        }
    }

    if (TT::is_vertex(ps)) {
        // _p corresponds to a vertex. Evaluate ori of _q w.r.t. the three incident faces

        // Get the three indicent faces
        const auto hfs = TT::incident_halffaces(TT::v(ps));

        // Evaluate oris
        std::array<ORI,3> oris;
        std::vector<TT::HF> zero_hfs;
        for (int i = 0; i < 3; ++i) {
            oris[i] = sign_orient3d(_tet.triangle(hfs[i]), _s.end());
            if (oris[i] == opp_ori) {return TT::VAR::O;} // outside
            else if (oris[i] == ORI::ZERO) {zero_hfs.push_back(hfs[i]);}
        }
        if (zero_hfs.size() == 0) {
            assert((oris[0]==tet_ori&&oris[1]==tet_ori&&oris[2]==tet_ori));
            return TT::VAR::ABCD; // into tet
        }
        if (zero_hfs.size() == 1) {
            return TT::var(zero_hfs[0]); // into face
        }
        if (zero_hfs.size() == 2) {
            return TT::var(TT::he(zero_hfs[0], zero_hfs[1])); // into edge
        }
    }

    assert(false);
    return TT::VAR::O;
}

template<vector_3d P>
struct TetSegmentIntersectionExp
{
    TetTopology::VAR simplex; // vertex, edge or face
    P point; // intersection point of segment with piercing pierce_face
    TetTopology::HF pierce_face; // piercing face
};

/**
 * Returns the lowest dimensional Simplex of the Tetrahedron given by the tet's vertex indices
 * on which the ray originating from _p and going in direction (_q - _p) lies (given by the segment s = (p,q)).
 * Mathematically, returns the unique lowest dimensional simplex s s.t.
 * for all delta>0 there exists delta>eps>0 s.t. p + eps*(q-p) is contained in s.
 */
template<vector_3d P>
TetSegmentIntersectionExp<P> exiting_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const Segment<P>& _s, std::optional<TetTopology::HF> _exclude_f = std::nullopt)
{
    using TT = TetTopology;
    using HF = TT::HF;
    using FT = typename Traits<P>::value_type;

    const ORI tet_ori = sign_orient3d(_tet);
    if (tet_ori == ORI::ZERO) {
        //std::cerr << "Warning: Cannot evaluate intersection_simplex_from_within_tet in tet with volume zero" << std::endl;
        return {TT::VAR::O};
    }
    const ORI opp_ori = (tet_ori==ORI::CCW)? ORI::CW : ORI::CCW;

    for (const auto hf : {HF::BDC,HF::CDA,HF::DBA,HF::ABC}) {
        if (_exclude_f.has_value() && hf == _exclude_f.value()) {continue;}

        auto is_pt = [&]() -> P
        {
            const P d1 = _s.end() - _s.start();
            const P v1 = _tet[TT::v1(hf)] - _tet[TT::v0(hf)];
            const P v2 = _tet[TT::v2(hf)] - _tet[TT::v0(hf)];

            const P h = cross(d1, v2);
            const FT a = FT(1.0) / dot(v1, h);

            const P s = _s.start() - _tet[TT::v0(hf)];
            const P q = cross(s, v1);

            const FT t = a * dot(v2, q);
            return _s.start() + d1 * t;
        };

        // Primary direction must cut through the plane given by the face
        if (sign_orient3d(_tet.triangle(hf), _s.start()) != tet_ori ||
            sign_orient3d(_tet.triangle(hf), _s.end()) != opp_ori) {
            continue;
        }

        // Get the orientations of the 3 tets formed around u, u+d1
        std::array<ORI, 3> oris;
        std::vector<TT::HE> zeros;
        for (int i = 0; i < 3; ++i) {
            const auto he = TT::he(hf, i);
            oris[i] = sign_orient3d(
                _tet[TT::v0(he)].data(),
                _tet[TT::v1(he)].data(),
                _s.start().data(), _s.end().data());
            if (oris[i] == ORI::ZERO) {zeros.push_back(he);}
        }

        // Check if primary direction cuts through interior/center of triangle
        if (oris[0] == oris[1] && oris[0] == oris[2]) {
            return {TT::var(hf), is_pt(), hf};
        }

        // Check if primary direction does not cut through triangle at all
        //if (!std::any_of(oris.begin(), oris.end(), [](ORIENTATION ori){return ori == ORI_ZERO;}) {
        if (std::any_of(oris.begin(), oris.end(), [&](ORI ori){return ori == ORI::CCW;})
            && std::any_of(oris.begin(), oris.end(), [&](ORI ori){return ori == ORI::CW;}))
        {
            continue;
        }

        // Otherwise the primary direction cuts through the triangles boundary (edge or vertex).
        assert(zeros.size() == 1 || zeros.size() ==2);
        if (zeros.size() == 1) {
            // Edge
            return {TT::var(zeros[0]), is_pt(), hf};
        } else if (zeros.size() == 2) {
            // Vertex
            const TT::V v = TT::v1(zeros[0]);
            assert(v == TT::v0(zeros[1]));
            return {TT::var(v), _tet[v], hf};
        }
    }
    return {TT::VAR::O};
}

template<vector_3d P>
TetSegmentIntersectionExp<P> entering_simplex_in_tet(
    const Tetrahedron<P>& _tet,
    const Segment<P>& _s)
{
    return exiting_simplex_in_tet(_tet, _s.reversed());
}

}
