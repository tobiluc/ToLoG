#pragma once
#include <ToLoG/geometry/Shapes.hpp>

namespace ToLoG
{

// Computes the barycentric coordinates of the point _p
// w.r.t. the triangle _tri, i.e. alpha, beta, gamma such that
// _p = _tri[0] * alpha + _tri[1] * beta + _tri[2] * gamma
// and alpha + beta + gamma = 1
// _p must lie within the triangle's plane!
template<vector PointT>
inline std::array<typename Traits<PointT>::value_type,3> barycentric_coordinates(const PointT& _p, const Triangle<PointT>& _tri)
{
    using FT = Traits<PointT>::value_type;
    constexpr int DIM = Traits<PointT>::dim;

    PointT ab = _tri[1] - _tri[0];
    PointT ac = _tri[2] - _tri[0];
    PointT ap = _p - _tri[0];

    // Compute barycentric coordinates
    FT d00 = 0, d01 = 0, d11 = 0, d20 = 0, d21 = 0;
    for (int i = 0; i < DIM; ++i) {
        d00 += ab[i]*ab[i];
        d01 += ab[i]*ac[i];
        d11 += ac[i]*ac[i];
        d20 += ap[i]*ab[i];
        d21 += ap[i]*ac[i];
    }
    FT denom = d00*d11 - d01*d01;
    FT u = 0, v = 0;
    u = (d11*d20 - d01*d21) / denom;
    v = (d00*d21 - d01*d20) / denom;
    return {FT(1)-u-v, u, v};
}

template<typename PointT>
inline std::array<typename Traits<PointT>::value_type,4> barycentric_coordinates(const PointT& _p, const Tetrahedron<PointT>& _tet)
{
    using FT = Traits<PointT>::value_type;
    constexpr int DIM = Traits<PointT>::dim;

    PointT AB = _tet[1] - _tet[0];
    PointT AC = _tet[2] - _tet[0];
    PointT AD = _tet[3] - _tet[0];
    PointT AP = _p - _tet[0];

    FT d00=0,d01=0,d02=0,d11=0,d12=0,d22=0,d0p=0,d1p=0,d2p=0;
    for (int i=0;i<DIM;i++){
        d00 += AB[i]*AB[i];
        d01 += AB[i]*AC[i];
        d02 += AB[i]*AD[i];
        d11 += AC[i]*AC[i];
        d12 += AC[i]*AD[i];
        d22 += AD[i]*AD[i];
        d0p += AB[i]*AP[i];
        d1p += AC[i]*AP[i];
        d2p += AD[i]*AP[i];
    }

    FT denom = d00*(d11*d22 - d12*d12) - d01*(d01*d22 - d12*d02) + d02*(d01*d12 - d11*d02);
    FT u=0,v=0,w=0;
    if (denom != 0) {
        u = (d0p*(d11*d22 - d12*d12) - d01*(d1p*d22 - d12*d2p) + d02*(d1p*d12 - d11*d2p)) / denom;
        v = (d00*(d1p*d22 - d12*d2p) - d0p*(d01*d22 - d12*d02) + d02*(d01*d2p - d1p*d02)) / denom;
        w = (d00*(d11*d2p - d1p*d12) - d01*(d01*d2p - d1p*d02) + d0p*(d01*d12 - d11*d02)) / denom;
    }
    return {FT(1)-u-v-w, u, v, w};
}

}
