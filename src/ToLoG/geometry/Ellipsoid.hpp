#pragma once
#include <ToLoG/vector_concepts.hpp>
#include <ToLoG/geometry/vector_math.hpp>
#include <array>

namespace ToLoG
{

template<vector P>
class Ellipsoid
{
public:
    using FT = typename Traits<P>::value_type;
    static constexpr int DIM = Traits<P>::dim;

    Ellipsoid() {
    }
    Ellipsoid(const P& _center, const std::array<P,DIM>& _axes) :
        center_(_center)
    {
        for (int i = 0; i < DIM; ++i) {
            directions_[i] = normalized(_axes[i]);
            radii_[i] = norm(_axes[i]);
            assert(radii_[i] > 0);
        }
#ifndef NDEBUG
        for (int i = 0; i < DIM; ++i) {
            assert(std::abs(radii_[i])>1e-14);
            for (int j = i+1; j < DIM; ++j) {
                assert(std::abs(dot(_axes[i], _axes[j]))<=1e-14);
            }
        }
#endif
    }
    const P& center() const {
        return center_;
    }
    P& center() {
        return center_;
    }
    const P& direction(int i) const {
        return directions_[i];
    }
    const FT& radius(int i) const {
        return radii_[i];
    }
    Ellipsoid<P> scaled(FT _scale) const {
        Ellipsoid<P> ell = *this;
        for (int i = 0; i < DIM; ++i) {
            ell.radii_[i] *= _scale;
        }
        return ell;
    }
    bool operator==(const Ellipsoid<P>& _e) const {
        return center_ == _e.center_ && directions_ == _e.directions_
               && radii_ == _e.radii_;
    }
private:
    P center_;
    std::array<P,DIM> directions_;
    std::array<FT,DIM> radii_;
};

template<vector P>
struct Traits<Ellipsoid<P>>
{
    using vector_type = P;
};

}
