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
        set_axes(_axes);
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
        for (FT& r : ell.radii_) {r *= _scale;}
        return ell;
    }
    constexpr P axis(int i) const {
        return direction(i) * radius(i);
    }
    constexpr std::array<P,DIM> axes() const {
        std::array<P,DIM> res;
        for (int i = 0; i < DIM; ++i) {res[i] = axis(i);}
        return res;
    }
    void set_axis(int i, const P& _axis)
    {
        directions_[i] = normalized(_axis);
        radii_[i] = norm(_axis);
        assert(radii_[i] > 0);
    }
    void set_axes(const std::array<P, DIM>& _axes)
    {
        for (int i = 0; i < DIM; ++i) {set_axis(i, _axes[i]);}
        assert(check_orthogonal());
    }
    bool check_orthogonal() const
    {
        for (int i = 0; i < DIM; ++i) {
            for (int j = i+1; j < DIM; ++j) {
                if (std::abs(dot(direction(i), direction(j))) > 1e-14) {
                    return false;
                }
            }
        }
        return true;
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
