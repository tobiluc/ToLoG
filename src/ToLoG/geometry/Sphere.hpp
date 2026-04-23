#pragma once
#include <ToLoG/vector_concepts.hpp>

namespace ToLoG
{

template<vector P>
class Sphere
{
public:
    using FT = typename Traits<P>::value_type;
    static constexpr int DIM = Traits<P>::dim;

    Sphere() {
    }
    Sphere(const P& _center, const FT& _radius) :
        center_(_center), radius_(_radius)
    {}
    inline const P& center() const {
        return center_;
    }
    inline const FT& radius() const {
        return radius_;
    }
    inline FT squared_radius() const {
        return radius_*radius_;
    }
    inline bool operator==(const Sphere<P>& _s) const {
        return center_ == _s.center_ && radius_ == _s.radius_;
    }
private:
    P center_;
    FT radius_;
};

template<vector P>
struct Traits<Sphere<P>>
{
    using vector_type = P;
};

}
