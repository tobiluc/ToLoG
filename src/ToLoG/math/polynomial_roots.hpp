#pragma once

#include <vector>

namespace ToLoG
{

template<typename FT>
std::vector<FT> solve_quadratic(FT a, FT b, FT c)
{
    // Handle degenerate linear case
    if (is_near_zero(a)) {
        if (std::abs(b) < 1e-12) {return {};}
        return {-c/b};
    }

    FT discriminant = b*b - 4*a*c;

    if (is_near_zero(discriminant)) {
        // 1 solution
        return {-b/(static_cast<FT>(2.0)*a)};
    } else if (discriminant > static_cast<FT>(0.0)) {
        // 2 solutions
        FT sqrtDiscriminant = std::sqrt<FT>(discriminant);
        FT s0 =(-b - sqrtDiscriminant) / (static_cast<FT>(2.0)*a);
        return {s0, s0 + sqrtDiscriminant / a};
    } else {
        // no solutions
        return {};
    }
}

template<typename FT>
std::vector<FT> solve_cubic(FT a3, FT a2, FT a1, FT a0)
{
    // Handle degenerate quadratic case
    if (is_near_zero(a3)) {
        return solve_quadratic(a2, a1, a0);
    }

    // Normalize equation
    FT tmp = 1.0/a3;
    a3 = 1.0; a2 *= tmp; a1 *= tmp; a0 *= tmp;

    // Calculate helpers
    FT tmp2 = a2/3.;
    FT q = a1/3. - tmp2*tmp2;
    FT r = (a1*a2 - 3.*a0)/6. - tmp2*tmp2*tmp2;

    tmp = r*r + q*q*q;
    if (tmp > 0) {
        // Case r^2 + q^3 > 0 => One real solution
        FT A = std::cbrt<FT>(std::abs(r) + std::sqrt<FT>(tmp));
        FT t1 = (r>=0)? A - q/A : q/A - A;
        return {(t1 - a2/FT(3.))};
    } else {
        // Case r^2 + q^3 <= 0 => Three real solutions
        FT theta = (q==0)? 0. : std::acos<FT>(r/std::pow<FT>(-q, 1.5));

        FT phi1 = theta / static_cast<FT>(3.);
        FT phi2 = phi1 - static_cast<FT>(2.*M_PI/3.);
        FT phi3 = phi1 + static_cast<FT>(2.*M_PI/3.);

        // Solutions are sorted from smallest to biggest
        tmp = FT(2.)*std::sqrt<FT>(-q);
        return {
            tmp*std::cos(phi3) - tmp2,
            tmp*std::cos(phi2) - tmp2,
            tmp*std::cos(phi1) - tmp2
        };
    }
}

template<typename FT>
std::vector<FT> solve_quartic(FT a4, FT a3, FT a2, FT a1, FT a0)
{
    // Handle degenerate cubic case
    if (is_near_zero(a4)) {
        return solve_cubic(a3, a2, a1, a0);
    }

    // Normalize equation
    FT tmp = 1.0/a4;
    a4 = 1.0; a3 *= tmp; a2 *= tmp; a1 *= tmp; a0 *= tmp;

    // Solve Cubic
    std::vector<FT> solnsCubic = solve_cubic(1.0, -a2, a1*a3 - 4.*a0, 4.*a0*a2 - a1*a1 - a0*a3*a3);
    FT u = solnsCubic.back(); // Greatest real solution of cubic

    // Solutions will be sorted from small to large.
    FT sigma = (a1 - a3*u*0.5)>0 ? 1. : -1.;

    tmp = std::sqrt<FT>(a3*a3*0.25 + u - a2);
    FT p1 = a3*0.5 - tmp;
    FT p2 = a3*0.5 + tmp;

    tmp = sigma*std::sqrt<FT>(u*u*0.25 - a0);
    FT q1 = u*0.5 + tmp;
    FT q2 = u*0.5 - tmp;

    std::vector<FT> sols;
    sols.reserve(4);

    tmp = std::sqrt<FT>(p2*p2*0.25 - q2);
    sols.push_back(-p2*0.5 - tmp);
    sols.push_back(-p2*0.5 + tmp);

    tmp = std::sqrt<FT>(p1*p1*0.25 - q1);
    sols.push_back(-p1*0.5 - tmp);
    sols.push_back(-p1*0.5 + tmp);

    return sols;
}

}
