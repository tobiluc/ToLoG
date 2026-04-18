#include <ToLoG/math/polynomial_roots.hpp>
#include <ToLoG/math/epsilon.hpp>

namespace ToLoG
{

std::vector<double> solve_quadratic(double a, double b, double c)
{
    // Handle degenerate linear case
    if (is_near_zero(a)) {
        if (is_near_zero(b)) {return {};}
        return {-c/b};
    }

    double discriminant = b*b - 4*a*c;

    if (is_near_zero(discriminant)) {
        // 1 solution
        return {-b/(static_cast<double>(2.0)*a)};
    } else if (discriminant > static_cast<double>(0.0)) {
        // 2 solutions
        double sqrtDiscriminant = std::sqrt<double>(discriminant);
        double s0 =(-b - sqrtDiscriminant) / (static_cast<double>(2.0)*a);
        return {s0, s0 + sqrtDiscriminant / a};
    } else {
        // no solutions
        return {};
    }
}

std::vector<double> solve_cubic(double a3, double a2, double a1, double a0)
{
    // Handle degenerate quadratic case
    if (is_near_zero(a3)) {
        return solve_quadratic(a2, a1, a0);
    }

    // Normalize equation
    double tmp = 1.0/a3;
    a3 = 1.0; a2 *= tmp; a1 *= tmp; a0 *= tmp;

    // Calculate helpers
    double tmp2 = a2/3.;
    double q = a1/3. - tmp2*tmp2;
    double r = (a1*a2 - 3.*a0)/6. - tmp2*tmp2*tmp2;

    tmp = r*r + q*q*q;
    if (tmp > epsilon)
    {
        // Case r^2 + q^3 > 0 => One real solution
        double A = std::cbrt<double>(std::abs(r) + std::sqrt<double>(tmp));
        double t1 = (r>=0)? A - q/A : q/A - A;
        return {(t1 - a2/double(3.))};
    }
    else if (tmp < -epsilon)
    {
        // Case r^2 + q^3 <= 0 => Three real solutions
        double theta = (q==0)? 0. : std::acos<double>(r/std::pow<double>(-q, 1.5));

        double phi1 = theta / static_cast<double>(3.);
        double phi2 = phi1 - static_cast<double>(2.*M_PI/3.);
        double phi3 = phi1 + static_cast<double>(2.*M_PI/3.);

        // Solutions are sorted from smallest to biggest
        tmp = double(2.)*std::sqrt<double>(-q);

        return {
            tmp*std::cos(phi3) - tmp2,
            tmp*std::cos(phi2) - tmp2,
            tmp*std::cos(phi1) - tmp2
        };
    }
    else
    {
        // Case r^2 + q^3 = 0
        if (is_near_zero(r) && is_near_zero(q)) {
            return {-tmp2};
        } else {
            tmp = std::cbrt(r);
            std::vector<double> x = {2.0*tmp - tmp2, -tmp - tmp2};
            if (x[0] > x[1]) {std::swap(x[0], x[1]);}
            return x;
        }
    }
}

std::vector<double> solve_quartic(double a4, double a3, double a2, double a1, double a0)
{
    // Handle degenerate cubic case
    if (is_near_zero(a4)) {
        return solve_cubic(a3, a2, a1, a0);
    }

    // Normalize equation
    double tmp = 1.0/a4;
    a4 = 1.0; a3 *= tmp; a2 *= tmp; a1 *= tmp; a0 *= tmp;

    // Solve Cubic
    std::vector<double> solnsCubic = solve_cubic(1.0, -a2, a1*a3 - 4.*a0, 4.*a0*a2 - a1*a1 - a0*a3*a3);
    double u = solnsCubic.back(); // Greatest real solution of cubic

    // Solutions will be sorted from small to large.
    double sigma = (a1 - a3*u*0.5)>0 ? 1. : -1.;

    tmp = std::sqrt<double>(a3*a3*0.25 + u - a2);
    double p1 = a3*0.5 - tmp;
    double p2 = a3*0.5 + tmp;

    tmp = sigma*std::sqrt<double>(u*u*0.25 - a0);
    double q1 = u*0.5 + tmp;
    double q2 = u*0.5 - tmp;

    std::array<double,4> roots;

    tmp = std::sqrt<double>(p2*p2*0.25 - q2);
    roots[0] = (-p2*0.5 - tmp);
    roots[1] = (-p2*0.5 + tmp);

    tmp = std::sqrt<double>(p1*p1*0.25 - q1);
    roots[2] = (-p1*0.5 - tmp);
    roots[3] = (-p1*0.5 + tmp);

    // Remove duplicates
    std::vector<double> unique = {roots[0]};
    for (int i = 1; i < 4; ++i) {
        if (!is_near_zero(roots[i] - unique.back())) {
            unique.push_back(roots[i]);
        }
    }
    return unique;
}

}
