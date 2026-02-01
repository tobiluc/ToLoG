#pragma once

#include <cassert>
#include <ostream>
#include <array>
#include <ToLoG/Traits_fwd.hpp>

namespace ToLoG
{

enum class ORI {
    CCW = -1,
    ZERO = 0,
    CW = 1,
};

inline bool is_opposite_ori(const ORI& o1, const ORI& o2) {
    return (o1==ORI::CCW && o2==ORI::CW) || (o1==ORI::CW && o2==ORI::CCW);
}

/// Maps an ::ORIENTATION to a string. Useful for debugging output.
inline std::ostream& operator<<(std::ostream& os, const ORI& ori){
    static const char *strs[] = {
        "ORI::CCW", "ORI::ZERO", "ORI::CW"
    };
    assert((int)ori >= -1 && (int)ori <= 1);
    return os << strs[(int)ori + 1];
}

}

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the exact predicates.
 *
 * This function has to be called before calling ::orient2d.
 */
void exactinit();

/**
 * @brief Computes the orientation of the supplied points.
 *
 * This function only returns correct values if exactinit() was called before.
 *
 * @param pa, pb, pc Arrays containing the two coordinates of a point, each.
 * @return A value the sign of which reflects the orientation of the three points.
 */
double orient2d(const double* pa, const double* pb, const double* pc);

/**
 * @brief Computes the orientation of the supplied points.
 *
 * This function only returns correct values if exactinit() was called before.
 *
 * @param pa, pb, pc, pd Arrays containing the three coordinates of a point, each.
 * @return A value the sign of which reflects the orientation of the four points.
 */
double orient3d(const double* pa, const double* pb, const double* pc, const double*  pd);

double incircle(const double* pa, const double* pb, const double* pc, const double*  pd);

#ifdef __cplusplus
} // extern "C"
#endif

namespace ToLoG
{

template<typename P> requires(is_vector_type<P>::value && Traits<P>::dim==2)
static inline double point_incircle(const P& a, const P& b, const P& c, const P& d) {
    const double pa[2] = {static_cast<double>(a[0]),static_cast<double>(a[1])};
    const double pb[2] = {static_cast<double>(b[0]),static_cast<double>(b[1])};
    const double pc[2] = {static_cast<double>(c[0]),static_cast<double>(c[1])};
    const double pd[2] = {static_cast<double>(d[0]),static_cast<double>(d[1])};
    return incircle(pa, pb, pc, pd);
}

/// Wrapper around ::orient2d. Returns the result as an ::ORIENTATION.
static inline ORI sign_orient2d(const double* pa, const double* pb, const double* pc) {
    const double result = orient2d(pa, pb, pc);
    // A little convoluted but branchless.
    return (ORI) ((result > 0.0) - (result < 0.0));
}

template<typename P> requires(is_vector_type<P>::value && Traits<P>::dim==2)
static inline double point_orient2d(const P& a, const P& b, const P& c) {
    const double pa[2] = {static_cast<double>(a[0]),static_cast<double>(a[1])};
    const double pb[2] = {static_cast<double>(b[0]),static_cast<double>(b[1])};
    const double pc[2] = {static_cast<double>(c[0]),static_cast<double>(c[1])};
    return orient2d(pa, pb, pc);
}

static inline double abs_orient2d(const double* pa, const double* pb, const double* pc) {
    const double result = orient2d(pa, pb, pc);
    return (result >= 0.0)? result : -result;
}

/// Wrapper around ::orient3d. Returns the result as an ::ORIENTATION.
static inline ORI sign_orient3d(const double* pa, const double* pb, const double* pc, const double* pd) {
    const double result = orient3d(pa, pb, pc, pd);
    // A little convoluted but branchless.
    return (ORI) ((result > 0.0) - (result < 0.0));
}

template<typename P> requires(is_vector_type<P>::value && Traits<P>::dim==3)
static inline double point_orient3d(const P& a, const P& b, const P& c, const P& d) {
    const double pa[3] = {static_cast<double>(a[0]),static_cast<double>(a[1]),static_cast<double>(a[2])};
    const double pb[3] = {static_cast<double>(b[0]),static_cast<double>(b[1]),static_cast<double>(b[2])};
    const double pc[3] = {static_cast<double>(c[0]),static_cast<double>(c[1]),static_cast<double>(c[2])};
    const double pd[3] = {static_cast<double>(d[0]),static_cast<double>(d[1]),static_cast<double>(d[2])};
    return orient3d(pa, pb, pc, pd);
}

static inline double abs_orient3d(const double* pa, const double* pb, const double* pc, const double* pd) {
    const double result = orient3d(pa, pb, pc, pd);
    return (result >= 0.0)? result : -result;
}

static inline char sign(const double& x) {return ((x > 0.0) - (x < 0.0));}

class PredicatesInitalizer
{
    PredicatesInitalizer();
    static PredicatesInitalizer instance;
};

}
