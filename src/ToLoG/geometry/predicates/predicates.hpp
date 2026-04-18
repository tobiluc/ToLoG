#pragma once

#include <cassert>
#include <ostream>
#include <ToLoG/vector_concepts.hpp>

namespace ToLoG
{

enum class ORI {
    CCW = -1,
    ZERO = 0,
    CW = 1
};

inline constexpr ORI sign_ori(const double d) {
    return static_cast<ORI>((d > 0.0)-(d < 0.0));
}

inline constexpr ORI flip_ori(const ORI o) {
    return (o==ORI::CCW)? ORI::CW : (o==ORI::CW)? ORI::CCW : ORI::ZERO;
}

inline constexpr bool is_opposite_ori(const ORI& o1, const ORI& o2) {
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

double insphere(const double* pa, const double* pb, const double* pc, const double*  pd, const double* pe);

#ifdef __cplusplus
} // extern "C"
#endif

namespace ToLoG
{

/// Wrapper around ::orient2d. Returns the result as an ::ORIENTATION.
static inline ORI sign_orient2d(const double* pa, const double* pb, const double* pc) {
    return sign_ori(orient2d(pa, pb, pc));
}

template<vector_2_double P>
static inline ORI sign_orient2d(const P& a, const P& b, const P& c) {
    return sign_ori(orient2d(a.data(), b.data(), c.data()));
}

template<vector_3_double P>
static inline ORI sign_orient2d_xy(const P& a, const P& b, const P& c) {
    using FT = Traits<P>::value_type;
    FT pa[2] = {a[0],a[1]};
    FT pb[2] = {b[0],b[1]};
    FT pc[2] = {c[0],c[1]};
    return sign_ori(orient2d(pa, pb, pc));
}

template<vector_3_double P>
static inline ORI sign_orient2d_yz(const P& a, const P& b, const P& c) {
    using FT = Traits<P>::value_type;
    FT pa[2] = {a[1],a[2]};
    FT pb[2] = {b[1],b[2]};
    FT pc[2] = {c[1],c[2]};
    return sign_ori(orient2d(pa, pb, pc));
}

template<vector_3_double P>
static inline ORI sign_orient2d_xz(const P& a, const P& b, const P& c) {
    using FT = Traits<P>::value_type;
    FT pa[2] = {a[0],a[2]};
    FT pb[2] = {b[0],b[2]};
    FT pc[2] = {c[0],c[2]};
    return sign_ori(orient2d(pa, pb, pc));
}

static inline ORI sign_incircle(const double* pa, const double* pb, const double* pc, const double* pd) {
    return sign_ori(incircle(pa, pb, pc, pd));
}

template<vector_2_double P>
static inline ORI sign_incircle(const P& a, const P& b, const P& c, const P& d) {
    return sign_ori(incircle(a.data(), b.data(), c.data(), d.data()));
}

/// Wrapper around ::orient3d. Returns the result as an ::ORIENTATION.
static inline ORI sign_orient3d(const double* pa, const double* pb, const double* pc, const double* pd) {
    return sign_ori(orient3d(pa, pb, pc, pd));
}

template<vector_3_double P>
static inline ORI sign_orient3d(const P& a, const P& b, const P& c, const P& d) {
    return sign_ori(orient3d(a.data(), b.data(), c.data(), d.data()));
}

static inline ORI sign_insphere(const double* pa, const double* pb, const double* pc, const double* pd, const double* pe) {
    return sign_ori(insphere(pa, pb, pc, pd, pe));
}

template<vector_3_double P>
static inline ORI sign_insphere(const P& a, const P& b, const P& c, const P& d, const P& e) {
    return sign_ori(insphere(a.data(), b.data(), c.data(), d.data(), e.data()));
}

class PredicatesInitalizer
{
    PredicatesInitalizer();
    static PredicatesInitalizer instance;
};

}
