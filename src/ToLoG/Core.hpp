#pragma once

#include <ToLoG/predicates/ExactPredicates.hpp>
#include <MacTypes.h>
#include <concepts>
#include <limits>
#include <numeric>

namespace ToLoG
{

template<typename T>
struct Traits {};

template<typename P>
class AABB
{
public:
    using FT = typename Traits<P>::value_type;
    static constexpr int DIM = Traits<P>::dim;

    AABB() {
        make_empty();
    }
    AABB(std::initializer_list<P> _pts) {
        make_empty();
        for (const auto& _p : _pts) {
            expand(_p);
        }
    }
    inline bool empty() const {
        return DIM==0 || (min_[0]>max_[0]);
    }
    inline void make_empty() {
        for (int i=0;i<Traits<P>::dim;i++) {
            min_[i] = std::numeric_limits<typename Traits<P>::value_type>::infinity();
            max_[i] = -min_[i];
        }
    }
    inline void expand(const AABB<P>& _aabb) {
        for (int i=0;i<Traits<P>::dim;i++) {
            min_[i] = std::min(min_[i], _aabb.min_[i]);
            max_[i] = std::max(max_[i], _aabb.max_[i]);
        }
    }
    inline void expand(const P& _p) {
        for (int i=0;i<Traits<P>::dim;i++) {
            min_[i] = std::min(min_[i], _p[i]);
            max_[i] = std::max(max_[i], _p[i]);
        }
    }
    inline const P& min() const {
        return min_;
    }
    inline const P& max() const {
        return max_;
    }
    inline std::array<P,1<<DIM> corners() const {
        std::array<P,1<<DIM> res;
        for (int32_t mask = 0; mask < (1<<DIM); ++mask) {
            for (int d = 0; d < DIM; ++d) {
                res[mask][d] = ((mask>>(DIM-1-d))&1)? max_[d] : min_[d];
            }
        }
        return res;
    }
    inline P centroid() const {
        return (min_ + max_) / typename Traits<P>::value_type(2);
    }
    inline AABB aabb() const {
        return *this;
    }
    inline bool operator==(const AABB<P>& _aabb) const {
        return min_ == _aabb.min_ && max_ == _aabb.max_;
    }
private:
    P min_, max_;
};

template<typename FT, int DIM>
class Point
{
public:
    Point() {
        for (int i=0;i<DIM;++i) {
            data_[i] = FT(0);
        }
    }
    Point(const FT* _data) {
        for (int i = 0; i < DIM; ++i) {
            data_[i] = _data[i];
        }
    }
    template<typename... Args,
             typename = std::enable_if_t<sizeof...(Args) == DIM>>
    Point(Args&&... args)
        : data_{ static_cast<FT>(args)... }
    {}
    static inline Point filled(const FT& _value) {
        Point p;
        for (int i = 0; i < DIM; ++i) {
            p[i] = _value;
        };
        return p;
    }
    inline size_t size() const {
        return static_cast<size_t>(DIM);
    }
    inline uint32_t argmax() const {
        uint32_t idx(0);
        for (int i = 0; i < DIM; ++i) {
            if (data_[i] > data_[idx]) {
                idx = i;
            }
        };
        return idx;
    }
    inline uint32_t argmin() const {
        uint32_t idx(0);
        for (int i = 0; i < DIM; ++i) {
            if (data_[i] < data_[idx]) {
                idx = i;
            }
        };
        return idx;
    }
    inline Point abs() const {
        Point res;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] = (data_[i]>=0)? data_[i] : -data_[i];
        }
        return res;
    }
    inline FT& operator[](const int& _i) {
        return data_[_i];
    }
    inline const FT& operator[](const int& _i) const {
        return data_[_i];
    }
    inline const FT& x() const {static_assert(DIM>=1); return data_[0];}
    inline const FT& y() const {static_assert(DIM>=2); return data_[1];}
    inline const FT& z() const {static_assert(DIM>=3); return data_[2];}
    inline Point<FT,DIM> operator-(const Point<FT,DIM>& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] -= _rhs.data_[i];
        }
        return res;
    }
    inline Point<FT,DIM> operator+(const Point<FT,DIM>& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] += _rhs.data_[i];
        }
        return res;
    }
    inline Point<FT,DIM> operator*(const FT& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] *= _rhs;
        }
        return res;
    }
    inline Point<FT,DIM> operator/(const FT& _rhs) const {
        Point<FT,DIM> res = *this;
        for (int i = 0; i < DIM; ++i) {
            res.data_[i] /= _rhs;
        }
        return res;
    }
    inline FT dot(const Point<FT,DIM>& _rhs) const {
        FT res(0);
        for (int i = 0; i < DIM; ++i) {
            res += data_[i] * _rhs.data_[i];
        }
        return res;
    }
    inline FT squared_norm() const {
        return this->dot(*this);
    }
    inline FT norm() const {
        return std::sqrt<FT>(squared_norm());
    }
    inline Point cross(const Point& _rhs) {
        static_assert(DIM==3, "Cross product is only defined in 3 dimensions");
        return Point(
            data_[1]*_rhs[2] - data_[2]*_rhs[1],
            data_[2]*_rhs[0] - data_[0]*_rhs[2],
            data_[0]*_rhs[1] - data_[1]*_rhs[0]
        );
    }
    inline const Point<FT,DIM>& centroid() const {
        return *this;
    }
    inline AABB<Point<FT,DIM>> aabb() const {
        return AABB<Point<FT,DIM>>({*this});
    }
    inline const FT* data() const {
        return data_;
    }
    inline bool operator==(const Point<FT,DIM>& _p) const {
        for (int i = 0; i < DIM; ++i) {
            if (data_[i] != _p.data_[i]) {
                return false;
            }
        }
        return true;
    }
    inline bool operator<(const Point& _rhs) const {
        return std::lexicographical_compare(
            data_, data_+DIM,
            _rhs.data_, _rhs.data_+DIM
        );
    }
    friend inline std::ostream& operator<<(std::ostream& _os, const Point& _p) {
        if (DIM == 0) {return _os;}
        for (int i = 0; i < DIM-1; ++i) {_os << _p[i] << " ";}
        return _os << _p[DIM-1];
    }
private:
    FT data_[DIM];
};
template<typename FT, int DIM>
using Vector = Point<FT, DIM>;

template<typename P, int N>
class Simplex
{
public:
    using FT = typename Traits<P>::value_type;
    static constexpr int DIM = Traits<P>::dim;

    Simplex() {}
    template<typename... Args,
             typename = std::enable_if_t<sizeof...(Args) == N>>
    Simplex(Args&&... args)
        : points_{static_cast<P>(args)...}
    {}
    inline P centroid() const {
        P res;
        for (int i = 0; i < N; ++i) {res = res + points_[i];}
        return res / static_cast<FT>(N);
    }
    inline AABB<P> aabb() const {
        AABB<P> res;
        for (int i = 0; i < N; ++i) {res.expand(points_[i]);}
        return res;
    }
    inline bool operator==(const Simplex<P,N>& _s) const {
        for (int i = 0; i < N; ++i) {
            if (points_[i] != _s.points_[i]) {
                return false;
            }
        }
        return true;
    }
private:
    P points_[N];
};

template<typename P>
class Segment
{
public:
    using FT = typename Traits<P>::value_type;
    static constexpr int DIM = Traits<P>::dim;

    Segment() {}
    Segment(const P& _start, const P& _end) :
        start_(_start), end_(_end)
    {}
    inline const P& start() const {
        return start_;
    }
    inline const P& end() const {
        return end_;
    }
    inline Segment reversed() const {
        return Segment(end_, start_);
    }
    inline FT squared_norm() const {
        return (end_ - start_).squared_norm();
    }
    inline FT norm() const {
        return (end_ - start_).norm();
    }
    inline P centroid() const {
        return (start_ + end_) / typename Traits<P>::value_type(2);
    }
    inline AABB<P> aabb() const {
        return AABB<P>({start_, end_});
    }
    inline bool operator==(const Segment<P>& _s) const {
        return start_ == _s.start_ && end_ == _s.end_;
    }
private:
    P start_, end_;
};

template<typename P>
class Triangle
{
public:
    using FT = typename Traits<P>::value_type;
    static constexpr int DIM = Traits<P>::dim;

    Triangle() {}
    Triangle(const P& _a, const P& _b, const P& _c)
    {
        t_[0] = _a;
        t_[1] = _b;
        t_[2] = _c;
    }
    inline const P& operator[](const int& _i) const {
        return t_[_i];
    }
    inline FT area() const {
        P u = t_[1] - t_[0];
        P v = t_[2] - t_[0];
        FT uu = u.dot(u);
        FT vv = v.dot(v);
        FT uv = u.dot(v);
        FT det = uu * vv - uv * uv;
        return std::sqrt(std::max<FT>(det, FT(0))) / FT(2);
    }
    inline FT circumference() const {
        return (t_[1]-t_[0]).norm() + (t_[2]-t_[1]).norm() + (t_[0]-t_[2]).norm();
    }
    inline Segment<P> edge(int _i) const {
        return Segment<P>(t_[_i], t_[(_i+1)%3]);
    }
    inline P normal() const {
        static_assert(DIM==3, "Triangle normal is only defined in 3 dimensions");
        return (t_[1] - t_[0]).cross(t_[2] - t_[0]);
    }
    inline P centroid() const {
        return (t_[0] + t_[1] + t_[2]) / FT(3);
    }
    inline AABB<P> aabb() const {
        return AABB<P>({t_[0], t_[1], t_[2]});
    }
    inline bool operator==(const Triangle<P>& _tri) const {
        return t_[0] == _tri[0] && t_[1] == _tri[1] && t_[2] == _tri[2];
    }
private:
    P t_[3];
};

template<typename P>
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
    inline const P& centroid() const {
        return center_;
    }
    inline const FT& radius() const {
        return radius_;
    }
    inline FT squared_radius() const {
        return radius_*radius_;
    }

    inline AABB<P> aabb() const {
        return AABB<P>({
            center_ - P::filled(radius()),
            center_ + P::filled(radius())
        });
    }
    inline bool operator==(const Sphere<P>& _s) const {
        return center_ == _s.center_ && radius_ == _s.radius_;
    }
private:
    P center_;
    FT radius_;
};

template<typename P>
class Tetrahedron
{
public:
    using FT = typename Traits<P>::value_type;
    static constexpr int DIM = Traits<P>::dim;

    Tetrahedron() {
    }
    Tetrahedron(const P& _a, const P& _b, const P& _c, const P& _d)
    {
        t_[0] = _a;
        t_[1] = _b;
        t_[2] = _c;
        t_[3] = _d;
    }
    inline const P& operator[](const int& _i) const {
        return t_[_i];
    }
    inline Segment<P> edge(int _i, int _j) const {
        return Segment<P>(t_[_i], t_[_j]);
    }
    inline Triangle<P> face(int _i, int _j, int _k) const {
        return Triangle<P>(t_[_i], t_[_j], t_[_k]);
    }
    inline ORI orientation() const {
        return sign_orient3d(t_[0].data(), t_[1].data(), t_[2].data(), t_[3].data());
    }
    inline P centroid() const {
        return (t_[0] + t_[1] + t_[2] + t_[3]) / FT(4);
    }
    inline AABB<P> aabb() const {
        return AABB<P>({t_[0],t_[1],t_[2],t_[4]});
    }
    inline bool operator==(const Tetrahedron<P>& _tet) const {
        return t_[0] == _tet[0]
               && t_[1] == _tet[1]
               && t_[2] == _tet[2]
               && t_[2] == _tet[2];
    }
private:
    P t_[4];
};

//=========================
// Traits
//=========================
template<typename P>
struct Traits<AABB<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename FT, int DIM>
struct Traits<Point<FT,DIM>>
{
    using value_type = FT;
    using vector_type = Point<FT,DIM>;
    constexpr static int dim = DIM;
};

template<typename P>
struct Traits<Segment<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename P>
struct Traits<Triangle<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename P, int N>
struct Traits<Simplex<P,N>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename P>
struct Traits<Sphere<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename P>
struct Traits<Tetrahedron<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

//=========================
// Barycentric
//=========================
template<int N, typename PointT, typename PrimT>
std::array<typename Traits<PointT>::value_type,N> barycentric_coordinates(const PointT& _p, const PrimT& _prim);

template<typename PointT>
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
    if (denom != 0) {
        u = (d11*d20 - d01*d21) / denom;
        v = (d00*d21 - d01*d20) / denom;
    }
    return {u, v, FT(1)-u-v};
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
    return {u, v, w, FT(1)-u-v-w};
}


//=========================
// Distance Functions
//=========================
template<typename PointT, typename PrimT>
Traits<PointT>::value_type point_squared_distance(const PointT& _p, const PrimT& _prim);

template<typename FT, int DIM>
inline FT point_squared_distance(const Point<FT,DIM>& _p, const Point<FT,DIM>& _q)
{
    return (_p - _q).squared_norm();
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const AABB<PointT>& _aabb)
{
    using FT = Traits<PointT>::value_type;
    FT res(0);
    for (int i = 0; i < Traits<PointT>::dim; ++i) {
        FT d = std::max(std::max(
            _aabb.min()[i] - _p[i],
            FT(0)),
            _p[i] - _aabb.max()[i]
        );
        res += d*d;
    }
    return res;
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const Segment<PointT>& _s)
{
    using FT = Traits<PointT>::value_type;
    if (_s.end() == _s.start()) {
        return point_squared_distance(_p, _s.start());
    }
    PointT ab = _s.end() - _s.start();
    PointT ap = _p - _s.start();
    FT t = std::clamp(ap.dot(ab) / ab.squared_norm(), FT(0), FT(1));
    PointT closest = _s.start() + ab*t;
    return (_p - closest).squared_norm();
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const Sphere<PointT>& _s)
{
    using FT = Traits<PointT>::value_type;
    FT d = std::max((_p - _s.centroid()).norm() - _s.radius(), static_cast<FT>(0));
    return d*d;
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const Triangle<PointT>& _tri)
{
    using FT = Traits<PointT>::value_type;
    constexpr int DIM = Traits<PointT>::dim;

    std::array<FT,3> bary = barycentric_coordinates(_p. _tri);

    // If inside triangle, project onto triangle plane
    if (bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0) {
        Point<FT,DIM> q = _tri[0] + (_tri[1] - _tri[0])*bary[0] + (_tri[2] - _tri[0])*bary[1]; // closest point
        return (_p - q).squared_norm();
    }

    // Otherwise, closest point is on an edge
    return std::min({
        point_squared_distance(_p, _tri.edge(0)),
        point_squared_distance(_p, _tri.edge(1)),
        point_squared_distance(_p, _tri.edge(2))
    });
}

template<typename PointT>
inline Traits<PointT>::value_type point_squared_distance(const PointT& _p, const Tetrahedron<PointT>& _tet)
{
    using FT = Traits<PointT>::value_type;

    const std::array<FT, 4> bary = barycentric_coordinates(_p, _tet);

    // Check if point is inside tetrahedron
    if (bary[0] >= 0 && bary[1] >= 0 && bary[2] >= 0 && bary[3] >= 0) {
        return static_cast<FT>(0);
    }

    // Otherwise, closest point lies on a face triangle
    return std::min({
        point_squared_distance(_p, _tet.face(0,1,2)),
        point_squared_distance(_p, _tet.face(0,1,3)),
        point_squared_distance(_p, _tet.face(0,2,3)),
        point_squared_distance(_p, _tet.face(1,2,3))
    });
}


}
