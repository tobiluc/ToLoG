#pragma once

#include <MacTypes.h>
#include <algorithm>
#include <limits>
#include <initializer_list>
#include <cmath>

namespace ToLoG
{

// define common properties for primitives
template<typename T>
struct Traits {};

// returns true if T is ToLoG::Point
template<typename T>
struct is_point : std::false_type {};

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
requires(DIM > 0)
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

template<typename P>
requires(is_point<P>::value)
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
requires(is_point<P>::value)
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
        return t_[0] == _tri[0]
            && t_[1] == _tri[1]
            && t_[2] == _tri[2];
    }
private:
    P t_[3];
};

template<typename P>
requires(is_point<P>::value)
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
requires(Traits<P>::dim == 3 && is_point<P>::value)
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
    inline P centroid() const {
        return (t_[0] + t_[1] + t_[2] + t_[3]) / FT(4);
    }
    inline AABB<P> aabb() const {
        return AABB<P>({t_[0],t_[1],t_[2],t_[3]});
    }
    inline bool operator==(const Tetrahedron<P>& _tet) const {
        return t_[0] == _tet[0]
               && t_[1] == _tet[1]
               && t_[2] == _tet[2]
               && t_[3] == _tet[3];
    }
private:
    P t_[4];
};

#include <ToLoG/Traits.inl.hpp>

template<int N, typename PointT, typename PrimT>
requires(is_point<PointT>::value)
std::array<typename Traits<PointT>::value_type,N> barycentric_coordinates(const PointT& _p, const PrimT& _prim);
#include <ToLoG/barycentric.inl.hpp>

template<typename PointT, typename PrimT>
requires(is_point<PointT>::value)
Traits<PointT>::value_type point_squared_distance(const PointT& _p, const PrimT& _prim);
#include <ToLoG/point_squared_distance.inl.hpp>


}
