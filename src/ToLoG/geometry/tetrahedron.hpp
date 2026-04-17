#pragma once
#include <ToLoG/Traits_fwd.hpp>
#include <ToLoG/geometry/segment.hpp>
#include <ToLoG/geometry/triangle.hpp>
#include <span>
#include <ToLoG/mesh/tet_topology.hpp>

namespace ToLoG
{

template<vector P>
class Tetrahedron
{
public:
    Tetrahedron() {
    }
    Tetrahedron(const P& _a, const P& _b, const P& _c, const P& _d)
    {
        t_[0] = _a;
        t_[1] = _b;
        t_[2] = _c;
        t_[3] = _d;
    }
    Tetrahedron(std::span<const P> _t)
    {
        assert(_t.size() == 4);
        t_[0] = _t[0];
        t_[1] = _t[1];
        t_[2] = _t[2];
        t_[3] = _t[3];
    }
    inline const P& operator[](const int& _i) const {
        return t_[_i];
    }
    inline const P& operator[](const TetTopology::V& _v) const {
        return t_[TetTopology::i(_v)];
    }
    inline Segment<P> segment(int _i, int _j) const {
        return Segment<P>(t_[_i], t_[_j]);
    }
    inline Segment<P> segment(const TetTopology::HE& _he) const {
        return Segment<P>(
            this->operator[](TetTopology::v0(_he)),
            this->operator[](TetTopology::v1(_he)));
    }
    inline Triangle<P> triangle(int _i, int _j, int _k) const {
        return Triangle<P>(t_[_i], t_[_j], t_[_k]);
    }
    inline Triangle<P> triangle(const TetTopology::HF& _hf) const {
        return Triangle<P>(
            this->operator[](TetTopology::v0(_hf)),
            this->operator[](TetTopology::v1(_hf)),
            this->operator[](TetTopology::v2(_hf)));
    }
    inline bool operator==(const Tetrahedron<P>& _tet) const {
        return    t_[0] == _tet[0]
               && t_[1] == _tet[1]
               && t_[2] == _tet[2]
               && t_[3] == _tet[3];
    }
private:
    P t_[4];
};

template<vector P>
struct Traits<Tetrahedron<P>>
{
    using vector_type = P;
};


}
