#pragma once
#include <ToLoG/Traits_fwd.hpp>

namespace ToLoG
{

template<vector P>
class Segment
{
public:
    Segment() {}
    Segment(const P& _start, const P& _end) :
        start_(_start), end_(_end)
    {}
    inline const P& start() const {return start_;}
    inline const P& end() const {return end_;}
    inline Segment reversed() const {return Segment(end_, start_);}

    inline P& start() {return start_;}
    inline P& end() {return end_;}
    inline void reverse() {std::swap(start_, end_);}

    inline bool operator==(const Segment<P>& _s) const {
        return start_ == _s.start_ && end_ == _s.end_;
    }
private:
    P start_, end_;
};

template<vector P>
struct Traits<Segment<P>>
{
    using vector_type = P;
};

}
