#pragma once
#include <ToLoG/vector_concepts.hpp>

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
    const P& start() const {return start_;}
    const P& end() const {return end_;}
    Segment reversed() const {return Segment(end_, start_);}

    P& start() {return start_;}
    P& end() {return end_;}
    void reverse() {std::swap(start_, end_);}

    bool operator==(const Segment<P>& _s) const {
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
