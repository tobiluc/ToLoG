#pragma once

#include <concepts>

namespace ToLoG
{

// An Index that is used to access a polgon's mesh elements
// is either directly convertible to an int or is itself
// an object that has a function idx() whose return type is convertible to an int
template<class I>
concept mesh_index =
    std::convertible_to<I, std::size_t> ||
    requires(const I& i) {
        {i.idx()} -> std::convertible_to<std::size_t>;
    };

template<mesh_index I>
static constexpr int index(const I& _i) {
    if constexpr(std::convertible_to<I, int>) {return static_cast<int>(_i);
    } else {return static_cast<int>(_i.idx());}
}

}
