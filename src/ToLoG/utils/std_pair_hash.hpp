#pragma once

#include <bit>
#include <cstddef>
#include <utility>

namespace std
{

template<class A,class B>
struct hash<pair<A,B>>{
    size_t operator() (const pair<A,B>& p) const {
        return std::rotl(hash<A>{}(p.first),1) ^
               hash<B>{}(p.second);
    }
};

}
