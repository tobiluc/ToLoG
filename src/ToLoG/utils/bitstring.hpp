#pragma once

#include <string>

namespace ToLoG
{

template<typename I>
inline std::string bitstring(I _b) {
    constexpr int n_bits = sizeof(_b)*8;
    std::string s = "0b";
    for (int i = n_bits-1; i >= 0; --i) {
        s += ((_b >> i) & 1)? "1" : "0";
    }
    return s;
}

}
