#pragma once
#include <array>
#include <cassert>
#include <iostream>
#include <ostream>

namespace ToLoG
{

class TetTopology
{
private:
    using bits = uint8_t;
    template<typename T>
    inline constexpr static bits b(const T _t) {
        return static_cast<bits>(_t);
    }
public:
    enum class V : bits;
    enum class HE : bits;
    enum class HF : bits;
    enum class VAR : bits;

    // Encoding is as follows
    // [type, v2, v1, v0]

    enum class VAR : bits {
        O=0b00000000,

        A=0b01000000,
        B=0b01000001,
        C=0b01000010,
        D=0b01000011,

        AB=0b10000100,
        AC=0b10001000,
        AD=0b10001100,
        BA=0b10000001,
        BC=0b10001001,
        BD=0b10001101,
        CA=0b10000010,
        CB=0b10000110,
        CD=0b10001110,
        DA=0b10000011,
        DB=0b10000111,
        DC=0b10001011,

        BDC=0b11101101,
        CDA=0b11001110,
        DBA=0b11000111,
        ABC=0b11100100,

        DCB=0b11011011,
        DAC=0b11100011,
        BAD=0b11110001,
        BCA=0b11001001,

        CBD=0b11110110,
        ACD=0b11111000,
        ADB=0b11011100,
        CAB=0b11010010,

        ABCD=0b11111100
    };
    enum class V : bits {
        A=static_cast<bits>(VAR::A),
        B=static_cast<bits>(VAR::B),
        C=static_cast<bits>(VAR::C),
        D=static_cast<bits>(VAR::D)
    };
    enum class HE : bits {
        AB=static_cast<bits>(VAR::AB),
        AC=static_cast<bits>(VAR::AC),
        AD=static_cast<bits>(VAR::AD),
        BA=static_cast<bits>(VAR::BA),
        BC=static_cast<bits>(VAR::BC),
        BD=static_cast<bits>(VAR::BD),
        CA=static_cast<bits>(VAR::CA),
        CB=static_cast<bits>(VAR::CB),
        CD=static_cast<bits>(VAR::CD),
        DA=static_cast<bits>(VAR::DA),
        DB=static_cast<bits>(VAR::DB),
        DC=static_cast<bits>(VAR::DC)
    };
    enum class HF : bits {
        BDC=static_cast<bits>(VAR::BDC),
        CDA=static_cast<bits>(VAR::CDA),
        DBA=static_cast<bits>(VAR::DBA),
        ABC=static_cast<bits>(VAR::ABC),

        DCB=static_cast<bits>(VAR::DCB),
        DAC=static_cast<bits>(VAR::DAC),
        BAD=static_cast<bits>(VAR::BAD),
        BCA=static_cast<bits>(VAR::BCA),

        CBD=static_cast<bits>(VAR::CBD),
        ACD=static_cast<bits>(VAR::ACD),
        ADB=static_cast<bits>(VAR::ADB),
        CAB=static_cast<bits>(VAR::CAB)
    };
    inline constexpr static uint8_t i(V _v) {
        return b(_v)&3;
    }
    inline constexpr static V v(const bits _b) {
        return static_cast<V>(0b01000000|(_b&3));
    }
    inline constexpr static bool is_vertex(VAR _var) {
        return (b(_var)&0b11000000)==0b01000000;
    }
    inline constexpr static bool is_halfedge(VAR _var) {
        return (b(_var)&0b11000000)==0b10000000;
    }
    inline constexpr static bool is_halfface(VAR _var) {
        return (b(_var)&0b11000000)==0b11000000;
    }
    inline constexpr static bool is_tet(VAR _var) {
        return (b(_var)&0b11111100)==0b11111100;
    }
    inline constexpr static bool is_valid(VAR _var) {
        return (b(_var)&0b11000000)!=0b00000000;
    }
    inline constexpr static bool is_same_halfface(HF _hf1, HF _hf2) {
        return opp(_hf1)==opp(_hf2);
    }
    inline constexpr static bool is_same_edge(HE _he1, HE _he2) {
        return _he1==_he2||_he1==opp(_he2);
    }
    inline constexpr static HE he(V _v0, V _v1) {
        return he((0b10<<6)|(i(_v1)<<2)|i(_v0));
    }
    inline constexpr static HF hf(V _v0, V _v1, V _v2) {
        return hf((0b11<<6)|((b(_v2)&3)<<4)|((b(_v1)&3)<<2)|(b(_v0)&3));
    }
    inline constexpr static V v(HE _he, uint8_t _idx) {
        return v((0b01<<6)|((b(_he)&(3<<(_idx<<1)))>>(_idx<<1)));
    }
    inline constexpr static V v0(HE _he) {
        return v(_he,0);
    }
    inline constexpr static V v1(HE _he) {
        return v(_he,1);
    }
    inline constexpr static V v0(HF _hf) {
        return v(_hf,0);
    }
    inline constexpr static V v1(HF _hf) {
        return v(_hf,1);
    }
    inline constexpr static V v2(HF _hf) {
        return v(_hf,2);
    }
    inline constexpr static V v(HF _hf, uint8_t _idx) {
        return v((0b01<<6)|((b(_hf)&(3<<(_idx<<1)))>>(_idx<<1)));
    }
    inline constexpr static HE opp(HE _he) {
        return he(v(_he,1), v(_he,0));
    }
    inline constexpr static V opp(HF _hf) {
        return fourth_vertex(v(_hf,0), v(_hf,1), v(_hf,2));
    }
    inline constexpr static HF opp(V v) {
        return v_opp_hf_[i(v)];
    }
    inline constexpr static HE he(HF _hf, uint8_t _idx) {
        return he(v(_hf,_idx), v(_hf,(_idx+1)%3));
    }
    inline constexpr static HE he0(HF _hf) {
        return he(_hf, 0);
    }
    inline constexpr static HE he1(HF _hf) {
        return he(_hf, 1);
    }
    inline constexpr static HE he2(HF _hf) {
        return he(_hf, 2);
    }
    inline constexpr static bool contains(HE _he, V _v) {
        return v(_he,0) == _v || v(_he,1) == _v;
    }
    inline constexpr static bool contains(HF _hf, V _v) {
        return v(_hf,0) == _v || v(_hf,1) == _v || v(_hf,2) == _v;
    }
    inline constexpr static const std::array<HF,3> incident_halffaces(V _v) {
        return v_inc_hf_[i(_v)];
    }
    inline constexpr static const std::array<HF,2> incident_halffaces(V _v1, V _v2) {
        assert(_v1!=_v2);
        const auto hfs1 = incident_halffaces(_v1);
        const auto hfs2 = incident_halffaces(_v2);
        std::array<HF,2> hfs;
        int idx(0);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (hfs1[i] == hfs2[j]) {
                    hfs[idx++] = hfs1[i];
                    break;
                }
            }
        }
        return hfs;
    }
    inline constexpr static const std::array<HE,3> incident_halfedges(V v) {
        return {he(v,add<1>(v)),he(v,add<2>(v)),he(v,add<3>(v))};
    }
    inline constexpr static const HE he(HF _hf1, HF _hf2) {
        for (int i = 0; i < 3; ++i) {
            const HE he1 = he(_hf1, i);
            for (int j = 0; j < 3; ++j) {
                if (he(_hf2,j) == opp(he1)) {
                    return he1;
                }
            }
        }
        assert(false);
        return {};
    }
    // inline constexpr static const HF incident_halfface(HE _he) {
    //     // const V a = v0(_he);
    //     // const V b = v1(_he);
    //     // // c is one of the two missing vertices

    //     // if (_he == HE::AB) {return HF::ABC;}
    //     // if (_he == HE::BA) {return HF::BAD;}
    //     // ...

    // }
    // inline constexpr static const HE next_halfedge(HF _hf, HE _he) {
    //     return _he;
    // }

private:
    inline constexpr static HE he(const bits _b) {return static_cast<HE>(_b);}
    inline constexpr static HF hf(const bits _b) {return static_cast<HF>(_b);}
    constexpr static const std::array<std::array<HF,3>,4> v_inc_hf_ = {{
        {HF::ABC,HF::ACD,HF::ADB},
        {HF::BAD,HF::BCA,HF::BDC},
        {HF::CAB,HF::CBD,HF::CDA},
        {HF::DAC,HF::DBA,HF::DCB}
    }};
    constexpr static const std::array<HF,4> v_opp_hf_ = {{
        HF::BDC,
        HF::CDA,
        HF::DBA,
        HF::ABC
    }};
    template<uint8_t a>
    inline constexpr static V add(V _v) {
        return v(0b01000000|((b(_v)+a)&3));
    }
    inline constexpr static V fourth_vertex(V _v0, V _v1, V _v2) {
        return v((0b01<<6)|(6-i(_v0)-i(_v1)-i(_v2)));
    }
};
// inline constexpr bool operator==(typename TetTopology::HF hf1, typename TetTopology::HF hf2) {
//     return TetTopology::opp(hf1)==TetTopology::opp(hf2);
// }

}
