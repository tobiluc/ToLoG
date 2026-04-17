#pragma once
#include <array>
#include <cassert>
#include <variant>

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
        O=0b00000000,

        A=static_cast<bits>(VAR::A),
        B=static_cast<bits>(VAR::B),
        C=static_cast<bits>(VAR::C),
        D=static_cast<bits>(VAR::D)
    };
    enum class HE : bits {
        O=0b00000000,

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
        O=0b00000000,

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
    inline constexpr static std::array<V,4> all_vertices() {
        return {V::A,V::B,V::C,V::D};
    }
    inline constexpr static std::array<HE,12> all_halfedges() {
        return {HE::AB,HE::AC,HE::AB,HE::BA,HE::BC,HE::BD,HE::CA,HE::CB,HE::CD,HE::DA,HE::DB,HE::DC};
    }
    inline constexpr static std::array<HF,4> all_halffaces() {
        return {HF::BDC,HF::CDA,HF::DBA,HF::ABC};
    }
    inline constexpr static uint8_t i(V _v) {
        return b(_v)&3;
    }
    // canonical halfedge index (lexographic)
    inline constexpr static uint8_t i(HE _he) {
        const uint8_t i0 = i(v0(_he));
        const uint8_t i1 = i(v1(_he));
        return (3*i0) + ((i1<i0)? i1 : (i1-1));
    }
    // canonical halfface index (hf[i] = opp_v(v[i]))
    inline constexpr static uint8_t i(HF _hf) {
        return i(opp_v(_hf));
    }
    inline constexpr static HE i2he(int _i) {
        if (_i<0||_i>11) {return HE::O;}
        uint8_t i0 = _i/3;
        uint8_t i1 = (_i%3+1);
        if (i1<=i0) {i1 -= 1;}
        return he(i2v(i0), i2v(i1));
    }
    inline constexpr static V i2v(int _i) {
        return (_i>=0&&_i<4)? v(static_cast<uint8_t>(_i)) : V::O;
    }
    inline constexpr static HF i2hf(int _i) {
        return (_i>=0&&_i<4)? opp_hf(i2v(_i)) : HF::O;
    }
    inline constexpr static bool is_vertex(VAR _var) {
        return (b(_var)&0b11000000)==0b01000000;
    }
    inline constexpr static bool is_halfedge(VAR _var) {
        return (b(_var)&0b11000000)==0b10000000;
    }
    inline constexpr static bool is_halfface(VAR _var) {
        return (b(_var)&0b11000000)==0b11000000 && !is_tet(_var);
    }
    inline constexpr static bool is_tet(VAR _var) {
        return (b(_var)&0b11111100)==0b11111100;
    }
    inline constexpr static bool is_valid(VAR _var) {
        return (b(_var)&0b11000000)!=0b00000000;
    }
    inline constexpr static VAR var(V _v) {
        return static_cast<VAR>(_v);
    }
    inline constexpr static VAR var(HE _he) {
        return static_cast<VAR>(_he);
    }
    inline constexpr static VAR var(HF _hf) {
        return static_cast<VAR>(_hf);
    }
    inline constexpr static V v(const VAR _var) {
        return is_vertex(_var)? static_cast<V>(_var) : V::O;
    }
    inline constexpr static HE he(const VAR _var) {
        return is_halfedge(_var)? static_cast<HE>(_var) : HE::O;
    }
    inline constexpr static HF hf(const VAR _var) {
        return is_halfface(_var)? static_cast<HF>(_var) : HF::O;
    }
    inline constexpr static bool is_same_halfface(HF _hf1, HF _hf2) {
        return opp_v(_hf1)==opp_v(_hf2);
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
    inline constexpr static V opp_v(HF _hf) {
        return fourth_vertex(v(_hf,0), v(_hf,1), v(_hf,2));
    }
    inline constexpr static HF opp_hf(V _v) {
        return is_valid(var(_v))? v_opp_hf_[i(_v)] : HF::O;
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
                    if (idx==2) {return hfs;}
                    break;
                }
            }
        }
        assert(false);
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
    inline constexpr static V v(const bits _b) {return static_cast<V>(0b01000000|(_b&3));}
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

template<typename Vertex, typename HalfEdge, typename HalfFace, typename Cell>
class TetTopologyT
{
private:
    using TT = TetTopology;
public:
    inline constexpr Vertex vertex(TT::V _v) const {
        return TT::is_valid(TT::var(_v))? v_[TT::i(_v)] : Vertex();
    }
    inline constexpr HalfEdge halfedge(TT::HE _he) const {
        return TT::is_valid(TT::var(_he))? he_[TT::i(_he)] : HalfEdge();
    }
    inline constexpr HalfFace halfface(TT::HF _hf) const {
        return TT::is_valid(TT::var(_hf))? hf_[TT::i(_hf)] : HalfFace();
    }
    inline constexpr Vertex operator[](TT::V _v) const {
        return vertex(_v);
    }
    inline constexpr HalfEdge operator[](TT::HE _he) const {
        return halfedge(_he);
    }
    inline constexpr HalfFace operator[](TT::HF _hf) const {
        return halfface(_hf);
    }
    inline constexpr std::optional<std::variant<Vertex,HalfEdge,HalfFace,Cell>> operator[](TT::VAR _var) const {
        if (TT::is_vertex(_var)) {return vertex(TT::v(_var));}
        if (TT::is_halfedge(_var)) {return halfedge(TT::he(_var));}
        if (TT::is_halfface(_var)) {return halfface(TT::hf(_var));}
        if (TT::is_tet(_var)) {return cell();}
        return std::nullopt;
    }
    inline constexpr const std::array<Vertex,4>& vertices() const {
        return v_;
    }
    inline constexpr const std::array<HalfEdge,12>& halfedges() const {
        return he_;
    }
    inline constexpr const std::array<HalfFace,4>& halffaces() const {
        return hf_;
    }
    inline constexpr const Cell& cell() const {
        return c_;
    }
    inline constexpr std::array<Vertex,2> vertices(TT::HE _he) const {
        return {vertex(TT::v0(_he)),vertex(TT::v1(_he))};
    }
    inline constexpr std::array<Vertex,3> vertices(TT::HF _hf) const {
        return {vertex(TT::v0(_hf)),vertex(TT::v1(_hf)),vertex(TT::v2(_hf))};
    }
    inline constexpr std::array<HalfEdge,3> halfedges(TT::HF _hf) const {
        return {halfedge(TT::he0(_hf)),halfedge(TT::he1(_hf)),halfedge(TT::he2(_hf))};
    }
    inline constexpr TT::V v(Vertex _vh) const {
        for (int idx = 0; idx < 4; ++idx) {if (v_[idx] == _vh) {return TT::i2v(idx);}}
        return TT::V::O;
    }
    inline constexpr TT::HE he(HalfEdge _heh) const {
        for (int idx = 0; idx < 12; ++idx) {if (he_[idx] == _heh) {return TT::i2he(idx);}}
        return TT::HE::O;
    }
    inline constexpr TT::HF hf(HalfFace _hfh) const {
        for (int idx = 0; idx < 4; ++idx) {if (hf_[idx] == _hfh) {return TT::i2hf(idx);}}
        return TT::HF::O;
    }
protected:
    std::array<Vertex,4> v_;
    std::array<HalfEdge,12> he_;
    std::array<HalfFace, 4> hf_;
    Cell c_;
};

}
