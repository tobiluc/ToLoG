#pragma once

#include <cstdint>
#include <ostream>

namespace ToLoG::Mesh
{

class BaseHandle
{
public:
    static constexpr uint32_t invalid = UINT32_MAX;
    constexpr BaseHandle() = default;
    explicit constexpr BaseHandle(uint32_t _idx) : idx_(_idx) {}
    [[nodiscard]] constexpr bool is_valid() const {return idx_ != invalid;}
    [[nodiscard]] constexpr uint32_t idx() const {return idx_;}
    void invalidate() {idx_ = invalid;}
    [[nodiscard]] constexpr bool operator==(const BaseHandle& _h) const {return idx_ == _h.idx_;}
    [[nodiscard]] constexpr bool operator<(const BaseHandle& _h) const {return idx_ < _h.idx_;}
    [[nodiscard]] constexpr bool operator>(const BaseHandle& _h) const {return idx_ > _h.idx_;}
    [[nodiscard]] constexpr bool operator<=(const BaseHandle& _h) const {return idx_ <= _h.idx_;}
    [[nodiscard]] constexpr bool operator>=(const BaseHandle& _h) const {return idx_ >= _h.idx_;}
    friend std::ostream& operator<<(std::ostream& _os, const BaseHandle& _h) {
        return _os << _h.idx();
    }
private:
    uint32_t idx_ = invalid;
};

class VH : public BaseHandle
{
    using BaseHandle::BaseHandle;
};
class HEH;

class EH : public BaseHandle
{
    using BaseHandle::BaseHandle;
public:
    constexpr HEH heh(uint8_t _subidx) const;
};

class HEH : public BaseHandle
{
    using BaseHandle::BaseHandle;
public:
    constexpr int subidx() const {
        return this->idx() & 1;
    }
    constexpr EH eh() const {
        return EH(this->idx()>>1);
    }
    constexpr HEH opp() const {
        return HEH(this->idx()^1);
    }
};

constexpr HEH EH::heh(uint8_t _subidx) const {
    return HEH((this->idx()<<1)+_subidx);
}

class HFH;

class FH : public BaseHandle
{
    using BaseHandle::BaseHandle;
public:
    constexpr HFH hfh(uint8_t _subidx) const;
};

class HFH : public BaseHandle
{
    using BaseHandle::BaseHandle;
public:
    constexpr int subidx() const {
        return this->idx() & 1;
    }
    constexpr FH fh() const {
        return FH(this->idx()>>1);
    }
    constexpr HFH opp() const {
        return HFH(this->idx()^1);
    }
};

constexpr HFH FH::hfh(uint8_t _subidx) const {
    return HFH((this->idx()<<1)+_subidx);
}

class CH : public BaseHandle
{
    using BaseHandle::BaseHandle;
};

}
