#pragma once

#include <iterator>
#include <ToLoG/mesh/cell_complex_handles.hpp>

namespace ToLoG::Mesh
{

class TopologicalCellComplex;
class TopologicalTetrahedralCellComplex;

template<typename Derived, typename H0, typename H1>
class IterT
{
public:
    // STL compatibility
    using iterator_category = std::input_iterator_tag;
    using value_type        = H1;
    using difference_type   = std::ptrdiff_t;
    using pointer           = const H1*;
    using reference         = const H1&;

    IterT(const TopologicalCellComplex* _mesh, H0 _ref) : mesh_(_mesh), ref_(_ref), curr_idx_() {}
    virtual ~IterT() = default;
    reference operator*() const {return curr_;}
    pointer operator->() const {return &curr_;}
    bool operator==(const IterT& other) const {
        return mesh_ == other.mesh_ &&
               ref_ == other.ref_ &&
               curr_idx_ == other.curr_idx_;
    }
    bool operator!=(const IterT& other) const {
        return !(*this == other);
    }
    operator bool() const {return curr_.is_valid();}
    IterT& operator++() {
        if ((++curr_idx_) < derived().end_idx()) {
            derived().update();
        }
        return *this;
    }
    IterT operator++(int) {
        IterT copy = *this;
        ++(*this);
        return copy;
    }
    const TopologicalCellComplex* mesh() const {return mesh_;}
    constexpr const H0& ref() const {return ref_;}
    void set_begin() {
        if ((curr_idx_=0) < derived().end_idx()) {
            derived().update();
        }
    }
    void set_end() {
        curr_idx_ = derived().end_idx();
        curr_.invalidate();
    }
protected:
    const TopologicalCellComplex* mesh_;
    const H0 ref_;
    H1 curr_;
    int curr_idx_ = 0;
    int& idx() {return curr_idx_;}
    Derived& derived() {return static_cast<Derived&>(*this);}
    const Derived& derived() const {return static_cast<const Derived&>(*this);}
};

class VEIter : public IterT<VEIter, VH, EH>
{
    friend class IterT<VEIter, VH, EH>;
public:
    VEIter(const TopologicalCellComplex* _mesh, VH _vh);
    int end_idx() const;
protected:
    void update();
};

class VOHEIter : public IterT<VOHEIter, VH, HEH>
{
    friend class IterT<VOHEIter, VH, HEH>;
public:
    VOHEIter(const TopologicalCellComplex* _mesh, VH _vh);
    int end_idx() const;
protected:
    void update();
};

class VIHEIter : public IterT<VIHEIter, VH, HEH>
{
    friend class IterT<VIHEIter, VH, HEH>;
public:
    VIHEIter(const TopologicalCellComplex* _mesh, VH _vh);
    int end_idx() const;
protected:
    void update();
};

class VVIter : public IterT<VVIter, VH, VH>
{
    friend class IterT<VVIter, VH, VH>;
public:
    VVIter(const TopologicalCellComplex* _mesh, VH _vh);
    int end_idx() const;
protected:
    void update();
};

class VFIter : public IterT<VFIter, VH, FH>
{
    friend class IterT<VFIter, VH, FH>;
public:
    VFIter(const TopologicalCellComplex* _mesh, VH _vh);
    int end_idx() const;
protected:
    void update();
    std::vector<FH> fhs_;
};

class EVIter : public IterT<EVIter, EH, VH>
{
    friend class IterT<EVIter, EH, VH>;
public:
    EVIter(const TopologicalCellComplex* _mesh, EH _eh);
    int end_idx() const;
protected:
    void update();
};

class EFIter : public IterT<EFIter, EH, FH>
{
    friend class IterT<EFIter, EH, FH>;
public:
    EFIter(const TopologicalCellComplex* _mesh, EH _eh);
    int end_idx() const;
protected:
    void update();
};

class FVIter : public IterT<FVIter, FH, VH>
{
    friend class IterT<FVIter, FH, VH>;
public:
    FVIter(const TopologicalCellComplex* _mesh, FH _fh);
    int end_idx() const;
protected:
    void update();
};

class HFVIter : public IterT<HFVIter, HFH, VH>
{
    friend class IterT<HFVIter, HFH, VH>;
public:
    HFVIter(const TopologicalCellComplex* _mesh, HFH _hfh);
    int end_idx() const;
protected:
    void update();
};

class HFHEIter : public IterT<HFHEIter, HFH, HEH>
{
    friend class IterT<HFHEIter, HFH, HEH>;
public:
    HFHEIter(const TopologicalCellComplex* _mesh, HFH _hfh);
    int end_idx() const;
protected:
    void update();
};

class FHEIter : public IterT<FHEIter, FH, HEH>
{
    friend class IterT<FHEIter, FH, HEH>;
public:
    FHEIter(const TopologicalCellComplex* _mesh, FH _fh);
    int end_idx() const;
protected:
    void update();
};

class FEIter : public IterT<FEIter, FH, EH>
{
    friend class IterT<FEIter, FH, EH>;
public:
    FEIter(const TopologicalCellComplex* _mesh, FH _fh);
    int end_idx() const;
protected:
    void update();
};

class CHFIter : public IterT<CHFIter, CH, HFH>
{
    friend class IterT<CHFIter, CH, HFH>;
public:
    CHFIter(const TopologicalCellComplex* _mesh, CH _ch);
    int end_idx() const;
protected:
    void update();
};

class CFIter : public IterT<CFIter, CH, FH>
{
    friend class IterT<CFIter, CH, FH>;
public:
    CFIter(const TopologicalCellComplex* _mesh, CH _ch);
    int end_idx() const;
protected:
    void update();
};

class CCIter : public IterT<CCIter, CH, CH>
{
    friend class IterT<CCIter, CH, CH>;
public:
    CCIter(const TopologicalCellComplex* _mesh, CH _ch);
    int end_idx() const;
protected:
    void update();
};

class CVIter : public IterT<CVIter, CH, VH>
{
    friend class IterT<CVIter, CH, VH>;
public:
    CVIter(const TopologicalCellComplex* _mesh, CH _ch);
    int end_idx() const;
protected:
    void update();
    std::vector<VH> vhs_;
};

class TetCVIter : public IterT<TetCVIter, CH, VH>
{
    friend class IterT<TetCVIter, CH, VH>;
public:
    TetCVIter(const TopologicalTetrahedralCellComplex* _mesh, CH _ch);
    int end_idx() const;
protected:
    void update();
    HFH hfh0;
};

template <typename Iter>
class IterRange {
public:
    IterRange(Iter _begin, Iter _end) : begin_(_begin), end_(_end) {}
    Iter begin() const {return begin_;}
    Iter end() const {return end_;}
private:
    Iter begin_;
    Iter end_;
};

}
