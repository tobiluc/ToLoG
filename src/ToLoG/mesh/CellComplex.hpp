#pragma once

#include <ToLoG/Traits_fwd.hpp>
#include <ToLoG/vector_concepts.hpp>
#include <cassert>
#include <filesystem>
#include <ranges>
#include <vector>
#include <fstream>
#include <ToLoG/geometry/AABB.hpp>

namespace ToLoG
{

/*
 * This is very much based on OpenMesh and OpenVolumeMesh
 */
class TopologicalCellComplex
{
public:
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
        [[nodiscard]] constexpr bool operator<(int _i) const {return idx_ < _i;}
        [[nodiscard]] constexpr bool operator>(int _i) const {return idx_ > _i;}
        [[nodiscard]] constexpr bool operator<=(int _i) const {return idx_ <= _i;}
        [[nodiscard]] constexpr bool operator>=(int _i) const {return idx_ >= _i;}
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
        constexpr HEH heh(uint8_t _subidx) const {
            return HEH((this->idx()<<1)+_subidx);
        }
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
    class HFH;
    class FH : public BaseHandle
    {
        using BaseHandle::BaseHandle;
    public:
        constexpr HFH hfh(uint8_t _subidx) const {
            return HFH((this->idx()<<1)+_subidx);
        }
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
    class CH : public BaseHandle
    {
        using BaseHandle::BaseHandle;
    };
    template<typename H, typename T>
    class PropT
    {
    public:
        PropT() {}
        PropT(uint32_t _size, const T& _def = T()) : data_(_size, _def) {}
        void push_back(const T& _val) {data_.push_back(_val);}
        void push_back(T&& _val) {data_.push_back(_val);}
        void reserve(uint32_t _size) {data_.reserve(_size);}
        T& operator[](H _h) {return data_[_h.idx()];}
        const T& operator[](H _h) const {return data_.at(_h.idx());}
        constexpr size_t size() const {return data_.size();}
        T* data() {return data_.data();}
        const T* data() const {return data_.data();}
        auto begin() const {return data_.begin();}
        auto end() const {return data_.end();}
    protected:
        std::vector<T> data_;
    };

    template<typename T>
    using VertexPropT = PropT<VH, T>;
    template<typename T>
    using EdgePropT = PropT<EH, T>;
    template<typename T>
    using HalfEdgePropT = PropT<HEH, T>;
    template<typename T>
    using FacePropT = PropT<FH, T>;
    template<typename T>
    using HalfFacePropT = PropT<HFH, T>;
    template<typename T>
    using CellPropT = PropT<CH, T>;

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
            if (curr_idx_ < derived().end_idx()) {
                ++curr_idx_;
            }
            derived().skip_deleted();
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
            curr_idx_ = 0;
            derived().skip_deleted();
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
        VEIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->vertices_[ref()].out_hehs_.size();
        }
    protected:
        void skip_deleted() {
            const size_t n = end_idx();
            if (this->mesh_->is_deleted(this->ref_)) {this->curr_idx_ = n;}
            while (this->curr_idx_ < n) {
                this->curr_ = mesh()->vertices_[ref()].out_hehs_[curr_idx_].eh();
                if (!this->mesh_->is_deleted(this->curr_)) {
                    return;
                }
                ++this->curr_idx_;
            }
            this->curr_.invalidate();
        }
    };

    class VOHEIter : public IterT<VOHEIter, VH, HEH>
    {
        friend class IterT<VOHEIter, VH, HEH>;
    public:
        VOHEIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->vertices_[ref()].out_hehs_.size();
        }
    protected:
        void skip_deleted() {
            const size_t n = end_idx();
            if (this->mesh_->is_deleted(this->ref_)) {this->curr_idx_ = n;}
            while (this->curr_idx_ < n) {
                HEH heh = mesh()->vertices_[ref()].out_hehs_[this->curr_idx_];
                if (!this->mesh_->is_deleted(heh.eh())) {
                    this->curr_ = heh;
                    return;
                }
                ++this->curr_idx_;
            }
            this->curr_.invalidate();
        }
    };

    class VVIter : public IterT<VVIter, VH, VH>
    {
        friend class IterT<VVIter, VH, VH>;
    public:
        VVIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->vertices_[ref()].out_hehs_.size();
        }
    protected:
        void skip_deleted() {
            const size_t n = end_idx();
            if (this->mesh_->is_deleted(this->ref_)) {this->curr_idx_ = n;}
            while (this->curr_idx_ < n) {
                HEH heh = mesh()->vertices_[ref()].out_hehs_[this->curr_idx_];
                if (!this->mesh_->is_deleted(heh.eh())) {
                    curr_ = ref()==mesh()->vh0(heh)?
                        mesh()->vh1(heh) : mesh()->vh0(heh);
                    return;
                }
                ++this->curr_idx_;
            }
            this->curr_.invalidate();
        }
    };

    class VFIter : public IterT<VFIter, VH, FH>
    {
        friend class IterT<VFIter, VH, FH>;
    public:
        VFIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
            fhs_.clear();
            for (EH eh : mesh()->vertex_edges(_vh)) {
                for (FH fh : mesh()->edge_faces(eh)) {
                    if (std::find(fhs_.begin(), fhs_.end(), fh) == fhs_.end()) {
                        fhs_.push_back(fh);
                    }
                }
            }
            skip_deleted();
        }
        int end_idx() const {
            return fhs_.size();
        }
    protected:
        void skip_deleted() {
            if (idx() < end_idx()) {
                curr_ = fhs_[idx()];
                return;
            }
            curr_.invalidate();
        }
        std::vector<FH> fhs_;
    };

    class EVIter : public IterT<EVIter, EH, VH>
    {
        friend class IterT<EVIter, EH, VH>;
    public:
        EVIter(const TopologicalCellComplex* _mesh, EH _eh) : IterT(_mesh, _eh) {
            skip_deleted();
        }
        int end_idx() const {
            return 2;
        }
    protected:
        void skip_deleted() {
            // Note that if the edge is not deleted,
            // vertices can't possibly be deleted!
            if (!mesh()->is_deleted(ref()) && idx() < end_idx()) {
                this->curr_ = mesh()->edges_[ref()].vhs_[this->curr_idx_];
            } else {
                idx() = end_idx();
                curr_.invalidate();
            }
        }
    };

    class EFIter : public IterT<EFIter, EH, FH>
    {
        friend class IterT<EFIter, EH, FH>;
    public:
        EFIter(const TopologicalCellComplex* _mesh, EH _eh) : IterT(_mesh, _eh) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->edges_[ref()].incident_hfhs_.size();
        }
    protected:
        void skip_deleted() {
            const size_t n = end_idx();
            if (mesh_->is_deleted(this->ref_)) {this->curr_idx_ = n;}
            while (this->curr_idx_ < n) {
                this->curr_ = mesh()->edges_[ref()].incident_hfhs_[this->curr_idx_].fh();
                if (!this->mesh_->is_deleted(this->curr_)) {
                    return;
                }
                ++this->curr_idx_;
            }
            this->curr_.invalidate();
        }
    };

    class FVIter : public IterT<FVIter, FH, VH>
    {
        friend class IterT<FVIter, FH, VH>;
    public:
        FVIter(const TopologicalCellComplex* _mesh, FH _fh) : IterT(_mesh, _fh) {
            skip_deleted();
        }
        int end_idx() const {
            return this->mesh_->face_valence(this->ref_);
        }
    protected:
        void skip_deleted() {
            // Note that if the face is not deleted,
            // vertices can't possibly be deleted!
            if (!mesh()->is_deleted(ref()) && idx() < end_idx()) {
                curr_ = mesh()->vh0(mesh()->faces_[ref()].hehs_[idx()]);
            } else {
                idx() = end_idx();
                curr_.invalidate();
            }
        }
    };

    class FHEIter : public IterT<FHEIter, FH, HEH>
    {
        friend class IterT<FHEIter, FH, HEH>;
    public:
        FHEIter(const TopologicalCellComplex* _mesh, FH _fh) : IterT(_mesh, _fh) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->face_valence(ref());
        }
    protected:
        void skip_deleted() {
            // Note that if the face is not deleted,
            // edges can't possibly be deleted!
            if (!mesh()->is_deleted(ref()) && idx() < end_idx()) {
                curr_ = mesh()->faces_[ref()].hehs_[idx()];
            } else {
                idx() = end_idx();
                curr_.invalidate();
            }
        }
    };

    class FEIter : public IterT<FEIter, FH, EH>
    {
        friend class IterT<FEIter, FH, EH>;
    public:
        FEIter(const TopologicalCellComplex* _mesh, FH _fh) : IterT(_mesh, _fh) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->face_valence(ref());
        }
    protected:
        void skip_deleted() {
            // Note that if the face is not deleted,
            // edges can't possibly be deleted!
            if (!mesh()->is_deleted(ref()) && idx() < end_idx()) {
                curr_ = mesh()->faces_[ref()].hehs_[idx()].eh();
            } else {
                idx() = end_idx();
                curr_.invalidate();
            }
        }
    };

    class CHFIter : public IterT<CHFIter, CH, HFH>
    {
        friend class IterT<CHFIter, CH, HFH>;
    public:
        CHFIter(const TopologicalCellComplex* _mesh, CH _ch) : IterT(_mesh, _ch) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->cells_[ref()].hfhs_.size();
        }
    protected:
        void skip_deleted() {
            if (!mesh()->is_deleted(ref()) && idx() < end_idx()) {
                curr_ = mesh()->cells_[ref()].hfhs_[idx()];
            } else {
                idx() = end_idx();
                curr_.invalidate();
            }
        }
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

public:
    TopologicalCellComplex() {
    };

    IterRange<VVIter> vertex_vertices(VH _vh) const {
        VVIter begin(this, _vh);
        VVIter end(this, _vh);
        end.set_end();
        return {begin, end};
    }

    IterRange<VEIter> vertex_edges(VH _vh) const {
        VEIter begin(this, _vh);
        VEIter end(this, _vh);
        end.set_end();
        return {begin, end};
    }

    IterRange<VOHEIter> vertex_out_halfedges(VH _vh) const {
        VOHEIter begin(this, _vh);
        VOHEIter end(this, _vh);
        end.set_end();
        return {begin, end};
    }

    IterRange<VFIter> vertex_faces(VH _vh) const {
        VFIter begin(this, _vh);
        VFIter end(this, _vh);
        end.set_end();
        return {begin, end};
    }

    IterRange<EVIter> edge_vertices(EH _eh) const {
        EVIter begin(this, _eh);
        EVIter end(this, _eh);
        end.set_end();
        return {begin, end};
    }

    IterRange<EFIter> edge_faces(EH _eh) const {
        EFIter begin(this, _eh);
        EFIter end(this, _eh);
        end.set_end();
        return {begin, end};
    }

    IterRange<FVIter> face_vertices(FH _fh) const {
        FVIter begin(this, _fh);
        FVIter end(this, _fh);
        end.set_end();
        return {begin, end};
    }

    IterRange<FEIter> face_edges(FH _fh) const {
        FEIter begin(this, _fh);
        FEIter end(this, _fh);
        end.set_end();
        return {begin, end};
    }

    IterRange<FHEIter> face_halfedges(FH _fh) const {
        FHEIter begin(this, _fh);
        FHEIter end(this, _fh);
        end.set_end();
        return {begin, end};
    }

    IterRange<CHFIter> cell_halffaces(CH _ch) const {
        CHFIter begin(this, _ch);
        CHFIter end(this, _ch);
        end.set_end();
        return {begin, end};
    }

    void reserve_vertices(uint32_t _size);

    void reserve_edges(uint32_t _size);

    void reserve_faces(uint32_t _size);

    void reserve_cells(uint32_t _size);

    constexpr bool is_deleted(VH _vh) const {
        return vertices_[_vh].deleted_;
    }

    constexpr bool is_deleted(EH _eh) const {
        return edges_[_eh].deleted_;
    }

    constexpr bool is_deleted(FH _fh) const {
        return faces_[_fh].deleted_;
    }

    constexpr bool is_deleted(CH _ch) const {
        return cells_[_ch].deleted_;
    }

    auto vertices() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_allocated_vertices());
        auto to_handle = std::views::transform([](uint32_t i) {return VH(i);});
        auto alive = std::views::filter([this](VH vh) {return !is_deleted(vh);});
        return indices | to_handle | alive;
    }

    auto edges() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_allocated_edges());
        auto to_handle = std::views::transform([](uint32_t i) {return EH(i);});
        auto alive = std::views::filter([this](EH eh) {return !is_deleted(eh);});
        return indices | to_handle | alive;
    }

    auto halfedges() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_allocated_halfedges());
        auto to_handle = std::views::transform([](uint32_t i) {return HEH(i);});
        auto alive = std::views::filter([this](HEH heh) {return !is_deleted(heh.eh());});
        return indices | to_handle | alive;
    }

    auto faces() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_allocated_faces());
        auto to_handle = std::views::transform([](uint32_t i) {return FH(i);});
        auto alive = std::views::filter([this](FH fh) {return !is_deleted(fh);});
        return indices | to_handle | alive;
    }

    auto halffaces() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_allocated_halffaces());
        auto to_handle = std::views::transform([](uint32_t i) {return HFH(i);});
        auto alive = std::views::filter([this](HFH hfh) {return !is_deleted(hfh.fh());});
        return indices | to_handle | alive;
    }

    auto cells() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_allocated_cells());
        auto to_handle = std::views::transform([](uint32_t i) {return CH(i);});
        auto alive = std::views::filter([this](CH ch) {return !is_deleted(ch);});
        return indices | to_handle | alive;
    }

    VH add_vertex();

    HEH add_halfedge(VH _vh0, VH _vh1);

    EH add_edge(VH _vh0, VH _vh1);

    HFH add_halfface(const std::vector<HEH>& _hehs);

    HFH add_halfface(const std::vector<VH>& _vhs);

    FH add_face(const std::vector<HEH>& _hehs);

    FH add_face(const std::vector<VH>& _vhs);

    CH add_cell(const std::vector<HFH>& _hfhs);

    void delete_vertex(VH _vh);

    void delete_edge(EH _eh);

    void delete_face(FH _fh);

    void delete_cell(CH _ch);

    HEH find_halfedge(VH _v0, VH _v1) const;

    EH find_edge(VH _v0, VH _v1) const;

    HFH find_halfface(const std::vector<VH>& _vhs) const;

    FH find_face(const std::vector<VH>& _vhs) const;

    uint32_t vertex_num_face_components(VH _v0) const;

    EH faces_shared_edge(FH _fi, FH _fj) const;

    bool vertices_are_adjacent(VH _vh0, VH _vh1) const;

    bool faces_are_adjacent(FH _fi, FH _fj) const;

    bool edge_contains_vertex(EH _eh, VH _vh) const;

    bool face_contains_vertex(FH _fh, VH _vh) const;

    bool face_contains_edge(FH _fh, EH _eh) const;

    CH halfface_incident_cell(HFH _hfh) const;

    constexpr size_t num_allocated_vertices() const {
        return vertices_.size();
    }

    constexpr size_t num_allocated_edges() const {
        return edges_.size();
    }

    constexpr size_t num_allocated_halfedges() const {
        return edges_.size()*2;
    }

    constexpr size_t num_allocated_faces() const {
        return faces_.size();
    }

    constexpr size_t num_allocated_halffaces() const {
        return faces_.size()*2;
    }

    constexpr size_t num_allocated_cells() const {
        return cells_.size();
    }

    constexpr size_t num_deleted_vertices() const {
        return n_deleted_vertices_;
    }

    constexpr size_t num_deleted_edges() const {
        return n_deleted_edges_;
    }

    constexpr size_t num_deleted_halfedges() const {
        return n_deleted_edges_*2;
    }

    constexpr size_t num_deleted_faces() const {
        return n_deleted_faces_;
    }

    constexpr size_t num_deleted_halffaces() const {
        return n_deleted_faces_*2;
    }

    constexpr size_t num_deleted_cells() const {
        return n_deleted_cells_;
    }

    constexpr size_t num_active_vertices() const {
        return num_allocated_vertices() - num_deleted_vertices();
    }

    constexpr size_t num_active_edges() const {
        return num_allocated_edges() - num_deleted_edges();
    }

    constexpr size_t num_active_halfedges() const {
        return num_active_edges()*2;
    }

    constexpr size_t num_active_faces() const {
        return num_allocated_faces() - num_deleted_faces();
    }

    constexpr size_t num_active_halffaces() const {
        return num_active_faces()*2;
    }

    constexpr size_t num_active_cells() const {
        return num_allocated_cells() - num_deleted_cells();
    }

    size_t vertex_valence(VH _vh) const {
        size_t valence(0);
        for (HEH heh : vertex_out_halfedges(_vh)) {++valence;}
        return valence;
    }

    size_t edge_valence(EH _eh) const {
        size_t valence(0);
        for (FH fh : edge_faces(_eh)) {++valence;}
        return valence;
    }

    size_t edge_valence(HEH _heh) const {
        return edge_valence(_heh.eh());
    }

    size_t face_valence(FH _fh) const {
        return faces_[_fh].hehs_.size();
    }

    size_t face_valence(HFH _hfh) const {
        return face_valence(_hfh.fh());
    }

    constexpr bool surface_vertex_is_manifold(VH _vh) const {
        return vertex_num_face_components(_vh) == 1u;
    }

    constexpr bool vertex_is_disconnected(VH _vh) const {
        return vertex_valence(_vh) == 0u;
    }

    constexpr bool surface_edge_is_manifold(EH _eh) const {
        uint32_t n = edge_valence(_eh);
        return n == 2u || n == 1u;
    }

    constexpr bool edge_is_disconnected(EH _eh) const {
        return edge_valence(_eh) == 0u;
    }

    bool surface_mesh_is_manifold() const;

    constexpr VH vh(HEH _heh, uint32_t _subidx) const {
        assert(_subidx < 2);
        return _heh.subidx()==_subidx? edges_[_heh.eh()].vhs_[0] : edges_[_heh.eh()].vhs_[1];
    }

    constexpr VH vh0(HEH _heh) const {
        return vh(_heh, 0);
    }

    constexpr VH vh1(HEH _heh) const {
        return vh(_heh, 1);
    }

    constexpr HEH heh(HFH _hfh, uint32_t _subidx) const {
        FH fh = _hfh.fh();
        uint32_t n = faces_[fh].hehs_.size();
        assert(_idx < n);
        if (is_deleted(fh)) {return HEH();}
        if (_hfh.subidx() == 0) {
            return faces_[fh].hehs_[_subidx];
        } else {
            return faces_[fh].hehs_[n-1-_subidx].opp();
        }
    }

    constexpr VH vh(HFH _hfh, uint32_t _subidx) const {
        return vh0(heh(_hfh, _subidx));
    }

    constexpr bool is_active(VH _vh) const {
        return _vh.is_valid() && _vh < vertices_.size()
               && !is_deleted(_vh);
    }

    constexpr bool is_active(EH _eh) const {
        return _eh.is_valid() && _eh < edges_.size()
        && !is_deleted(_eh);
    }

    constexpr bool is_active(FH _fh) const {
        return _fh.is_valid() && _fh < faces_.size()
        && !is_deleted(_fh);
    }

    constexpr bool is_active(CH _ch) const {
        return _ch.is_valid() && _ch < cells_.size()
        && !is_deleted(_ch);
    }

protected:
    struct Vertex
    {
        std::vector<HEH> out_hehs_ = {};
        bool deleted_ = false;
    };
    struct Edge
    {
        std::array<VH,2> vhs_ = {VH(), VH()};
        std::vector<HFH> incident_hfhs_ = {};
        bool deleted_ = false;
    };
    struct Face
    {
        std::vector<HEH> hehs_ = {};
        std::array<CH,2> chs_ = {CH(), CH()};
        bool deleted_ = false;
    };
    struct Cell
    {
        std::vector<HFH> hfhs_ = {};
        bool deleted_ = false;
    };

    VertexPropT<Vertex> vertices_;
    EdgePropT<Edge> edges_;
    FacePropT<Face> faces_;
    CellPropT<Cell> cells_;
    size_t n_deleted_vertices_ = 0;
    size_t n_deleted_edges_ = 0;
    size_t n_deleted_faces_ = 0;
    size_t n_deleted_cells_ = 0;
};

template<vector Point>
class GeometricCellComplex : public TopologicalCellComplex
{
private:
    using PositionProp = VertexPropT<Point>;
    using FT = Traits<Point>::value_type;
    static constexpr int DIM = Traits<Point>::dim;

public:

    VH add_vertex(const Point& _pos) {
        VH vh = TopologicalCellComplex::add_vertex();
        positions_.push_back(_pos);
        return vh;
    }

    void reserve_vertices(size_t _size) {
        TopologicalCellComplex::reserve_vertices(_size);
        positions_.reserve(_size);
    }

    const Point& point(VH _vh) const {
        return positions_[_vh];
    }

    auto points() const {
        auto to_point = std::views::transform([this](VH vh) {return point(vh);});
        return vertices() | to_point;
    }

    Point edge_barycenter(EH _eh) const {
        return (point(vh0(_eh)) + point(vh1(_eh)))*FT(0.5);
    }

    Point face_barycenter(FH _fh) const
    {
        Point bary = filled<Point>(0);
        FT valence(0);
        for (VH vh : face_vertices(_fh)) {
            bary += point(vh);
            valence += 1;
        }
        return bary / valence;
    }

    void laplacian_smoothing(int _smoothing_iters = 1, bool _skip_nonmanifold = true)
    {
        // Cache Vertex Manifoldness
        VertexPropT<uint8_t> v_manifold(num_allocated_vertices(), false);
        for (VH vh : vertices()) {
            v_manifold[vh] = surface_vertex_is_manifold(vh);
        }

        for (int iter = 0; iter < _smoothing_iters; ++iter) {
            PositionProp next_points(num_allocated_vertices());
            for (VH vh0 : vertices()) {
                FT n(1);
                next_points[vh0] = point(vh0);
                if (_skip_nonmanifold && !v_manifold[vh0]) {continue;}
                for (EH eh : vertex_edges(vh0)) {
                    if (_skip_nonmanifold && !surface_edge_is_manifold(eh)) {continue;}
                    VH vh1 = (edges_[eh][0]==vh0)? edges_[eh][1] : edges_[eh][0];
                    if (_skip_nonmanifold && !v_manifold[vh1]) {continue;}
                    next_points[vh0] += point(vh1);
                    ++n;
                }
                next_points[vh0] /= n;
            }
            positions_ = std::move(next_points);
        }
    }

    enum class TriangulationStrategy {
        BARY, // insert new vertex at barycenter
        FAN
    };

    void triangulate_faces(const TriangulationStrategy _strat = TriangulationStrategy::BARY)
    {
        GeometricCellComplex tri_mesh;
        tri_mesh.reserve_vertices(num_allocated_vertices());
        tri_mesh.reserve_edges(num_allocated_edges());
        tri_mesh.reserve_faces(num_allocated_faces());
        for (const auto& p : positions_) {
            tri_mesh.add_vertex(p);
        }
        if (_strat == TriangulationStrategy::BARY)
        {
            for (FH fh : faces()) {
                VH v0 = tri_mesh.add_vertex(face_barycenter(fh));
                const auto& f = face_vertices(fh);
                for (int i = 0; i < f.size(); ++i) {
                    tri_mesh.add_face({
                        v0,
                        f[i],
                        f[(i+1)%f.size()]
                    });
                }
            }
        }
        else if (_strat == TriangulationStrategy::FAN)
        {
            for (FH fh : faces()) {
                const auto& f = face_vertices(fh);
                for (int i = 1; i < f.size()-1; ++i) {
                    tri_mesh.add_face({
                        f[0],
                        f[i],
                        f[i+1]
                    });
                }
            }
        }
        *this = std::move(tri_mesh);
    }

    void save_obj(const std::filesystem::path& _path) const {
        std::ofstream file(_path);
        for (const auto& p : positions_) {
            file << "v";
            for (int i = 0; i < DIM; ++i) {
                file << " " << p[i];
            }
            file << std::endl;
        }
        for (const auto& eh : edges()) {
            if (!edge_is_disconnected(eh)) {continue;}
            HEH heh = eh.heh(0);
            file << "l " << (vh0(heh).idx()+1) << " "
                 << (vh1(heh).idx()+1) << std::endl;
        }
        for (const auto& fh : faces()) {
            file << "f";
            for (VH vh : face_vertices(fh)) {
                file << " " << (vh.idx()+1);
            }
            file << std::endl;
        }
        file.close();
    }

protected:
    PositionProp positions_;
};

template<typename Point>
using PolyhedralMesh = GeometricCellComplex<Point>;

template<vector Point>
AABB<Point> aabb(const PolyhedralMesh<Point>& _mesh)
{
    AABB<Point> bbox;
    for (const auto& p : _mesh.points()) {
        bbox.expand(p);
    }
    return bbox;
}

// struct PolygonMeshTags
// {
//     TopologicalCellComplex::VertexPropT<int> v_tags;
//     TopologicalCellComplex::EdgePropT<int> e_tags;
//     TopologicalCellComplex::FacePropT<int> f_tags;
// };

}
