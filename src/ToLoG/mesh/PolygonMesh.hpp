#pragma once

#include <ToLoG/Traits_fwd.hpp>
#include <ToLoG/vector_concepts.hpp>
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
class PolygonMeshTopologyKernel
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
    class FH : public BaseHandle
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

        IterT(const PolygonMeshTopologyKernel* _mesh, H0 _ref) : mesh_(_mesh), ref_(_ref), curr_idx_() {}
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
        const PolygonMeshTopologyKernel* mesh() const {return mesh_;}
        constexpr H0 ref() const {return ref_;}
        void set_begin() {
            curr_idx_ = 0;
            derived().skip_deleted();
        }
        void set_end() {
            curr_idx_ = derived().end_idx();
            curr_.invalidate();
        }
    protected:
        const PolygonMeshTopologyKernel* mesh_;
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
        VEIter(const PolygonMeshTopologyKernel* _mesh, VH _vh) : IterT(_mesh, _vh) {
            skip_deleted();
        }
        int end_idx() const {
            return this->mesh_->vertex_out_halfedges_[this->ref_].size();
        }
    protected:
        void skip_deleted() {
            const size_t n = end_idx();
            if (this->mesh_->vertex_is_deleted(this->ref_)) {this->curr_idx_ = n;}
            while (this->curr_idx_ < n) {
                this->curr_ = this->mesh_->vertex_out_halfedges_[this->ref_][this->curr_idx_].eh();
                if (!this->mesh_->edge_is_deleted(this->curr_)) {
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
        VOHEIter(const PolygonMeshTopologyKernel* _mesh, VH _vh) : IterT(_mesh, _vh) {
            skip_deleted();
        }
        int end_idx() const {
            return this->mesh_->vertex_out_halfedges_[this->ref_].size();
        }
    protected:
        void skip_deleted() {
            const size_t n = end_idx();
            if (this->mesh_->vertex_is_deleted(this->ref_)) {this->curr_idx_ = n;}
            while (this->curr_idx_ < n) {
                HEH heh = this->mesh_->vertex_out_halfedges_[this->ref_][this->curr_idx_];
                if (!this->mesh_->halfedge_is_deleted(heh)) {
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
        VVIter(const PolygonMeshTopologyKernel* _mesh, VH _vh) : IterT(_mesh, _vh) {
            skip_deleted();
        }
        int end_idx() const {
            return this->mesh_->vertex_out_halfedges_[this->ref_].size();
        }
    protected:
        void skip_deleted() {
            const size_t n = end_idx();
            if (this->mesh_->vertex_is_deleted(this->ref_)) {this->curr_idx_ = n;}
            while (this->curr_idx_ < n) {
                HEH heh = this->mesh_->vertex_out_halfedges_[this->ref_][this->curr_idx_];
                if (!this->mesh_->halfedge_is_deleted(heh)) {
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
        VFIter(const PolygonMeshTopologyKernel* _mesh, VH _vh) : IterT(_mesh, _vh) {
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
        EVIter(const PolygonMeshTopologyKernel* _mesh, EH _eh) : IterT(_mesh, _eh) {
            skip_deleted();
        }
        int end_idx() const {
            return 2;
        }
    protected:
        void skip_deleted() {
            // Note that if the edge is not deleted,
            // vertices can't possibly be deleted!
            if (!mesh()->edge_is_deleted(ref()) && idx() < end_idx()) {
                this->curr_ = this->mesh_->edges_[this->ref_][this->curr_idx_];
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
        EFIter(const PolygonMeshTopologyKernel* _mesh, EH _eh) : IterT(_mesh, _eh) {
            skip_deleted();
        }
        int end_idx() const {
            return this->mesh_->edge_faces_[this->ref_].size();
        }
    protected:
        void skip_deleted() {
            const size_t n = end_idx();
            if (mesh_->edge_is_deleted(this->ref_)) {this->curr_idx_ = n;}
            while (this->curr_idx_ < n) {
                this->curr_ = this->mesh_->edge_faces_[this->ref_][this->curr_idx_];
                if (!this->mesh_->face_is_deleted(this->curr_)) {
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
        FVIter(const PolygonMeshTopologyKernel* _mesh, FH _fh) : IterT(_mesh, _fh) {
            skip_deleted();
        }
        int end_idx() const {
            return this->mesh_->face_valence(this->ref_);
        }
    protected:
        void skip_deleted() {
            // Note that if the face is not deleted,
            // vertices can't possibly be deleted!
            if (!mesh()->face_is_deleted(ref()) && idx() < end_idx()) {
                curr_ = mesh()->vh0(mesh()->faces_[ref()][idx()]);
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
        FHEIter(const PolygonMeshTopologyKernel* _mesh, FH _fh) : IterT(_mesh, _fh) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->face_valence(ref());
        }
    protected:
        void skip_deleted() {
            // Note that if the face is not deleted,
            // edges can't possibly be deleted!
            if (!mesh()->face_is_deleted(ref()) && idx() < end_idx()) {
                curr_ = mesh()->faces_[ref()][idx()];
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
        FEIter(const PolygonMeshTopologyKernel* _mesh, FH _fh) : IterT(_mesh, _fh) {
            skip_deleted();
        }
        int end_idx() const {
            return mesh()->face_valence(ref());
        }
    protected:
        void skip_deleted() {
            // Note that if the face is not deleted,
            // edges can't possibly be deleted!
            if (!mesh()->face_is_deleted(ref()) && idx() < end_idx()) {
                curr_ = mesh()->faces_[ref()][idx()].eh();
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
    PolygonMeshTopologyKernel() {
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

    void reserve_vertices(uint32_t _size);

    void reserve_edges(uint32_t _size);

    void reserve_faces(uint32_t _size);

    constexpr bool vertex_is_deleted(VH _vh) const {
        return vertex_deleted_[_vh];
    }

    constexpr bool edge_is_deleted(EH _eh) const {
        return edge_deleted_[_eh];
    }

    constexpr bool halfedge_is_deleted(HEH _heh) const {
        return edge_is_deleted(_heh.eh());
    }

    constexpr bool face_is_deleted(FH _fh) const {
        return face_deleted_[_fh];
    }

    auto vertices() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_vertices());
        auto to_handle = std::views::transform([](uint32_t i) {return VH(i);});
        auto alive = std::views::filter([this](VH vh) {return !vertex_is_deleted(vh);});
        return indices | to_handle | alive;
    }

    auto edges() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_edges());
        auto to_handle = std::views::transform([](uint32_t i) {return EH(i);});
        auto alive = std::views::filter([this](EH eh) {return !edge_is_deleted(eh);});
        return indices | to_handle | alive;
    }

    auto halfedges() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_halfedges());
        auto to_handle = std::views::transform([](uint32_t i) {return HEH(i);});
        auto alive = std::views::filter([this](HEH heh) {return !halfedge_is_deleted(heh);});
        return indices | to_handle | alive;
    }

    auto faces() const
    {
        auto indices = std::views::iota(uint32_t{0}, num_faces());
        auto to_handle = std::views::transform([](uint32_t i) {return FH(i);});
        auto alive = std::views::filter([this](FH fh) {return !face_is_deleted(fh);});
        return indices | to_handle | alive;
    }

    VH add_vertex();

    HEH add_halfedge(VH _vh0, VH _vh1);

    EH add_edge(VH _vh0, VH _vh1);

    FH add_face(const std::vector<HEH>& _hehs);

    FH add_face(const std::vector<VH>& _vhs);

    void delete_vertex(VH _vh);

    void delete_edge(EH _eh);

    void delete_face(FH _fh);

    HEH find_halfedge(VH _v0, VH _v1) const;

    EH find_edge(VH _v0, VH _v1) const;

    FH find_face(const std::vector<VH>& _vhs) const;

    uint32_t vertex_num_face_components(VH _v0) const;

    EH faces_shared_edge(FH _fi, FH _fj) const;

    bool vertices_are_adjacent(VH _vh0, VH _vh1) const;

    bool faces_are_adjacent(FH _fi, FH _fj) const;

    bool edge_contains_vertex(EH _eh, VH _vh) const;

    bool face_contains_vertex(FH _fh, VH _vh) const;

    bool face_contains_edge(FH _fh, EH _eh) const;

    constexpr size_t num_vertices() const {
        return n_vertices_;
    }

    constexpr size_t num_edges() const {
        return edges_.size();
    }

    constexpr size_t num_halfedges() const {
        return edges_.size()*2;
    }

    constexpr size_t num_faces() const {
        return faces_.size();
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

    size_t face_valence(FH _fh) const {
        return faces_[_fh].size();
    }

    constexpr bool vertex_is_manifold(VH _vh) const {
        return vertex_num_face_components(_vh) == 1u;
    }

    constexpr bool vertex_is_disconnected(VH _vh) const {
        return vertex_valence(_vh) == 0u;
    }

    constexpr bool edge_is_manifold(EH _eh) const {
        uint32_t n = edge_valence(_eh);
        return n == 2u || n == 1u;
    }

    constexpr bool edge_is_disconnected(EH _eh) const {
        return edge_valence(_eh) == 0u;
    }

    bool mesh_is_manifold() const;

    constexpr VH vh0(HEH _heh) const {
        return _heh.subidx()? edges_[_heh.eh()][1] : edges_[_heh.eh()][0];
    }

    constexpr VH vh1(HEH _heh) const {
        return _heh.subidx()? edges_[_heh.eh()][0] : edges_[_heh.eh()][1];
    }

protected:
    size_t n_vertices_ = 0;
    EdgePropT<std::array<VH,2>> edges_;
    FacePropT<std::vector<HEH>> faces_;
    VertexPropT<std::vector<HEH>> vertex_out_halfedges_;
    EdgePropT<std::vector<FH>> edge_faces_;
    VertexPropT<uint8_t> vertex_deleted_;
    EdgePropT<uint8_t> edge_deleted_;
    FacePropT<uint8_t> face_deleted_;
};

template<vector Point>
class PolygonMeshGeometryKernel : public PolygonMeshTopologyKernel
{
private:
    using PositionProp = VertexPropT<Point>;
    using FT = Traits<Point>::value_type;
    static constexpr int DIM = Traits<Point>::dim;

public:

    VH add_vertex(const Point& _pos) {
        VH vh = PolygonMeshTopologyKernel::add_vertex();
        positions_.push_back(_pos);
        return vh;
    }

    void reserve_vertices(size_t _size) {
        PolygonMeshTopologyKernel::reserve_vertices(_size);
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
        VertexPropT<uint8_t> v_manifold(num_vertices(), false);
        for (VH vh : vertices()) {
            v_manifold[vh] = vertex_is_manifold(vh);
        }

        for (int iter = 0; iter < _smoothing_iters; ++iter) {
            PositionProp next_points(num_vertices());
            for (VH vh0 : vertices()) {
                FT n(1);
                next_points[vh0] = point(vh0);
                if (_skip_nonmanifold && !v_manifold[vh0]) {continue;}
                for (EH eh : vertex_edges(vh0)) {
                    if (_skip_nonmanifold && !edge_is_manifold(eh)) {continue;}
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
        PolygonMeshGeometryKernel tri_mesh;
        tri_mesh.reserve_vertices(num_vertices());
        tri_mesh.reserve_edges(num_edges());
        tri_mesh.reserve_faces(num_faces());
        for (const auto& p : points()) {
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
using PolygonMesh = PolygonMeshGeometryKernel<Point>;

template<vector Point>
AABB<Point> aabb(const PolygonMesh<Point>& _mesh)
{
    AABB<Point> bbox;
    for (const auto& p : _mesh.points()) {
        bbox.expand(p);
    }
    return bbox;
}

struct PolygonMeshTags
{
    PolygonMeshTopologyKernel::VertexPropT<int> v_tags;
    PolygonMeshTopologyKernel::EdgePropT<int> e_tags;
    PolygonMeshTopologyKernel::FacePropT<int> f_tags;
};

}
