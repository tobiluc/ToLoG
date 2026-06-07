#pragma once

#include <ToLoG/Traits_fwd.hpp>
#include <ToLoG/vector_concepts.hpp>
#include <cassert>
#include <ranges>
#include <vector>
#include <ToLoG/geometry/AABB.hpp>
#include <queue>
#include <ToLoG/HashMap.hpp>
#include <ToLoG/mesh/cell_complex_handles.hpp>
#include <ToLoG/mesh/cell_complex_iterators.hpp>

namespace ToLoG::Mesh
{

class TopologicalCellComplex;

template<typename H, typename T>
class PropT
{
public:
    typedef H Handle;

    PropT()
    {}

    PropT(uint32_t _size, const T& _def = T()) :
        data_(_size, _def)
    {}

    constexpr size_t size() {
        return data_.size();
    }

    T& operator[](H _h) {
        return data_[_h.idx()];
    }

    const T& operator[](H _h) const {
        return data_.at(_h.idx());
    }

    constexpr size_t size() const {
        return data_.size();
    }

    T* data() {
        return data_.data();
    }

    const T* data() const {
        return data_.data();
    }

    auto begin() const {
        return data_.begin();
    }

    auto end() const {
        return data_.end();
    }

    void push_back(const T& _val = T()) {
        data_.push_back(_val);
    }

    void push_back(T&& _val) {
        data_.push_back(_val);
    }

    void clear() {
        data_.clear();
    }

    void reserve(size_t _size) {
        data_.reserve(_size);
    }

    void resize(size_t _size) {
        data_.resize(_size);
    }

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

/*
 * This is very much based on OpenMesh and OpenVolumeMesh
 */
class TopologicalCellComplex
{
    friend class VVIter;
    friend class VEIter;
    friend class VIHEIter;
    friend class VOHEIter;
    friend class VFIter;
    friend class VHFIter;
    friend class VCIter;
    friend class EVIter;
    friend class EFIter;
    friend class ECIter;
    friend class FVIter;
    friend class FEIter;
    friend class FHEIter;
    friend class FHVIter;
    friend class FCIter;
    friend class HFVIter;
    friend class HFHEIter;
    friend class HFEIter;
    friend class CHFIter;
    friend class CFIter;
    friend class CVIter;
    friend class CCIter;

public:
    TopologicalCellComplex()
    {};

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

    IterRange<VIHEIter> vertex_in_halfedges(VH _vh) const {
        VIHEIter begin(this, _vh);
        VIHEIter end(this, _vh);
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

    IterRange<HFVIter> halfface_vertices(HFH _hfh) const {
        HFVIter begin(this, _hfh);
        HFVIter end(this, _hfh);
        end.set_end();
        return {begin, end};
    }

    IterRange<HFHEIter> halfface_halfedges(HFH _hfh) const {
        HFHEIter begin(this, _hfh);
        HFHEIter end(this, _hfh);
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

    IterRange<CFIter> cell_faces(CH _ch) const {
        CFIter begin(this, _ch);
        CFIter end(this, _ch);
        end.set_end();
        return {begin, end};
    }

    IterRange<CVIter> cell_vertices(CH _ch) const {
        CVIter begin(this, _ch);
        CVIter end(this, _ch);
        end.set_end();
        return {begin, end};
    }

    IterRange<CCIter> cell_cells(CH _ch) const {
        CCIter begin(this, _ch);
        CCIter end(this, _ch);
        end.set_end();
        return {begin, end};
    }

    void reserve_vertices(uint32_t _size) {
        vertices_.reserve(_size);
        vertex_deleted_.reserve(_size);
    }

    void reserve_edges(uint32_t _size) {
        edges_.reserve(_size);
        edge_deleted_.reserve(_size);
    }

    void reserve_faces(uint32_t _size) {
        faces_.reserve(_size);
        face_deleted_.reserve(_size);
    }

    void reserve_cells(uint32_t _size) {
        cells_.reserve(_size);
        cell_deleted_.reserve(_size);
    }

    constexpr bool is_deleted(VH _vh) const {
        return vertex_deleted_[_vh];
    }

    constexpr bool is_deleted(EH _eh) const {
        return edge_deleted_[_eh];
    }

    constexpr bool is_deleted(FH _fh) const {
        return face_deleted_[_fh];
    }

    constexpr bool is_deleted(CH _ch) const {
        return cell_deleted_[_ch];
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

    VH shared_vertex(EH _eh1, EH _eh2) const;

    EH shared_edge(FH _fh1, FH _fh2) const;

    FH shared_face(CH _ch1, CH _ch2) const;

    bool are_adjacent(VH _vh1, VH _vh2) const;

    bool are_adjacent(EH _eh1, EH _eh2) const;

    bool are_adjacent(FH _fh1, FH _fh2) const;

    bool are_adjacent(CH _ch1, CH _ch2) const;

    bool are_incident(VH _vh, EH _eh) const;

    bool are_incident(VH _vh, FH _fh) const;

    bool are_incident(VH _vh, CH _ch) const;

    bool are_incident(EH _eh, FH _fh) const;

    bool are_incident(EH _eh, CH _ch) const;

    bool are_incident(FH _fh, CH _ch) const;

    VH opposite_vertex(VH _vh, EH _eh) const {
        return edges_[_eh].vhs_[(edges_[_eh].vhs_[0]==_vh)];
    }

    constexpr CH incident_cell(HFH _hfh) const {
        return faces_[_hfh.fh()].chs_[_hfh.subidx()];
    }

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

    template<typename H>
    constexpr size_t num_allocated() const;

    template<>
    constexpr size_t num_allocated<VH>() const {
        return num_allocated_vertices();
    }

    template<>
    constexpr size_t num_allocated<EH>() const {
        return num_allocated_edges();
    }

    template<>
    constexpr size_t num_allocated<HEH>() const {
        return num_allocated_halfedges();
    }

    template<>
    constexpr size_t num_allocated<FH>() const {
        return num_allocated_faces();
    }

    template<>
    constexpr size_t num_allocated<HFH>() const {
        return num_allocated_halffaces();
    }

    template<>
    constexpr size_t num_allocated<CH>() const {
        return num_allocated_cells();
    }

    constexpr size_t num_deleted_vertices() const {
        return deleted_vertices_.size();
    }

    constexpr size_t num_deleted_edges() const {
        return deleted_edges_.size();
    }

    constexpr size_t num_deleted_halfedges() const {
        return num_deleted_edges()*2;
    }

    constexpr size_t num_deleted_faces() const {
        return deleted_faces_.size();
    }

    constexpr size_t num_deleted_halffaces() const {
        return num_deleted_faces()*2;
    }

    constexpr size_t num_deleted_cells() const {
        return deleted_cells_.size();
    }

    constexpr bool has_deleted() const {
        return num_deleted_vertices()
                || num_deleted_edges()
                || num_deleted_faces()
               || num_deleted_cells();
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

    constexpr size_t vertex_valence(VH _vh) const {
        return vertices_[_vh].out_hehs_.size();
    }

    constexpr size_t edge_valence(EH _eh) const {
        return edges_[_eh].incident_hfhs_.size();
    }

    constexpr size_t edge_valence(HEH _heh) const {
        return edge_valence(_heh.eh());
    }

    constexpr size_t face_valence(FH _fh) const {
        return faces_[_fh].hehs_.size();
    }

    constexpr size_t face_valence(HFH _hfh) const {
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
        return _vh.is_valid() && _vh.idx() < vertices_.size()
               && !is_deleted(_vh);
    }

    constexpr bool is_active(EH _eh) const {
        return _eh.is_valid() && _eh.idx() < edges_.size()
        && !is_deleted(_eh);
    }

    constexpr bool is_active(FH _fh) const {
        return _fh.is_valid() && _fh.idx() < faces_.size()
        && !is_deleted(_fh);
    }

    constexpr bool is_active(CH _ch) const {
        return _ch.is_valid() && _ch.idx() < cells_.size()
        && !is_deleted(_ch);
    }

protected:
    struct Vertex
    {
        std::vector<HEH> out_hehs_ = {};
    };
    struct Edge
    {
        std::array<VH,2> vhs_ = {VH(), VH()};
        std::vector<HFH> incident_hfhs_ = {};
    };
    struct Face
    {
        std::vector<HEH> hehs_ = {};
        std::array<CH,2> chs_ = {CH(), CH()};
    };
    struct Cell
    {
        std::vector<HFH> hfhs_ = {};
    };

    VertexPropT<Vertex> vertices_;
    EdgePropT<Edge> edges_;
    FacePropT<Face> faces_;
    CellPropT<Cell> cells_;
    VertexPropT<uint8_t> vertex_deleted_;
    EdgePropT<uint8_t> edge_deleted_;
    FacePropT<uint8_t> face_deleted_;
    CellPropT<uint8_t> cell_deleted_;
    std::queue<VH> deleted_vertices_;
    std::queue<EH> deleted_edges_;
    std::queue<FH> deleted_faces_;
    std::queue<CH> deleted_cells_;

    void collect_garbage();
};

class TopologicalTetrahedralCellComplex : public TopologicalCellComplex
{
    using TopologicalCellComplex::cell_vertices;

public:
    TopologicalTetrahedralCellComplex() :
        TopologicalCellComplex()
    {};

    IterRange<TetCVIter> cell_vertices(CH _ch) const {
        TetCVIter begin(this, _ch);
        TetCVIter end(this, _ch);
        end.set_end();
        return {begin, end};
    }

    VH opposite_vertex(HFH _hfh) const;

};

template<vector Point, typename TopoKernel = TopologicalCellComplex>
class GeometricCellComplex : public TopoKernel
{
private:
    using TopoKernel::TopoKernel;
    using PositionProp = VertexPropT<Point>;
    using FT = Traits<Point>::value_type;
    static constexpr int DIM = Traits<Point>::dim;

public:

    VH add_vertex(const Point& _pos) {
        VH vh = TopoKernel::add_vertex();
        positions_.resize(TopoKernel::num_allocated_vertices());
        positions_[vh] = _pos;
        return vh;
    }

    void reserve_vertices(size_t _size) {
        TopoKernel::reserve_vertices(_size);
        positions_.reserve(_size);
    }

    const Point& point(VH _vh) const {
        return positions_[_vh];
    }

    auto points() const {
        auto to_point = std::views::transform([this](VH vh) {return point(vh);});
        return TopoKernel::vertices() | to_point;
    }

    void set_point(VH _vh, const Point& _pos) {
        positions_[_vh] = _pos;
    }

    void set_points(const PositionProp& _points) {
        for (VH vh : TopoKernel::vertices()) {
            positions_[vh] = _points[vh];
        }
    }

    Point barycenter(EH _eh) const {
        return (point(TopoKernel::vh0(_eh)) + point(TopoKernel::vh1(_eh)))*FT(0.5);
    }

    Point barycenter(FH _fh) const
    {
        Point bary = filled<Point>(0);
        FT valence(0);
        for (VH vh : TopoKernel::face_vertices(_fh)) {
            bary += point(vh);
            valence += 1;
        }
        return bary / valence;
    }

    Point barycenter(CH _ch) const
    {
        Point bary = filled<Point>(0);
        FT valence(0);
        for (VH vh : TopoKernel::cell_vertices(_ch)) {
            bary += point(vh);
            valence += 1;
        }
        return bary / valence;
    }

protected:
    PositionProp positions_;
};

template<typename Point>
using PolyhedralMesh = GeometricCellComplex<Point, TopologicalCellComplex>;

template<typename Point>
using TetrahedralMesh = GeometricCellComplex<Point, TopologicalTetrahedralCellComplex>;

template<vector Point, typename Topology>
AABB<Point> aabb(const GeometricCellComplex<Point, Topology>& _mesh)
{
    AABB<Point> bbox;
    for (const auto& p : _mesh.points()) {
        bbox.expand(p);
    }
    return bbox;
}

}
