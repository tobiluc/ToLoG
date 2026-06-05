#include <ToLoG/mesh/CellComplex.hpp>
#include <queue>

namespace ToLoG
{

using VH = TopologicalCellComplex::VH;
using EH = TopologicalCellComplex::EH;
using HEH = TopologicalCellComplex::HEH;
using FH = TopologicalCellComplex::FH;
using HFH = TopologicalCellComplex::HFH;
using CH = TopologicalCellComplex::CH;

void TopologicalCellComplex::reserve_vertices(uint32_t _size)
{
    vertices_.reserve(_size);
}

void TopologicalCellComplex::reserve_edges(uint32_t _size)
{
    edges_.reserve(_size);
}

void TopologicalCellComplex::reserve_faces(uint32_t _size)
{
    faces_.reserve(_size);
}

void TopologicalCellComplex::reserve_cells(uint32_t _size)
{
    cells_.reserve(_size);
}

VH TopologicalCellComplex::add_vertex()
{
    VH vh(num_allocated_vertices());
    Vertex v;
    vertices_.push_back(std::move(v));
    return vh;
}

HEH TopologicalCellComplex::add_halfedge(VH _vh0, VH _vh1)
{
    HEH heh = find_halfedge(_vh0, _vh1);
    if (heh.is_valid()) {return heh;}
    if (!is_active(_vh0) || !is_active(_vh1)) {
        throw std::runtime_error("vertex for edge does not exist!");
    }
    heh = HEH(num_allocated_halfedges());
    Edge e;
    e.vhs_ = {_vh0, _vh1};
    edges_.push_back(std::move(e));
    vertices_[_vh0].out_hehs_.push_back(heh);
    vertices_[_vh1].out_hehs_.push_back(heh.opp());
    return heh;
}

EH TopologicalCellComplex::add_edge(VH _vh0, VH _vh1)
{
    return add_halfedge(_vh0, _vh1).eh();
}

HFH TopologicalCellComplex::add_halfface(const std::vector<HEH>& _hehs)
{
    if (_hehs.size() < 3) {
        throw std::runtime_error("face needs to have at least valence 3");
    }
    for (int i = 0; i < _hehs.size(); ++i) {
        if (!is_active(_hehs[i].eh())) {
            throw std::runtime_error("edge for face does not exist");
        }
        if (vh1(_hehs[i]) != vh0(_hehs[(i+1)%_hehs.size()])) {
            throw std::runtime_error("face needs to consist of consecutive halfedges");
        }
    }

    FH fh(num_allocated_faces());
    HFH hfh = fh.hfh(0);
    for (HEH heh : _hehs) {
        if (heh.subidx() == 0) {
            edges_[heh.eh()].incident_hfhs_.push_back(hfh);
        } else {
            edges_[heh.eh()].incident_hfhs_.push_back(hfh.opp());
        }
    }
    Face f;
    f.hehs_ = _hehs;
    faces_.push_back(std::move(f));
    return hfh;
}

HFH TopologicalCellComplex::add_halfface(const std::vector<VH>& _vhs)
{
    std::vector<HEH> hehs(_vhs.size());
    for (int i = 0; i < _vhs.size(); ++i) {
        hehs[i] = add_halfedge(_vhs[i], _vhs[(i+1)%_vhs.size()]);
    }
    return add_halfface(hehs);
}

FH TopologicalCellComplex::add_face(const std::vector<HEH>& _hehs)
{
    return add_halfface(_hehs).fh();
}

FH TopologicalCellComplex::add_face(const std::vector<VH>& _vhs)
{
    return add_halfface(_vhs).fh();
}

CH TopologicalCellComplex::add_cell(const std::vector<HFH>& _hfhs)
{
    CH ch(num_allocated_cells());
    for (HFH hfh : _hfhs) {
        if (halfface_incident_cell(hfh).is_valid()) {
            throw std::runtime_error("halfface already has an incident cell");
        }
        faces_[hfh.fh()].chs_[hfh.subidx()] = ch;
    }
    Cell c;
    c.hfhs_ = _hfhs;
    cells_.push_back(std::move(c));
    return ch;
}

void TopologicalCellComplex::delete_vertex(VH _vh)
{
    if (!is_active(_vh)) {return;}

    // Delete incident edges
    for (EH eh : vertex_edges(_vh)) {
        delete_edge(eh);
    }
    vertices_[_vh].deleted_ = true;
    vertices_[_vh].out_hehs_.clear();
    ++n_deleted_vertices_;
}

void TopologicalCellComplex::delete_edge(EH _eh)
{
    if (!is_active(_eh)) {return;}

    // Delete incident faces
    for (FH fh : edge_faces(_eh)) {
        delete_face(fh);
    }

    // Remove references in incident vertices
    for (uint8_t subidx : {0,1}) {
        HEH heh = _eh.heh(subidx);
        vertices_[vh0(heh)].out_hehs_.erase(
            std::remove(vertices_[vh0(heh)].out_hehs_.begin(),
                        vertices_[vh0(heh)].out_hehs_.end(),
                        heh), vertices_[vh0(heh)].out_hehs_.end());
    }

    edges_[_eh].deleted_ = true;
    edges_[_eh].incident_hfhs_.clear();
    ++n_deleted_edges_;
}

void TopologicalCellComplex::delete_face(FH _fh)
{
    if (!is_active(_fh)) {return;}

    // Delete incident cells
    for (CH ch : faces_[_fh].chs_) {
        if (ch.is_valid()) {
            delete_cell(ch);
        }
    }

    // Remove references in incident edges
    for (EH eh : face_edges(_fh)) {
        for (HFH hfh : {_fh.hfh(0), _fh.hfh(1)}) {
            edges_[eh].incident_hfhs_.erase(
                std::remove(edges_[eh].incident_hfhs_.begin(),
                            edges_[eh].incident_hfhs_.end(),
                            hfh), edges_[eh].incident_hfhs_.end());
        }
    }

    faces_[_fh].deleted_ = true;
    faces_[_fh].hehs_.clear();
    ++n_deleted_faces_;
}

void TopologicalCellComplex::delete_cell(CH _ch)
{
    if (!is_active(_ch)) {return;}

    // Remove references in incident halffaces
    for (HFH hfh : cells_[_ch].hfhs_) {
        for (int i = 0; i <= 1; ++i) {
            if (faces_[hfh.fh()].chs_[i] == _ch) {
                faces_[hfh.fh()].chs_[i].invalidate();
            }
        }
    }

    cells_[_ch].deleted_ = true;
    cells_[_ch].hfhs_.clear();
    ++n_deleted_cells_;
}

HEH TopologicalCellComplex::find_halfedge(VH _vh0, VH _vh1) const
{
    for (HEH heh : vertex_out_halfedges(_vh0)) {
        if (vh1(heh) == _vh1) {
            return heh;
        }
    }
    return HEH();
}

EH TopologicalCellComplex::find_edge(VH _vh0, VH _vh1) const
{
    HEH heh = find_halfedge(_vh0, _vh1);
    return heh.is_valid()? heh.eh() : EH();
}

HFH TopologicalCellComplex::find_halfface(const std::vector<VH>& _vhs) const
{
    uint32_t n = _vhs.size();
    if (n<=2) {return HFH();}
    for (FH fh : vertex_faces(_vhs.front())) {
        if (n != face_valence(fh)) {continue;}
        for (HFH hfh : {fh.hfh(0), fh.hfh(1)}) {
            for (int shift = 0; shift < n; ++shift) {
                bool all_eq(true);
                for (int i = 0; i < n; ++i) {
                    if (vh(hfh, (i+shift)%n) != _vhs[i]) {
                        all_eq = false;
                        break;
                    }
                }
                if (all_eq) {return hfh;}
            }
        }
    }
    return HFH();
}

FH TopologicalCellComplex::find_face(const std::vector<VH>& _vhs) const
{
    HFH hfh = find_halfface(_vhs);
    return hfh.is_valid()? hfh.fh() : FH();
}

EH TopologicalCellComplex::faces_shared_edge(FH fi, FH fj) const
{
    for (EH eh1 : face_edges(fi)) {
        for (EH eh2 : face_edges(fj)) {
            if (eh1 == eh2) {return eh1;}
        }
    }
    return EH();
};

bool TopologicalCellComplex::vertices_are_adjacent(VH _vh0, VH _vh1) const
{
    return find_edge(_vh0, _vh1).is_valid();
}

bool TopologicalCellComplex::faces_are_adjacent(FH _fi, FH _fj) const
{
    return faces_shared_edge(_fi, _fj).is_valid();
}

bool TopologicalCellComplex::edge_contains_vertex(EH _eh, VH _vh) const
{
    HEH heh = _eh.heh(0);
    return vh0(heh) == _vh || vh1(heh) == _vh;
}

bool TopologicalCellComplex::face_contains_vertex(FH _fh, VH _vh) const
{
    for (VH vj : face_vertices(_fh)) {
        if (vj == _vh) {return true;}
    }
    return false;
}

bool TopologicalCellComplex::face_contains_edge(FH _fh, EH _eh) const
{
    for (EH eh : face_edges(_fh)) {
        if (eh == _eh) {return true;}
    }
    return false;
}

CH TopologicalCellComplex::halfface_incident_cell(HFH _hfh) const
{
    return faces_[_hfh.fh()].chs_[_hfh.subidx()];
}

uint32_t TopologicalCellComplex::vertex_num_face_components(VH _vi) const
{
    std::vector<FH> v_faces;
    for (auto fh : vertex_faces(_vi)) {
        v_faces.push_back(fh);
    }
    if (v_faces.empty()) {return 0u;}
    uint32_t num_components(0);
    std::vector<bool> visited(v_faces.size(), false);
    for (uint32_t i_seed = 0; i_seed < v_faces.size(); ++i_seed) {
        if (visited[i_seed]) {continue;}
        std::queue<uint32_t> q;
        q.push(i_seed);
        while (!q.empty()) {
            uint32_t i = q.front();
            q.pop();
            if (visited[i]) {continue;}
            visited[i] = true;
            for (uint32_t j = 0; j < v_faces.size(); ++j) {
                if (j != i &&
                    !visited[j] &&
                    faces_are_adjacent(v_faces[i], v_faces[j]))
                {
                    q.push(j);
                }
            }
        }
        ++num_components;
    }
    return num_components;
}

bool TopologicalCellComplex::surface_mesh_is_manifold() const
{
    for (EH eh : edges()) {
        if (!surface_edge_is_manifold(eh)) {
            return false;
        }
    }
    for (VH vh : vertices()) {
        if (!surface_vertex_is_manifold(vh)) {
            return false;
        }
    }
    return true;
}

}
