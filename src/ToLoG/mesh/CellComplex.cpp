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

VH TopologicalCellComplex::add_vertex()
{
    VH vh;
    if (deleted_vertices_.empty()) {
        vh = VH(num_allocated_vertices());
        vertices_.push_back();
        vertex_deleted_.push_back();
    } else {
        vh = deleted_vertices_.front();
        deleted_vertices_.pop();
        assert(is_deleted(vh));
    }
    vertices_[vh].out_hehs_.clear();
    vertex_deleted_[vh] = false;
    return vh;
}

HEH TopologicalCellComplex::add_halfedge(VH _vh0, VH _vh1)
{
    HEH heh = find_halfedge(_vh0, _vh1);
    if (heh.is_valid()) {return heh;}
    if (!is_active(_vh0) || !is_active(_vh1)) {
        throw std::runtime_error("vertex for edge does not exist!");
    }
    if (deleted_edges_.empty()) {
        heh = HEH(num_allocated_halfedges());
        edges_.push_back();
        edge_deleted_.push_back();
    } else {
        heh = deleted_edges_.front().heh(0);
        deleted_edges_.pop();
        assert(is_deleted(heh.eh()));
    }
    EH eh = heh.eh();
    edges_[eh].incident_hfhs_.clear();
    edge_deleted_[eh] = false;
    edges_[eh].vhs_ = {_vh0, _vh1};
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

    HFH hfh;
    if (deleted_faces_.empty()) {
        hfh = HFH(num_allocated_halffaces());
        faces_.push_back();
        face_deleted_.push_back();
    } else {
        hfh = deleted_faces_.front().hfh(0);
        deleted_faces_.pop();
        assert(is_deleted(hfh.fh()));
    }

    FH fh = hfh.fh();
    for (HEH heh : _hehs) {
        if (heh.subidx() == 0) {
            edges_[heh.eh()].incident_hfhs_.push_back(hfh);
        } else {
            edges_[heh.eh()].incident_hfhs_.push_back(hfh.opp());
        }
    }
    faces_[fh].chs_ = {CH(), CH()};
    face_deleted_[fh] = false;
    faces_[fh].hehs_ = _hehs;
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
    CH ch;
    if (deleted_cells_.empty()) {
        ch = CH(num_allocated_cells());
        cells_.push_back();
        cell_deleted_.push_back();
    } else {
        ch = deleted_cells_.front();
        deleted_cells_.pop();
        assert(is_deleted(ch));
    }
    for (HFH hfh : _hfhs) {
        if (incident_cell(hfh).is_valid()) {
            throw std::runtime_error("halfface already has an incident cell");
        }
        faces_[hfh.fh()].chs_[hfh.subidx()] = ch;
    }
    cell_deleted_[ch] = false;
    cells_[ch].hfhs_ = _hfhs;
    return ch;
}

void TopologicalCellComplex::delete_vertex(VH _vh)
{
    if (!is_active(_vh)) {return;}

    // Delete incident edges
    for (EH eh : vertex_edges(_vh)) {
        delete_edge(eh);
    }
    vertex_deleted_[_vh] = true;
    vertices_[_vh].out_hehs_.clear();
    deleted_vertices_.push(_vh);
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

    edge_deleted_[_eh] = true;
    edges_[_eh].incident_hfhs_.clear();
    deleted_edges_.push(_eh);
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

    face_deleted_[_fh] = true;
    faces_[_fh].hehs_.clear();
    deleted_faces_.push(_fh);
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

    cell_deleted_[_ch] = true;
    cells_[_ch].hfhs_.clear();
    deleted_cells_.push(_ch);
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

VH TopologicalCellComplex::shared_vertex(EH _eh1, EH _eh2) const
{
    for (VH vh1 : edges_[_eh1].vhs_) {
        for (VH vh2 : edges_[_eh2].vhs_) {
            if (vh1 == vh2) {
                return vh1;
            }
        }
    }
    return VH();
}

EH TopologicalCellComplex::shared_edge(FH _fh1, FH _fh2) const
{
    for (EH eh1 : face_edges(_fh1)) {
        for (EH eh2 : face_edges(_fh2)) {
            if (eh1 == eh2) {
                return eh1;
            }
        }
    }
    return EH();
};

FH TopologicalCellComplex::shared_face(CH _ch1, CH _ch2) const
{
    for (FH fh1 : cell_faces(_ch1)) {
        for (FH fh2 : cell_faces(_ch2)) {
            if (fh1 == fh2) {
                return fh1;
            }
        }
    }
    return FH();
}

bool TopologicalCellComplex::are_adjacent(VH _vh0, VH _vh1) const
{
    return find_edge(_vh0, _vh1).is_valid();
}

bool TopologicalCellComplex::are_adjacent(EH _eh1, EH _eh2) const
{
    return shared_vertex(_eh1, _eh2).is_valid();
}

bool TopologicalCellComplex::are_adjacent(FH _fh1, FH _fh2) const
{
    return shared_edge(_fh1, _fh2).is_valid();
}

bool TopologicalCellComplex::are_adjacent(CH _ch1, CH _ch2) const
{
    return shared_face(_ch1, _ch2).is_valid();
}

bool TopologicalCellComplex::are_incident(VH _vh, EH _eh) const
{
    HEH heh = _eh.heh(0);
    return vh0(heh) == _vh || vh1(heh) == _vh;
}

bool TopologicalCellComplex::are_incident(VH _vh, FH _fh) const
{
    for (VH vj : face_vertices(_fh)) {
        if (vj == _vh) {
            return true;
        }
    }
    return false;
}

bool TopologicalCellComplex::are_incident(VH _vh, CH _ch) const
{
    for (HFH hfh : cell_halffaces(_ch)) {
        for (VH vh : halfface_vertices(hfh)) {
            if (vh == _vh) {
                return true;
            }
        }
    }
    return false;
}

bool TopologicalCellComplex::are_incident(EH _eh, FH _fh) const
{
    for (EH eh : face_edges(_fh)) {
        if (eh == _eh) {return true;}
    }
    return false;
}

bool TopologicalCellComplex::are_incident(EH _eh, CH _ch) const
{
    for (FH fh : edge_faces(_eh)) {
        if (incident_cell(fh.hfh(0)) == _ch
            || incident_cell(fh.hfh(1)) == _ch) {
            return true;
        }
    }
    return false;
}

bool TopologicalCellComplex::are_incident(FH _fh, CH _ch) const
{
    return (incident_cell(_fh.hfh(0)) == _ch
            || incident_cell(_fh.hfh(1)) == _ch);
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
                    are_adjacent(v_faces[i], v_faces[j]))
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

void TopologicalCellComplex::collect_garbage()
{
    //TODO
    // VertexPropT<uint32_t> vh_idx(num_allocated_vertices());
    // EdgePropT<uint32_t> eh_idx(num_allocated_edges());
    // FacePropT<uint32_t> fh_idx(num_allocated_faces());
    // CellPropT<uint32_t> ch_idx(num_allocated_cells());
}

}
