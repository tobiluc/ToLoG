#include <ToLoG/mesh/cell_complex_iterators.hpp>
#include <ToLoG/mesh/CellComplex.hpp>

namespace ToLoG::Mesh
{

VEIter::VEIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
    (end_idx() > 0)? update() : set_end();
}
int VEIter::end_idx() const {
    return mesh()->vertices_[ref()].out_hehs_.size();
}
void VEIter::update() {
    curr_ = mesh()->vertices_[ref()].out_hehs_[idx()].eh();
}

VOHEIter::VOHEIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
    (end_idx() > 0)? update() : set_end();
}
int VOHEIter::end_idx() const {
    return mesh()->vertices_[ref()].out_hehs_.size();
}
void VOHEIter::update() {
    curr_ = mesh()->vertices_[ref()].out_hehs_[idx()];
}

VIHEIter::VIHEIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
    (end_idx() > 0)? update() : set_end();
}
int VIHEIter::end_idx() const {
    return mesh()->vertices_[ref()].out_hehs_.size();
}
void VIHEIter::update() {
    curr_ = mesh()->vertices_[ref()].out_hehs_[idx()].opp();
}

VVIter::VVIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
    (end_idx() > 0)? update() : set_end();
}
int VVIter::end_idx() const {
    return mesh()->vertices_[ref()].out_hehs_.size();
}
void VVIter::update() {
    curr_ = mesh()->vh1(mesh()->vertices_[ref()].out_hehs_[idx()]);
}

VFIter::VFIter(const TopologicalCellComplex* _mesh, VH _vh) : IterT(_mesh, _vh) {
    fhs_.clear();
    for (EH eh : mesh()->vertex_edges(_vh)) {
        for (FH fh : mesh()->edge_faces(eh)) {
            if (std::find(fhs_.begin(), fhs_.end(), fh) == fhs_.end()) {
                fhs_.push_back(fh);
            }
        }
    }
    if (!fhs_.empty()) {
        update();
    }
}
int VFIter::end_idx() const {
    return fhs_.size();
}
void VFIter::update() {
    curr_ = fhs_[idx()];
}

EVIter::EVIter(const TopologicalCellComplex* _mesh, EH _eh) : IterT(_mesh, _eh) {
    update();
}
int EVIter::end_idx() const {
    return 2;
}
void EVIter::update() {
    curr_ = mesh()->edges_[ref()].vhs_[idx()];
}

EFIter::EFIter(const TopologicalCellComplex* _mesh, EH _eh) : IterT(_mesh, _eh) {
    (end_idx() > 0)? update() : set_end();
}
int EFIter::end_idx() const {
    return mesh()->edges_[ref()].incident_hfhs_.size();
}
void EFIter::update() {
    curr_ = mesh()->edges_[ref()].incident_hfhs_[idx()].fh();
}

FVIter::FVIter(const TopologicalCellComplex* _mesh, FH _fh) : IterT(_mesh, _fh) {
    update();
}
int FVIter::end_idx() const {
    return mesh()->face_valence(ref());
}
void FVIter::update() {
    curr_ = mesh()->vh(ref().hfh(0), idx());
}

HFVIter::HFVIter(const TopologicalCellComplex* _mesh, HFH _hfh) : IterT(_mesh, _hfh) {
    update();
}
int HFVIter::end_idx() const {
    return mesh()->face_valence(ref());
}
void HFVIter::update() {
    curr_ = mesh()->vh(ref(), idx());
}

HFHEIter::HFHEIter(const TopologicalCellComplex* _mesh, HFH _hfh) : IterT(_mesh, _hfh) {
    update();
}
int HFHEIter::end_idx() const {
    return mesh()->face_valence(ref());
}
void HFHEIter::update() {
    curr_ = mesh()->heh(ref(), idx());
}

FHEIter::FHEIter(const TopologicalCellComplex* _mesh, FH _fh) : IterT(_mesh, _fh) {
    update();
}
int FHEIter::end_idx() const {
    return mesh()->face_valence(ref());
}
void FHEIter::update() {
    curr_ = mesh()->faces_[ref()].hehs_[idx()];
}

FEIter::FEIter(const TopologicalCellComplex* _mesh, FH _fh) : IterT(_mesh, _fh) {
    update();
}
int FEIter::end_idx() const {
    return mesh()->face_valence(ref());
}
void FEIter::update() {
    curr_ = mesh()->faces_[ref()].hehs_[idx()].eh();
}

CHFIter::CHFIter(const TopologicalCellComplex* _mesh, CH _ch) : IterT(_mesh, _ch) {
    update();
}
int CHFIter::end_idx() const {
    return mesh()->cells_[ref()].hfhs_.size();
}
void CHFIter::update() {
    curr_ = mesh()->cells_[ref()].hfhs_[idx()];
}

CFIter::CFIter(const TopologicalCellComplex* _mesh, CH _ch) : IterT(_mesh, _ch) {
    update();
}
int CFIter::end_idx() const {
    return mesh()->cells_[ref()].hfhs_.size();
}
void CFIter::update() {
    curr_ = mesh()->cells_[ref()].hfhs_[idx()].fh();
}

CCIter::CCIter(const TopologicalCellComplex* _mesh, CH _ch) : IterT(_mesh, _ch) {
    update();
}
int CCIter::end_idx() const {
    return mesh()->cells_[ref()].hfhs_.size();
}
void CCIter::update() {
    while (!((curr_ = mesh()->incident_cell(mesh()->cells_[ref()].hfhs_[idx()].opp())).is_valid())) {
        if (++curr_idx_ >= end_idx()) {
            break;
        }
    }
    if (!curr_.is_valid()) {set_end();}
}

CVIter::CVIter(const TopologicalCellComplex* _mesh, CH _ch) : IterT(_mesh, _ch)
{
    for (HFH hfh : _mesh->cell_halffaces(_ch)) {
        for (VH vh : _mesh->halfface_vertices(hfh)) {
            if (std::find(vhs_.begin(), vhs_.end(), vh) == vhs_.end()) {
                vhs_.push_back(vh);
            }
        }
    }
    update();
}
int CVIter::end_idx() const {
    return vhs_.size();
}
void CVIter::update() {
    curr_ = vhs_[idx()];
}

TetCVIter::TetCVIter(const TopologicalTetrahedralCellComplex* _mesh, CH _ch) :
    IterT(_mesh, _ch)
{
    hfh0 = *mesh()->cell_halffaces(ref()).begin();
    update();
}
int TetCVIter::end_idx() const {
    return 4;
}
void TetCVIter::update() {
    if (idx() < 3) {
        curr_ = mesh()->vh(hfh0, idx());
    } else {
        assert(idx() == 3);
        curr_ = static_cast<const TopologicalTetrahedralCellComplex*>(mesh())->opposite_vertex(hfh0);
    }
}

}
