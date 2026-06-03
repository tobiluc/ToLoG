#include <ToLoG/mesh/PolygonMesh.hpp>
#include <queue>

namespace ToLoG
{

using VH = PolygonMeshTopologyKernel::VH;
using EH = PolygonMeshTopologyKernel::EH;
using HEH = PolygonMeshTopologyKernel::HEH;
using FH = PolygonMeshTopologyKernel::FH;

void PolygonMeshTopologyKernel::reserve_vertices(uint32_t _size)
{
    vertex_out_halfedges_.reserve(_size);
    vertex_deleted_.reserve(_size);
}

void PolygonMeshTopologyKernel::reserve_edges(uint32_t _size)
{
    edges_.reserve(_size);
    edge_faces_.reserve(_size);
    edge_deleted_.reserve(_size);
}

void PolygonMeshTopologyKernel::reserve_faces(uint32_t _size)
{
    faces_.reserve(_size);
    face_deleted_.reserve(_size);
}

VH PolygonMeshTopologyKernel::add_vertex()
{
    VH vh(n_vertices_);
    vertex_out_halfedges_.push_back({});
    vertex_deleted_.push_back(false);
    ++n_vertices_;
    return vh;
}

HEH PolygonMeshTopologyKernel::add_halfedge(VH _vh0, VH _vh1)
{
    HEH heh = find_halfedge(_vh0, _vh1);
    if (heh.is_valid()) {return heh;}
    if (_vh0 >= num_vertices() || _vh1 >= num_vertices()) {
        throw std::runtime_error("vertex for edge does not exist!");
    }
    heh = HEH(num_halfedges());
    edges_.push_back({_vh0, _vh1});
    edge_deleted_.push_back(false);
    vertex_out_halfedges_[_vh0].push_back(heh);
    vertex_out_halfedges_[_vh1].push_back(heh.opp());
    edge_faces_.push_back({});
    return heh;
}

EH PolygonMeshTopologyKernel::add_edge(VH _vh0, VH _vh1)
{
    return add_halfedge(_vh0, _vh1).eh();
}

FH PolygonMeshTopologyKernel::add_face(const std::vector<HEH>& _hehs)
{
    if (_hehs.size() < 3) {
        throw std::runtime_error("face needs to have at least valence 3");
    }
    FH fh(num_faces());
    for (HEH heh : _hehs) {
        edge_faces_[heh.eh()].push_back(fh);
    }
    faces_.push_back(_hehs);
    face_deleted_.push_back(false);
    return fh;
}

FH PolygonMeshTopologyKernel::add_face(const std::vector<VH>& _vhs)
{
    std::vector<HEH> hehs(_vhs.size());
    for (int i = 0; i < _vhs.size(); ++i) {
        hehs[i] = add_halfedge(_vhs[i], _vhs[(i+1)%_vhs.size()]);
    }
    return add_face(hehs);
}

void PolygonMeshTopologyKernel::delete_vertex(VH _vh)
{
    for (EH eh : vertex_edges(_vh)) {
        delete_edge(eh);
    }
    vertex_out_halfedges_[_vh].clear();
    vertex_deleted_[_vh] = true;
}

void PolygonMeshTopologyKernel::delete_edge(EH _eh)
{
    // Remove references in incident vertices
    for (uint8_t subidx : {0,1}) {
        HEH heh = _eh.heh(subidx);
        vertex_out_halfedges_[vh0(heh)].erase(
            std::remove(vertex_out_halfedges_[vh0(heh)].begin(),
                        vertex_out_halfedges_[vh0(heh)].end(),
                        heh), vertex_out_halfedges_[vh0(heh)].end());
    }
    edge_faces_[_eh].clear();
    for (FH fh : edge_faces(_eh)) {
        delete_face(fh);
    }
    edge_deleted_[_eh] = true;
}

void PolygonMeshTopologyKernel::delete_face(FH _fh)
{
    // Remove references in incident edges
    for (EH eh : face_edges(_fh)) {
        edge_faces_[eh].erase(
            std::remove(edge_faces_[eh].begin(),
                        edge_faces_[eh].end(),
                        _fh), edge_faces_[eh].end());
    }
    face_deleted_[_fh] = true;
}

HEH PolygonMeshTopologyKernel::find_halfedge(VH _vh0, VH _vh1) const
{
    for (HEH heh : vertex_out_halfedges(_vh0)) {
        if (vh1(heh) == _vh1) {
            return heh;
        }
    }
    return HEH();
}

EH PolygonMeshTopologyKernel::find_edge(VH _vh0, VH _vh1) const
{
    HEH heh = find_halfedge(_vh0, _vh1);
    return heh.is_valid()? heh.eh() : EH();
}

FH PolygonMeshTopologyKernel::find_face(const std::vector<VH>& _vhs) const
{
    uint32_t n = _vhs.size();
    if (n==0) {return FH();}
    for (FH fh : vertex_faces(_vhs.front())) {
        if (n != face_valence(fh)) {continue;}
        for (int shift = 0; shift < n; ++shift) {
            bool all_eq(true);
            for (int i = 0; i < n; ++i) {
                if (vh0(faces_[fh][(i+shift)%n]) != _vhs[i]) {
                    all_eq = false;
                    break;
                }
            }
            if (all_eq) {return fh;}
        }
    }
    return FH();
}

EH PolygonMeshTopologyKernel::faces_shared_edge(FH fi, FH fj) const
{
    for (EH eh1 : face_edges(fi)) {
        for (EH eh2 : face_edges(fj)) {
            if (eh1 == eh2) {return eh1;}
        }
    }
    return EH();
};

bool PolygonMeshTopologyKernel::vertices_are_adjacent(VH _vh0, VH _vh1) const
{
    return find_edge(_vh0, _vh1).is_valid();
}

bool PolygonMeshTopologyKernel::faces_are_adjacent(FH _fi, FH _fj) const
{
    return faces_shared_edge(_fi, _fj).is_valid();
}

bool PolygonMeshTopologyKernel::edge_contains_vertex(EH _eh, VH _vh) const
{
    HEH heh = _eh.heh(0);
    return vh0(heh) == _vh || vh1(heh) == _vh;
}

bool PolygonMeshTopologyKernel::face_contains_vertex(FH _fh, VH _vh) const
{
    for (VH vj : face_vertices(_fh)) {
        if (vj == _vh) {return true;}
    }
    return false;
}

bool PolygonMeshTopologyKernel::face_contains_edge(FH _fh, EH _eh) const
{
    for (EH eh : face_edges(_fh)) {
        if (eh == _eh) {return true;}
    }
    return false;
}

uint32_t PolygonMeshTopologyKernel::vertex_num_face_components(VH _vi) const
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

bool PolygonMeshTopologyKernel::mesh_is_manifold() const
{
    for (EH eh : edges()) {
        if (!edge_is_manifold(eh)) {
            return false;
        }
    }
    for (VH vh : vertices()) {
        if (!vertex_is_manifold(vh)) {
            return false;
        }
    }
    return true;
}

}
