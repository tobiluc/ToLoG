#include <gtest/gtest.h>
#include <ToLoG/utils/OVM_Traits.hpp>
#include <ToLoG/Core.hpp>
#include <ToLoG/geometry/predicates/predicates_wrapper.hpp>
#include <OpenVolumeMesh/Mesh/TetrahedralGeometryKernel.hh>

TEST(TetTopologyTest, Test1)
{
    using TT = ToLoG::TetTopology;
    using V = TT::V;
    using HE = TT::HE;
    using HF = TT::HF;
    using VAR = TT::VAR;

    EXPECT_TRUE(TT::is_vertex(VAR::B));
    EXPECT_FALSE(TT::is_halfedge(VAR::B));
    EXPECT_FALSE(TT::is_halfface(VAR::B));

    EXPECT_FALSE(TT::is_vertex(VAR::CB));
    EXPECT_TRUE(TT::is_halfedge(VAR::CB));
    EXPECT_FALSE(TT::is_halfface(VAR::CB));

    EXPECT_FALSE(TT::is_vertex(VAR::ADB));
    EXPECT_FALSE(TT::is_halfedge(VAR::ADB));
    EXPECT_TRUE(TT::is_halfface(VAR::ADB));

    EXPECT_EQ(TT::v0(HE::AB), V::A);
    EXPECT_EQ(TT::v1(HE::AB), V::B);

    EXPECT_EQ(TT::v0(HE::AB), V::A);
    EXPECT_EQ(TT::v1(HE::AB), V::B);
    EXPECT_EQ(TT::v0(HE::AC), V::A);
    EXPECT_EQ(TT::v1(HE::CA), V::A);
    EXPECT_EQ(TT::v0(HE::AD), V::A);
    EXPECT_EQ(TT::v1(HE::CB), V::B);
    EXPECT_EQ(TT::v0(HE::BC), V::B);
    EXPECT_EQ(TT::v1(HE::BC), V::C);

    EXPECT_EQ(TT::opp_hf(V::A), HF::BDC);
    EXPECT_EQ(TT::opp_hf(V::B), HF::CDA);
    EXPECT_EQ(TT::opp_hf(V::C), HF::DBA);
    EXPECT_EQ(TT::opp_hf(V::D), HF::ABC);

    EXPECT_EQ(TT::opp(HE::AB), HE::BA);
    EXPECT_EQ(TT::opp(HE::DB), HE::BD);

    EXPECT_EQ(TT::opp_v(HF::ABC), V::D);

    EXPECT_EQ(TT::opp_hf(V::A), HF::BDC);
    EXPECT_EQ(TT::opp_hf(V::C), HF::DBA);

    EXPECT_EQ(TT::v0(HF::DBA), V::D);
    EXPECT_EQ(TT::v1(HF::DBA), V::B);
    EXPECT_EQ(TT::v2(HF::DBA), V::A);

    EXPECT_EQ(TT::he0(HF::ACD), HE::AC);
    EXPECT_EQ(TT::he1(HF::ACD), HE::CD);
    EXPECT_EQ(TT::he2(HF::ACD),  HE::DA);

    EXPECT_EQ(TT::hf(V::C, V::A, V::B), TT::HF::CAB);
    EXPECT_EQ(TT::opp_v(TT::hf(V::C, V::A, V::B)), V::D);

    EXPECT_EQ(TT::incident_halfedges(V::A)[0], HE::AB);
    EXPECT_EQ(TT::incident_halfedges(V::A)[1], HE::AC);
    EXPECT_EQ(TT::incident_halfedges(V::A)[2], HE::AD);
    EXPECT_EQ(TT::incident_halfedges(V::C)[0], HE::CD);
    EXPECT_EQ(TT::incident_halfedges(V::C)[1], HE::CA);
    EXPECT_EQ(TT::incident_halfedges(V::C)[2], HE::CB);
}

TEST(TetTopologyTest, Test2)
{
    using TT = ToLoG::TetTopology;
    using V = TT::V;
    using HE = TT::HE;
    using HF = TT::HF;
    using VAR = TT::VAR;
    using P = ToLoG::Point<double,3>;
    using Tet = ToLoG::Tetrahedron<P>;
    using Tri = ToLoG::Triangle<P>;
    using Seg = ToLoG::Segment<P>;

    P a(0,0,0);
    P b(0,0,1);
    P c(1,0,0);
    P d(0,1,0);
    Tet tet(a,b,c,d);

    EXPECT_EQ(tet.triangle(HF::ABC), Tri(a,b,c));
    EXPECT_EQ(tet.triangle(HF::ACD), Tri(a,c,d));
    EXPECT_EQ(tet.triangle(HF::ADB), Tri(a,d,b));
    EXPECT_EQ(tet.segment(HE::BD), Seg(b,d));

    EXPECT_EQ(orient3d(a.data(),b.data(),c.data(),a.data()), 0.0);
}

TEST(TetTopologyTest, CanonicalIndexTest)
{
    using TT = ToLoG::TetTopology;
    using V = TT::V;
    using HE = TT::HE;
    using HF = TT::HF;

    EXPECT_EQ(TT::i(V::A), 0);
    EXPECT_EQ(TT::i(V::B), 1);
    EXPECT_EQ(TT::i(V::C), 2);
    EXPECT_EQ(TT::i(V::D), 3);

    EXPECT_EQ(TT::i2v(0), V::A);
    EXPECT_EQ(TT::i2v(1), V::B);
    EXPECT_EQ(TT::i2v(2), V::C);
    EXPECT_EQ(TT::i2v(3), V::D);
    EXPECT_EQ(TT::i2v(-1), V::O);
    EXPECT_EQ(TT::i2v(12), V::O);

    EXPECT_EQ(TT::i(HE::AB), 0);
    EXPECT_EQ(TT::i(HE::AC), 1);
    EXPECT_EQ(TT::i(HE::AD), 2);
    EXPECT_EQ(TT::i(HE::BA), 3);
    EXPECT_EQ(TT::i(HE::BC), 4);
    EXPECT_EQ(TT::i(HE::BD), 5);
    EXPECT_EQ(TT::i(HE::CA), 6);
    EXPECT_EQ(TT::i(HE::CB), 7);
    EXPECT_EQ(TT::i(HE::CD), 8);
    EXPECT_EQ(TT::i(HE::DA), 9);
    EXPECT_EQ(TT::i(HE::DB), 10);
    EXPECT_EQ(TT::i(HE::DC), 11);

    EXPECT_EQ(TT::i2he(0), HE::AB);
    EXPECT_EQ(TT::i2he(1), HE::AC);
    EXPECT_EQ(TT::i2he(2), HE::AD);
    EXPECT_EQ(TT::i2he(3), HE::BA);
    EXPECT_EQ(TT::i2he(4), HE::BC);
    EXPECT_EQ(TT::i2he(5), HE::BD);
    EXPECT_EQ(TT::i2he(6), HE::CA);
    EXPECT_EQ(TT::i2he(7), HE::CB);
    EXPECT_EQ(TT::i2he(8), HE::CD);
    EXPECT_EQ(TT::i2he(9), HE::DA);
    EXPECT_EQ(TT::i2he(10), HE::DB);
    EXPECT_EQ(TT::i2he(11), HE::DC);
    EXPECT_EQ(TT::i2he(-1), HE::O);
    EXPECT_EQ(TT::i2he(12), HE::O);

    EXPECT_EQ(TT::i(HF::BDC), 0);
    EXPECT_EQ(TT::i(HF::CDA), 1);
    EXPECT_EQ(TT::i(HF::DBA), 2);
    EXPECT_EQ(TT::i(HF::ABC), 3);

    EXPECT_EQ(TT::i2hf(0), HF::BDC);
    EXPECT_EQ(TT::i2hf(1), HF::CDA);
    EXPECT_EQ(TT::i2hf(2), HF::DBA);
    EXPECT_EQ(TT::i2hf(3), HF::ABC);
    EXPECT_EQ(TT::i2hf(4), HF::O);
    EXPECT_EQ(TT::i2hf(-1), HF::O);
}

TEST(TetTopologyTest, OVMTetTopologyTest)
{
    using TT = ToLoG::TetTopology;
    using P = OpenVolumeMesh::Vec3d;
    using VH = OpenVolumeMesh::VH;
    using HEH = OpenVolumeMesh::HEH;
    using TetMesh = OpenVolumeMesh::TetrahedralGeometryKernel<P, OpenVolumeMesh::TetrahedralMeshTopologyKernel>;
    TetMesh mesh;
    std::vector<VH> vhs = {
        mesh.add_vertex(P(0,0,0)),
        mesh.add_vertex(P(0,0,1)),
        mesh.add_vertex(P(1,0,0)),
        mesh.add_vertex(P(0,1,0))
    };
    OpenVolumeMesh::CH ch = mesh.add_cell(vhs[0],vhs[1],vhs[2],vhs[3]);

    ToLoG::OVMTetTopology tt(mesh, ch);
    EXPECT_EQ(tt.vertex(TT::V::A), vhs[0]);
    EXPECT_EQ(tt.vertex(TT::V::B), vhs[1]);
    EXPECT_EQ(tt.vertex(TT::V::C), vhs[2]);
    EXPECT_EQ(tt.vertex(TT::V::D), vhs[3]);

    EXPECT_EQ(tt.v(vhs[0]), TT::V::A);
    EXPECT_EQ(tt.v(vhs[1]), TT::V::B);
    EXPECT_EQ(tt.v(vhs[2]), TT::V::C);
    EXPECT_EQ(tt.v(vhs[3]), TT::V::D);
    EXPECT_EQ(tt.v(VH(-1)), TT::V::O);
    EXPECT_EQ(tt.v(VH(10)), TT::V::O);

    EXPECT_EQ(tt.halfedge(TT::HE::AB), mesh.find_halfedge(vhs[0], vhs[1]));
    EXPECT_EQ(tt.halfedge(TT::HE::CA), mesh.find_halfedge(vhs[2], vhs[0]));
    EXPECT_EQ(tt.halfedge(TT::HE::AD), mesh.find_halfedge(vhs[0], vhs[3]));
    EXPECT_EQ(tt.halfedge(TT::HE::BC), mesh.find_halfedge(vhs[1], vhs[2]));
    EXPECT_EQ(tt.halfedge(TT::HE::CD), mesh.find_halfedge(vhs[2], vhs[3]));
    EXPECT_EQ(tt.halfedge(TT::HE::DB), mesh.find_halfedge(vhs[3], vhs[1]));

    EXPECT_EQ(tt.he(mesh.find_halfedge(vhs[0], vhs[1])), TT::HE::AB);
    EXPECT_EQ(tt.he(mesh.find_halfedge(vhs[3], vhs[2])), TT::HE::DC);

    EXPECT_EQ(tt.halfface(TT::HF::ABC), mesh.find_halfface({vhs[0], vhs[1], vhs[2]}));
    EXPECT_EQ(tt.halfface(TT::HF::ACD), mesh.find_halfface({vhs[0], vhs[2], vhs[3]}));
    EXPECT_EQ(tt.halfface(TT::HF::ADB), mesh.find_halfface({vhs[0], vhs[3], vhs[1]}));
    EXPECT_EQ(tt.halfface(TT::HF::BDC), mesh.find_halfface({vhs[1], vhs[3], vhs[2]}));
}
