#include "ToLoG/Core.hpp"
#include <gtest/gtest.h>
#include <ToLoG/mesh/tet_topology.hpp>
#include <ToLoG/predicates/predicates_wrapper.hpp>

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

    EXPECT_EQ(TT::opp(V::A), HF::BDC);
    EXPECT_EQ(TT::opp(V::B), HF::CDA);
    EXPECT_EQ(TT::opp(V::C), HF::DBA);
    EXPECT_EQ(TT::opp(V::D), HF::ABC);

    EXPECT_EQ(TT::opp(HE::AB), HE::BA);
    EXPECT_EQ(TT::opp(HE::DB), HE::BD);

    EXPECT_EQ(TT::opp(HF::ABC), V::D);

    EXPECT_EQ(TT::opp(V::A), HF::BDC);
    EXPECT_EQ(TT::opp(V::C), HF::DBA);

    EXPECT_EQ(TT::v0(HF::DBA), V::D);
    EXPECT_EQ(TT::v1(HF::DBA), V::B);
    EXPECT_EQ(TT::v2(HF::DBA), V::A);

    EXPECT_EQ(TT::he0(HF::ACD), HE::AC);
    EXPECT_EQ(TT::he1(HF::ACD), HE::CD);
    EXPECT_EQ(TT::he2(HF::ACD),  HE::DA);

    EXPECT_EQ(TT::hf(V::C, V::A, V::B), TT::HF::CAB);
    EXPECT_EQ(TT::opp(TT::hf(V::C, V::A, V::B)), V::D);

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
