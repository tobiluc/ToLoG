#pragma once

#include "IGRec/geometry/BoundingBox.h"
#include "IGRec/geometry/sdf.h"
#include "IGRec/typedefs/TypedefsEigen.h"

namespace IGRec::Geometry
{

class MarchingCubes
{
public:
    struct TriangleMesh {
        std::vector<Eig::Vec3d> vertices;
        std::vector<std::array<uint,3>> triangles;
    };

    struct Settings {
        size_t nx = 32, ny = 32, nz = 32; // number of vertices per dimension
        BoundingBox bounds = {-5,5,-5,5,-5,5};
        uint maxDepth = 5;
    };

private:
    MarchingCubes() {}

    const static int edgeTable[256];
    const static int triangleTable[256][16];
    const static Eig::Vec3d cubeVertices[8];
    const static int cubeEdges[12][2];

    /// Generate Triangles on a single node
    static int generate(const SDF<Eig::Vec3d> &f, TriangleMesh &mesh, const BoundingBox& bounds);

public:
    /// Generates a Triangle Mesh representing the surface f(x,y,z) = 0
    //void generate(SDF<Eig::Vec3d>& f, TriangleMesh& mesh);

    /// Using an Octree
    static TriangleMesh generateAdaptive(const SDF<Eig::Vec3d>& sdf, const Settings& settings);

};

}
