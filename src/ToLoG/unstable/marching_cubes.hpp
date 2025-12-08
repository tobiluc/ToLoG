#pragma once

#include <ToLoG/Core.hpp>

namespace ToLoG
{

class MarchingCubes3d
{
public:
    using Point = Point<double,3>;
    using AABB = AABB<Point>;
    using SDF = std::function<double(const Point&)>;

    struct TriangleMesh {
        std::vector<Point> vertices;
        std::vector<std::array<uint,3>> triangles;
    };

    struct Settings {
        size_t nx = 32, ny = 32, nz = 32; // number of vertices per dimension
        AABB bounds = {{-5,5,-5},{5,-5,5}};
        uint maxDepth = 5;
    };

private:
    MarchingCubes3d() {}

    const static int edgeTable[256];
    const static int triangleTable[256][16];
    const static Point cubeVertices[8];
    const static int cubeEdges[12][2];

    /// Generate Triangles on a single node
    static int generate(const SDF& f, TriangleMesh &mesh, const AABB& bounds);

public:
    /// Using an Octree
    static TriangleMesh generate_adaptive(const SDF& sdf, const Settings& settings);

};

}
