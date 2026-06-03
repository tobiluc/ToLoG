#pragma once
#include <ToLoG/Core.hpp>
#include <functional>

namespace ToLoG
{

extern const int mc3_edge_table[256];
extern const int mc3_triangle_table[256][16];
extern const int mc3_cube_vertices[8][3];
extern const int mc3_cube_edges[12][4];

template<vector_of_dim<3> Point>
class MarchingCubes3d
{
public:
    using FT = Traits<Point>::value_type;
    using AABB = AABB<Point>;
    using SDF = std::function<FT(FT,FT,FT)>;

    struct TriangleMesh {
        std::vector<Point> vertices;
        std::vector<std::array<uint32_t,3>> triangles;
    };

    struct Settings {
        size_t nx = 32, ny = 32, nz = 32; // number of vertices per dimension
        AABB bounds = {{-5,5,-5},{5,-5,5}};
        uint maxDepth = 5;
    };

private:
    MarchingCubes3d() {}

public:
    static TriangleMesh generate(const SDF& _sdf,
                        const AABB& _bounds,
                        const uint32_t _resolution)
    {
        TriangleMesh mesh;

        // num voxels along single dimension
        // meaning n+1 vertices along dimension
        const uint32_t n = _resolution;

        // size of a single voxel
        const Point s = (_bounds.max()-_bounds.min()) / n;

        auto v_point_x = [&](uint32_t _x) -> FT {
            return _bounds.min()[0] + s[0]*_x;
        };
        auto v_point_y = [&](uint32_t _y) -> FT {
            return _bounds.min()[1] + s[1]*_y;
        };
        auto v_point_z = [&](uint32_t _z) -> FT {
            return _bounds.min()[2] + s[2]*_z;
        };
        auto v_point = [&](uint32_t _x, uint32_t _y, uint32_t _z) -> Point {
            return {v_point_x(_x), v_point_y(_y), v_point_z(_z)};
        };
        auto v_flatten_index = [&](uint32_t _x, uint32_t _y, uint32_t _z) -> uint32_t {
            return _x * ((n+1)*(n+1)) + _y * (n+1) + _z;
        };
        auto v_nest_index = [&](uint32_t _i) -> std::array<uint32_t,3> {
            const uint32_t s2d = (n + 1) * (n + 1);
            const uint32_t s1d = (n + 1);
            const uint32_t x = _i / s2d;
            const uint32_t y = (_i % s2d) / s1d;
            const uint32_t z = _i % s1d;
            return {x, y, z};
        };

        // Evaluate sdf per vertex
        std::vector<FT> v_vals;
        v_vals.resize((n+1)*(n+1)*(n+1));
        uint32_t num_pos(0);
        uint32_t num_neg(0);
        for (uint32_t x = 0; x <= n; ++x) {
            const FT px = v_point_x(x);
            for (uint32_t y = 0; y <= n; ++y) {
                const FT py = v_point_y(y);
                for (uint32_t z = 0; z <= n; ++z) {
                    const FT pz = v_point_z(z);
                    FT val = _sdf(px, py, pz);
                    v_vals[v_flatten_index(x,y,z)] = val;
                    num_pos += (val>0);
                    num_neg += (val<0);
                }
            }
        }

        // Mesh Memory Reservation
        if (num_pos > 0 && num_neg > 0)
        {
            // Estimate how many voxels cross the isosurface
            double estimated_voxels = std::clamp<double>(
                std::min(num_pos, num_neg) * 1.5,
                0.0,
                n * n * n);
            // Estimate Vertices/Triangles
            mesh.vertices.reserve(static_cast<size_t>(estimated_voxels * 2.0));
            mesh.triangles.reserve(static_cast<size_t>(estimated_voxels * 2.5));
        } else {
            // whole volume is entirely inside or outside.
            return mesh;
        }

        // Store intersection per edge
        // so we can easily connect the mesh
        auto e_flatten_index = [&](uint32_t _x, uint32_t _y, uint32_t _z, uint8_t _axis) -> uint32_t {
            return v_flatten_index(_x, _y, _z)*3 + _axis;
        };
        std::vector<int32_t> e_mesh_vertex((n+1)*(n+1)*(n+1)*3, -1);

        // Classify each voxel
        for (uint32_t ox = 0; ox < n; ++ox) {
            for (uint32_t oy = 0; oy < n; ++oy) {
                for (uint32_t oz = 0; oz < n; ++oz) {
                    int code = 0;
                    for (int i = 0; i < 8; ++i) {
                        uint32_t idx = v_flatten_index(
                            ox + mc3_cube_vertices[i][0],
                            oy + mc3_cube_vertices[i][1],
                            oz + mc3_cube_vertices[i][2]
                            );
                        if (v_vals[idx] < 0) {
                            code |= (1 << i);
                        }
                    }

                    // Get the points per cube edge
                    int edges = mc3_edge_table[code];
                    if (edges == 0) {continue;}

                    // Interpolate positions
                    std::array<uint32_t,12> cell_vertices;
                    for (int e = 0; e < 12; ++e) {
                        if (edges & (1 << e))
                        {
                            // Get global index for edge
                            // and its vertices
                            std::array<uint32_t, 3> v0 = {
                                ox + mc3_cube_edges[e][0],
                                oy + mc3_cube_edges[e][1],
                                oz + mc3_cube_edges[e][2]};
                            uint32_t e_axis = mc3_cube_edges[e][3];
                            uint32_t e_idx = e_flatten_index(v0[0], v0[1], v0[2], e_axis);

                            if (e_mesh_vertex[e_idx] < 0)
                            {
                                // Compute point on edge for the first time

                                std::array<uint32_t, 3> v1 = v0;
                                v1[e_axis] += 1;

                                const uint32_t i0 = v_flatten_index(v0[0], v0[1], v0[2]);
                                const uint32_t i1 = v_flatten_index(v1[0], v1[1], v1[2]);

                                const Point p0 = v_point(v0[0], v0[1], v0[2]);
                                Point p1 = p0; p1[e_axis] += s[e_axis];
                                const FT t = v_vals[i0] / (v_vals[i0] - v_vals[i1]);

                                // Add new vertex on edge
                                e_mesh_vertex[e_idx] = static_cast<int32_t>(mesh.vertices.size());
                                mesh.vertices.emplace_back(p0 + (p1 - p0) * t);
                            }

                            // Retrieve the mesh vertex index
                            cell_vertices[e] = static_cast<uint32_t>(e_mesh_vertex[e_idx]);
                        }
                    }

                    // Construct the triangles using the shared vertex indices
                    for (int i = 0; mc3_triangle_table[code][i] != -1; i += 3) {
                        uint32_t i0 = cell_vertices[mc3_triangle_table[code][i+0]];
                        uint32_t i1 = cell_vertices[mc3_triangle_table[code][i+1]];
                        uint32_t i2 = cell_vertices[mc3_triangle_table[code][i+2]];
                        mesh.triangles.push_back({i0, i1, i2});
                    }
                }
            }
        }
        return mesh;
    }
};

}
