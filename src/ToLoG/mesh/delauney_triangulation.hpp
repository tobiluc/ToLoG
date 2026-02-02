#pragma once
#include <ToLoG/mesh/mesh_types.hpp>
#include <cassert>
#include <iostream>
#include <numeric>
#include <ostream>
#include <stack>
#include <ToLoG/io/ply_writer.hpp>

namespace ToLoG
{

template<typename Point>
requires(is_vector_type<Point>::value && Traits<Point>::dim == 2)
std::pair<
    std::vector<Point>,
    std::vector<typename Mesh<Point>::Cell>> delauney_triangulation(
    const std::vector<Point>& _pts)
{
    using FT = typename Traits<Point>::value_type;

    struct Triangle {
        uint32_t v[3] = {
            UINT32_MAX,UINT32_MAX,UINT32_MAX
        }; // vertices
        uint32_t adj[3] = {
            UINT32_MAX,UINT32_MAX,UINT32_MAX
        }; // adj[i] is triangle adjacent over edge opposite to v[i]
        uint32_t child_idx = UINT32_MAX; // 1st child idx (others are idx+1/2)
        inline uint32_t operator[](int i) const {return v[i];}
    };

    // Get Points normalized
    AABB<Point> bbox(_pts);
    FT max_size = max(bbox.max() - bbox.min());
    std::vector<Point> pts;
    pts.reserve(_pts.size()+3);
    for (const auto& p : _pts) {
        pts.emplace_back((p-bbox.min())/max_size);
    }

    std::vector<Triangle> tris;

    // Add 3 more points that make up the big triangle
    pts.push_back(Point(-100,-100));
    pts.push_back(Point(100,-100));
    pts.push_back(Point(0,100));
    tris.emplace_back();
    tris.back().v[0] = pts.size()-3;
    tris.back().v[1] = pts.size()-2;
    tris.back().v[2] = pts.size()-1;
    tris.back().adj[0] = tris.back().adj[1] = tris.back().adj[2] = UINT32_MAX;

    auto split_triangle = [&](uint32_t tri_i, uint32_t pt_i)
    {
        // Create 3 new triangles around pt (assumes pt inside tri)
        uint32_t i0 = tris.size();
        uint32_t i1 = i0+1;
        uint32_t i2 = i1+1;
        tris[tri_i].child_idx = i0;

        tris.emplace_back();
        tris.back().v[0] = pt_i;
        tris.back().v[1] = tris[tri_i].v[0];
        tris.back().v[2] = tris[tri_i].v[1];
        tris.back().adj[0] = tris[tri_i].adj[2];
        tris.back().adj[1] = i1;
        tris.back().adj[2] = i2;
        for (int j = 0; j < 3; ++j) {
            if (tris[tri_i].adj[2] < UINT32_MAX && tris[tris[tri_i].adj[2]].adj[j] == tri_i) {
                tris[tris[tri_i].adj[2]].adj[j] = i0;
                break;
            }
        }

        tris.emplace_back();
        tris.back().v[0] = pt_i;
        tris.back().v[1] = tris[tri_i].v[1];
        tris.back().v[2] = tris[tri_i].v[2];
        tris.back().adj[0] = tris[tri_i].adj[0];
        tris.back().adj[1] = i2;
        tris.back().adj[2] = i0;
        for (int j = 0; j < 3; ++j) {
            if (tris[tri_i].adj[0] < UINT32_MAX && tris[tris[tri_i].adj[0]].adj[j] == tri_i) {
                tris[tris[tri_i].adj[0]].adj[j] = i1;
                break;
            }
        }

        tris.emplace_back();
        tris.back().v[0] = pt_i;
        tris.back().v[1] = tris[tri_i].v[2];
        tris.back().v[2] = tris[tri_i].v[0];
        tris.back().adj[0] = tris[tri_i].adj[1];
        tris.back().adj[1] = i0;
        tris.back().adj[2] = i1;
        for (int j = 0; j < 3; ++j) {
            if (tris[tri_i].adj[1] < UINT32_MAX && tris[tris[tri_i].adj[1]].adj[j] == tri_i) {
                tris[tris[tri_i].adj[1]].adj[j] = i2;
                break;
            }
        }
    };

    auto locate_triangle = [&](uint32_t tri_i, uint32_t pt_i) -> uint32_t
    {
        while (tris[tri_i].child_idx != UINT32_MAX)
        {
            uint32_t ci = tris[tris[tri_i].child_idx].v[0]; // center point index of triangle
            //auto o = point_orient2d(pts[tris[tri_i].v[0]], pts[tris[tri_i].v[1]], pts[tris[tri_i].v[2]]);
            const double o0 = point_orient2d(pts[ci], pts[tris[tri_i].v[0]], pts[pt_i]);
            const double o1 = point_orient2d(pts[ci], pts[tris[tri_i].v[1]], pts[pt_i]);
            const double o2 = point_orient2d(pts[ci], pts[tris[tri_i].v[2]], pts[pt_i]);

            if (o0>=0.0&&o1<=0.0) {tri_i = tris[tri_i].child_idx;}
            else if (o1>=0.0&&o2<=0.0) {tri_i = tris[tri_i].child_idx+1;}
            else if (o2>=0.0&&o0<=0.0) {tri_i = tris[tri_i].child_idx+2;}
            else {std::exit(1);}
        }
        return tri_i;
    };

    auto check_delauney = [&](uint32_t tri_i)
    {
        std::stack<uint32_t> stack;
        stack.push(tri_i);

        while (!stack.empty())
        {
            tri_i = stack.top();
            stack.pop();

            // Check with each neighbour
            for (int j = 0; j < 3; ++j) {

                // Get neighbour
                uint32_t tri_j = tris[tri_i].adj[j];
                if (tri_j == UINT32_MAX) {continue;}

                // Get the three vertices of the neighboring triangle
                // s.t. v0, v1 are the common edge, v2 is the 3rd triangle vertex
                // and v3 is the 3rd triangle vertex of the neighbour
                uint32_t v2 = tris[tri_i].v[j];
                uint32_t v0 = tris[tri_i].v[(j+1)%3];
                uint32_t v1 = tris[tri_i].v[(j+2)%3];
                uint32_t v3 = UINT32_MAX;
                for (int k = 0; k < 3; ++k) {
                    if (tris[tri_j].adj[k] == tri_i) {
                        v3 = tris[tri_j].v[k];
                        break;
                    }
                }
                assert(v3 != UINT32_MAX && v3 != v0 && v3 != v1 && v3 != v2);

                // Incircle Test
                if (!point_incircle(
                    pts[v0],
                    pts[v1],
                    pts[v2],
                    pts[v3]))
                {
                    // Push neighbours onto stack
                    for (int k = 0; k < 3; ++k) {
                        if (tris[tri_j].adj[k] != tri_i) {stack.push(tris[tri_j].adj[k]);}
                        if (tris[tri_i].adj[k] != tri_j) {stack.push(tris[tri_i].adj[k]);}
                    }

                    // Flip edge between triangles
                    //TODO


                }
            }
        }
    };

    // Insert points 1 by 1
    for (uint32_t pt_i = 0; pt_i < pts.size()-3; ++pt_i)
    {
        // Get the triangle containing pt
        uint32_t tri_i = locate_triangle(0, pt_i);

        // Split the triangle
        split_triangle(tri_i, pt_i);

        // Check delauney for the created children
        // check_delauney(tris[tri_i].child_idx);
        // check_delauney(tris[tri_i].child_idx+1);
        // check_delauney(tris[tri_i].child_idx+2);
    }

    // Remove sentinel triangles
    tris.erase(std::remove_if(
        tris.begin(), tris.end(),
        [&](const Triangle& t) {
            if (t.child_idx != UINT32_MAX) // not leaf
            {return true;}
            for (int j = 0; j < 3; ++j) {
                for (uint32_t v = pts.size()-3; v < pts.size(); ++v) {
                    if (t.v[j] == v) {return true;} // part of big triangle
                }
            }
            return false;
        }), tris.end());
    pts.pop_back();
    pts.pop_back();
    pts.pop_back();
    write_triangles_ply("/Users/tobiaskohler/Desktop/delauney.ply", pts, tris);

    return {{}, {}};
}

}
