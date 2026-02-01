#pragma once
#include <ToLoG/mesh/mesh_types.hpp>

namespace ToLoG
{

/*
template<typename Point>
requires(is_vector_type<Point>::value && Traits<Point>::dim == 2)
std::vector<typename Mesh<Point>::Cell> delauney_triangulation(
    const std::vector<Point>& _pts)
{
    using Mesh = Mesh<Point>;
    using Triangle = Mesh::Cell;
    std::vector<Triangle> triangles;

    // Compute bounding box
    AABB<Point> bbox(_pts);

    //TODO

    return triangles;
}
*/

}
