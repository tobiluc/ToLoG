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

    return {{}, {}};
}

}
