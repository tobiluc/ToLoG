#pragma once

template<vector_type P>
struct Traits<AABB<P>>
{
    using vector_type = P;
};

template<typename FT, int DIM>
struct Traits<Point<FT,DIM>>
{
    using value_type = FT;
    using vector_type = Point<FT,DIM>;
    constexpr static int dim = DIM;
};

template<vector_type P>
struct Traits<Segment<P>>
{
    using vector_type = P;
};

template<vector_type P>
struct Traits<Triangle<P>>
{
    using vector_type = P;
};

template<vector_type P>
struct Traits<Sphere<P>>
{
    using vector_type = P;
};

template<vector_type P>
struct Traits<Tetrahedron<P>>
{
    using vector_type = P;
};
