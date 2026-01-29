#pragma once

template<typename FT, int DIM>
struct is_point<Point<FT, DIM>> : std::true_type {};

template<typename P>
struct Traits<AABB<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename FT, int DIM>
struct Traits<Point<FT,DIM>>
{
    using value_type = FT;
    using vector_type = Point<FT,DIM>;
    constexpr static int dim = DIM;
};

template<typename P>
struct Traits<Segment<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename P>
struct Traits<Triangle<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename P, int N>
struct Traits<Simplex<P,N>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename P>
struct Traits<Sphere<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};

template<typename P>
struct Traits<Tetrahedron<P>>
{
    using value_type = Traits<P>::value_type;
    using vector_type = Traits<P>::vector_type;
    constexpr static int dim = Traits<P>::dim;
};
