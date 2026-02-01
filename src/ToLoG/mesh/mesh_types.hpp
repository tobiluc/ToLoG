#pragma once
#include <ToLoG/Core.hpp>

namespace ToLoG
{

template<typename P> requires(is_vector_type<P>::value)
struct Mesh
{
    template<int K>
    struct Entity {
        uint32_t v[K+1]; // vertices
        uint32_t a[K+1]; // neighbor across lower entity opposite v[i]
    };
    using Vertex = P;
    using Edge = Entity<1>;
    using Facet = Entity<Traits<P>::dim-1>;
    using Cell = Entity<Traits<P>::dim>;
};

}
