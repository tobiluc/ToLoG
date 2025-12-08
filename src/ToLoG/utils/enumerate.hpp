#pragma once
#include <cstddef>
#include <utility>

namespace ToLoG
{

template <typename T>
class enumerate
{
private:
    T& container_;
public:
    enumerate(T& _container) : container_(_container) {}

    struct iterator
    {
        std::size_t index = 0;
        decltype(std::begin(container_)) iter;
        auto operator!=(const iterator& other) const {return iter != other.iter;}
        void operator++() {++iter; ++index;}
        auto operator*() const {
            return std::pair<std::size_t, decltype(*iter)>(index, *iter);
        }
    };
    iterator begin() {return {0, std::begin(container_)};}
    iterator end() {return {0, std::end(container_)};}
};

}
