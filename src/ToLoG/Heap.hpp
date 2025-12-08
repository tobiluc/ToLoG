#pragma once

#include <vector>

namespace ToLoG
{

/// Default is Max Heap
template<typename T, typename Cmp = std::less<T>>
class Heap
{
private:
    std::vector<T> elements_;
    size_t size_ = 0;
    Cmp cmp;

    static inline constexpr size_t parent(size_t idx) {
        return (idx-1) / 2;
    }

    static inline constexpr size_t left_child(size_t idx) {
        return (idx<<1) + 1;
    }

    static inline constexpr size_t right_child(size_t idx) {
        return (idx<<1) + 2;
    }

    // TODO: This is way too slow!!!
    inline void heapify(size_t idx)
    {
        T tmp = std::move(elements_[idx]);
        size_t n = size();

        while (true) {
            size_t l = left_child(idx);
            size_t r = right_child(idx);

            if (l >= n) {break;}

            size_t next = l;
            if (r < n && cmp(elements_[next], elements_[r])) {
                next = r;
            }

            if (!cmp(tmp, elements_[next])) {break;}

            elements_[idx] = std::move(elements_[next]);
            idx = next;
        }
        elements_[idx] = std::move(tmp);
    }

public:
    Heap(size_t _capacity = 16) {
        elements_.reserve(_capacity);
    }

    inline size_t size() const {
        return size_;
    }

    inline bool empty() const {
        return size_ == 0;
    }

    const T& front() const {
        return elements_.front();
    }

    const void pop_front()
    {
        if (empty()) {std::runtime_error("Can't pop of empty heap");}
        elements_[0] = std::move(elements_[size()-1]);
        --size_;
        heapify(0);
    }

    inline void insert(const T& _key)
    {
        size_t idx = size();
        if (idx < elements_.size()) {elements_[idx] = _key;}
        else {elements_.emplace_back(_key);}
        while (idx != 0 && cmp(elements_[parent(idx)], elements_[idx])) {
            std::swap(elements_[idx], elements_[parent(idx)]);
            idx = parent(idx);
        }
        ++size_;
    }
};

}
