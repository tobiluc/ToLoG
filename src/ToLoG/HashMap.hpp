#pragma once

#include <vector>
#include <functional>
#include <optional>
#include <cstring>

namespace ToLoG
{

template <typename K, typename V, typename H = std::hash<K>>
class HashMap
{
public:
    using key_type   = K;
    using value_type = V;
    using hash = H;

    HashMap(size_t _capacity = 16)
        : size_(0)
    {
        capacity_ = next_pow2(_capacity);
        keys_.resize(capacity_);
        values_.resize(capacity_);
        occupied_.resize(capacity_, false);
    }

    inline size_t size() const {
        return size_;
    }

    inline size_t capacity() const {
        return capacity_;
    }

    inline bool empty() const {
        return size_ == 0;
    }

    inline void insert(const K& key, const V& value)
    {
        ensure_capacity();

        size_t idx = probe(key);
        if (!occupied_[idx]) {
            occupied_[idx] = true;
            keys_[idx] = key;
            values_[idx] = value;
            ++size_;
        } else {
            values_[idx] = value;  // overwrite
        }
    }

    inline bool erase(const K& key) {
        size_t idx = find(key);
        if (idx == nullidx) {return false;}
        erase_at(idx);
        return true;
    }

    inline V& operator[](const K& key) {
        ensure_capacity();
        size_t idx = probe(key);

        if (!occupied_[idx]) {
            occupied_[idx] = true;
            keys_[idx] = key;
            values_[idx] = V();
            ++size_;
        }
        return values_[idx];
    }

    inline std::optional<std::reference_wrapper<const V>> get(const K& key) const {
        size_t idx = find(key);
        if (idx == nullidx) {return std::nullopt;}
        return std::cref(values_[idx]);
    }

    inline bool contains(const K& key) const {
        return find(key) != nullidx;
    }

private:
    size_t size_;
    size_t capacity_;
    std::vector<K> keys_;
    std::vector<V> values_;
    std::vector<bool> occupied_;

    static constexpr size_t nullidx = static_cast<size_t>(-1);

    inline size_t hash_key(const K& key) const {
        return hash{}(key) & (capacity_-1);
    }

    // linear probing
    inline size_t probe(const K& key) {
        size_t idx = hash_key(key);
        while (occupied_[idx] && !(keys_[idx] == key)) {
            idx = next_index(idx);
        }
        return idx;
    }

    inline size_t find(const K& key) const {
        size_t idx = hash_key(key);
        size_t start = idx;

        while (occupied_[idx]) {
            if (keys_[idx] == key) {return idx;}
            idx = next_index(idx);
            if (idx == start) {break;} // key not found
        }
        return nullidx;
    }

    inline void ensure_capacity() {
        if (size_*1.3 < capacity_) {return;}
        rehash(capacity_<<1);
    }

    inline void rehash(size_t new_cap)
    {
        new_cap = next_pow2(new_cap);

        std::vector<K> old_keys = std::move(keys_);
        std::vector<V> old_vals = std::move(values_);
        std::vector<bool> old_occ = std::move(occupied_);

        keys_.resize(new_cap);
        values_.resize(new_cap);
        occupied_.assign(new_cap, false);

        size_t old_cap = capacity_;
        capacity_ = new_cap;
        size_ = 0;

        for (size_t i = 0; i < old_cap; ++i)
        {
            if (old_occ[i]) {
                insert(old_keys[i], old_vals[i]);
            }
        }
    }

    inline void erase_at(size_t idx)
    {
        occupied_[idx] = false;
        --size_;

        size_t next = next_index(idx);

        while (occupied_[next]) {
            K k = std::move(keys_[next]);
            V v = std::move(values_[next]);

            occupied_[next] = false;
            --size_;

            insert(k, v);

            next = next_index(next);
        }
    }

    inline size_t next_index(size_t idx) const {
        return (idx + 1) & (capacity_ - 1);
    }

    static inline size_t next_pow2(size_t x) {
        size_t p = 1;
        while (p < x) {p <<= 1;}
        return p;
    }
};

}
