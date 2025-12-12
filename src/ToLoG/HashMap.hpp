#pragma once

#include <vector>
#include <functional>
#include <optional>
#include <cstring>
#include <ostream>

namespace ToLoG
{

template <typename K, typename V, typename H = std::hash<K>>
class HashMap
{
private:
    template<bool is_const>
    class base_iterator {
    public:
        using map_type = std::conditional_t<is_const, const HashMap, HashMap>;
        using key_type = const K&;
        using value_type = std::conditional_t<is_const, const V&, V&>;

        base_iterator(map_type* map, size_t index)
            : map_(map), index_(index)
        {
            skip_empty();
        }

        inline base_iterator& operator++() {
            ++index_;
            skip_empty();
            return *this;
        }

        inline bool operator==(const base_iterator& _it) const {
            return map_ == _it.map_ && index_ == _it.index_;
        }

        inline bool operator!=(const base_iterator& other) const {
            return !(*this == other);
        }

        inline key_type key() const {
            return map_->keys_[index_];
        }

        inline value_type value() const {
            return map_->values_[index_];
        }

        inline std::pair<key_type,value_type> operator*() const {
            return {key(), value()};
        }

    private:
        void skip_empty() {
            while (index_ < map_->capacity_ && !map_->occupied_[index_]) {
                ++index_;
            }
        }

        map_type* map_;
        size_t index_;
    };

public:
    using key_type = K;
    using value_type = V;
    using hash = H;
    using const_iterator = base_iterator<true>;
    using iterator = base_iterator<false>;

    HashMap(size_t _capacity = 16)
        : size_(0), capacity_(0)
    {
        reserve(_capacity);
    }

    inline void reserve(size_t _capacity) {
        if (_capacity <= capacity_) {return;}
        capacity_ = next_pow2(_capacity);
        keys_.resize(capacity_);
        values_.resize(capacity_);
        occupied_.resize(capacity_, false);
    }

    inline iterator begin() {
        return iterator(this, 0);
    }

    inline iterator end() {
        return iterator(this, capacity_);
    }

    inline const_iterator cbegin() const {
        return const_iterator(this, 0);
    }

    inline const_iterator cend() const {
        return const_iterator(this, capacity_);
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

    inline size_t erase_if(std::function<bool(const K&,const V&)> _f) {
        size_t n(0);
        for (size_t idx = 0; idx < capacity_; ++idx) {
            if (occupied_[idx] && _f(keys_[idx],values_[idx])) {
                erase_at(idx);
                ++n;
            }
        }
        return n;
    }

    inline void clear() {
        capacity_ = next_pow2(16);
        keys_.resize(capacity_);
        values_.resize(capacity_);
        occupied_.resize(capacity_);
        size_ = 0;
        std::fill(occupied_.begin(), occupied_.end(), false);
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

    inline void apply(const std::function<void(const K& _k, V& _v)> _f) {
        for (size_t idx = 0; idx < capacity_; ++idx) {
            if (occupied_[idx]) {
                _f(keys_[idx], values_[idx]);
            }
        }
    }

    inline std::optional<std::reference_wrapper<const V>> get(const K& key) const {
        size_t idx = find(key);
        if (idx == nullidx) {return std::nullopt;}
        return std::cref(values_[idx]);
    }

    inline std::optional<std::reference_wrapper<V>> get(const K& key) {
        size_t idx = find(key);
        if (idx == nullidx) {return std::nullopt;}
        return std::ref(values_[idx]);
    }

    inline const V& get_or_default(const K& key, const V& _default) const {
        size_t idx = find(key);
        if (idx == nullidx) {return _default;}
        return values_[idx];
    }

    inline V& get_or_default(const K& key, const V& _default) {
        size_t idx = find(key);
        if (idx == nullidx) {return _default;}
        return values_[idx];
    }

    inline bool contains(const K& key) const {
        return find(key) != nullidx;
    }

    inline friend std::ostream& operator<<(std::ostream& _os, const HashMap<K,V>& _map) {
        for (auto it = _map.cbegin(); it != _map.cend(); ++it) {
            _os << "{" << it.key() << ": " << it.value() << "}";
        }
        return _os;
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

    inline size_t probe(const K& key) {
        size_t idx = hash_key(key);
        size_t start = idx;
        size_t i = 0;
        while (occupied_[idx] && !(keys_[idx] == key)) {
            idx = next_index(idx, ++i);
        }
        return idx;
    }

    inline size_t find(const K& key) const {
        size_t idx = hash_key(key);
        size_t start = idx;
        size_t i = 0;
        while (occupied_[idx]) {
            if (keys_[idx] == key) {return idx;}
            idx = next_index(idx, ++i);
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

        size_t i = 0;
        size_t next = next_index(idx, i);

        while (occupied_[next]) {
            K k = std::move(keys_[next]);
            V v = std::move(values_[next]);

            occupied_[next] = false;
            --size_;

            insert(k, v);

            next = next_index(next, ++i);
        }
    }

    inline size_t next_index(size_t idx, size_t i) const {
        return (idx + 1) & (capacity_ - 1);
        //return (idx + i + 1) & (capacity_ - 1);
    }

    static inline size_t next_pow2(size_t x) {
        size_t p = 1;
        while (p < x) {p <<= 1;}
        return p;
    }
};

}
