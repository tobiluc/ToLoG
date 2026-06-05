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

        base_iterator& operator++() {
            ++index_;
            skip_empty();
            return *this;
        }

        bool operator==(const base_iterator& _it) const {
            return map_ == _it.map_ && index_ == _it.index_;
        }

        bool operator!=(const base_iterator& other) const {
            return !(*this == other);
        }

        key_type key() const {
            return map_->buckets_[index_].key_;
        }

        value_type value() const {
            return map_->buckets_[index_].value_;
        }

        std::pair<key_type,value_type> operator*() const {
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

    void reserve(size_t _capacity) {
        if (_capacity <= capacity_) {return;}
        capacity_ = next_pow2(_capacity);
        buckets_.resize(capacity_);
        occupied_.resize(capacity_, false);
    }

    iterator begin() {
        return iterator(this, 0);
    }

    iterator end() {
        return iterator(this, capacity_);
    }

    const_iterator cbegin() const {
        return const_iterator(this, 0);
    }

    const_iterator cend() const {
        return const_iterator(this, capacity_);
    }

    size_t size() const {
        return size_;
    }

    size_t capacity() const {
        return capacity_;
    }

    bool empty() const {
        return size_ == 0;
    }

    void insert(const K& key, const V& value)
    {
        ensure_capacity();

        size_t idx = probe(key);
        if (!occupied_[idx]) {
            occupied_[idx] = true;
            buckets_[idx] = {.key_ = key, .value_ = value};
            ++size_;
        } else {
            buckets_[idx].value_ = value;  // overwrite
        }
    }

    bool erase(const K& key) {
        size_t idx = find(key);
        if (idx == nullidx) {return false;}
        erase_at(idx);
        return true;
    }

    size_t erase_if(std::function<bool(const K&,const V&)> _f) {
        size_t n(0);
        for (size_t idx = 0; idx < capacity_; ++idx) {
            if (occupied_[idx] && _f(buckets_[idx].key_,buckets_[idx].value_)) {
                erase_at(idx);
                ++n;
            }
        }
        return n;
    }

    void clear() {
        capacity_ = next_pow2(16);
        buckets_.resize(capacity_);
        occupied_.resize(capacity_);
        size_ = 0;
        std::fill(occupied_.begin(), occupied_.end(), false);
    }

    V& operator[](const K& key) {
        ensure_capacity();
        size_t idx = probe(key);

        if (!occupied_[idx]) {
            occupied_[idx] = true;
            buckets_[idx] = {.key_ = key, .value_ = V()};
            ++size_;
        }
        return buckets_[idx].value_;
    }

    void apply(const std::function<void(const K& _k, V& _v)> _f) {
        for (size_t idx = 0; idx < capacity_; ++idx) {
            if (occupied_[idx]) {
                _f(buckets_[idx].key_, buckets_[idx].value_);
            }
        }
    }

    std::optional<std::reference_wrapper<const V>> get(const K& key) const {
        size_t idx = find(key);
        if (idx == nullidx) {return std::nullopt;}
        return std::cref(buckets_[idx].value_);
    }

    std::optional<std::reference_wrapper<V>> get(const K& key) {
        size_t idx = find(key);
        if (idx == nullidx) {return std::nullopt;}
        return std::ref(buckets_[idx].value_);
    }

    const V& get_or_set(const K& _key, const V& _val) {
        std::optional<std::reference_wrapper<const V>> g = get(_key);
        if (g.has_value()) {return g.value();}
        this->operator[](_key) = _val;
        return _val;
    }

    const V& get_or_default(const K& key, const V& _default) const {
        size_t idx = find(key);
        if (idx == nullidx) {return _default;}
        return buckets_[idx].value_;
    }

    V& get_or_default(const K& key, const V& _default) {
        size_t idx = find(key);
        if (idx == nullidx) {return _default;}
        return buckets_[idx].value_;
    }

    bool contains(const K& key) const {
        return find(key) != nullidx;
    }

    friend std::ostream& operator<<(std::ostream& _os, const HashMap<K,V>& _map) {
        for (auto it = _map.cbegin(); it != _map.cend(); ++it) {
            _os << "{" << it.key() << ": " << it.value() << "}";
        }
        return _os;
    }

private:
    struct Bucket {
        enum class State : uint8_t {EMPTY, OCCUPIED, DELETED};
        K key_;
        V value_;
        //State state_;
    };

    size_t size_;
    size_t capacity_;

    std::vector<Bucket> buckets_;
    std::vector<uint8_t> occupied_;

    static constexpr size_t nullidx = static_cast<size_t>(-1);

    constexpr size_t hash_key(const K& key) const {
        return hash{}(key) & (capacity_-1);
    }

    size_t probe(const K& key) {
        size_t idx = hash_key(key);
        size_t start = idx;
        size_t i = 0;
        while (occupied_[idx] && !(buckets_[idx].key_ == key)) {
            idx = next_index(idx, ++i);
        }
        return idx;
    }

    size_t find(const K& key) const {
        size_t idx = hash_key(key);
        size_t start = idx;
        size_t i = 0;
        while (occupied_[idx]) {
            if (buckets_[idx].key_ == key) {return idx;}
            idx = next_index(idx, ++i);
            if (idx == start) {break;} // key not found
        }
        return nullidx;
    }

    void ensure_capacity() {
        if (size_*1.3 < capacity_) {return;}
        rehash(capacity_<<1);
    }

    void rehash(size_t new_cap)
    {
        new_cap = next_pow2(new_cap);

        std::vector<Bucket> old_buckets = std::move(buckets_);
        std::vector<uint8_t> old_occ = std::move(occupied_);

        buckets_.resize(new_cap);
        occupied_.assign(new_cap, false);

        size_t old_cap = capacity_;
        capacity_ = new_cap;
        size_ = 0;

        for (size_t i = 0; i < old_cap; ++i)
        {
            if (old_occ[i]) {
                insert(std::move(old_buckets[i].key_), std::move(old_buckets[i].value_));
            }
        }
    }

    void erase_at(size_t idx)
    {
        occupied_[idx] = false;
        --size_;

        size_t i = 0;
        size_t next = next_index(idx, i);

        while (occupied_[next]) {
            Bucket b = std::move(buckets_[next]);

            occupied_[next] = false;
            --size_;

            insert(std::move(b.key_), std::move(b.value_));

            next = next_index(next, ++i);
        }
    }

    constexpr size_t next_index(size_t idx, size_t i) const {
        return (idx + 1) & (capacity_ - 1);
        //return (idx + i + 1) & (capacity_ - 1);
    }

    constexpr static size_t next_pow2(size_t x) {
        return (x<=1ull)? 1ull : (1ull << (64 - std::countl_zero(x-1)));
    }
    // constexpr static size_t next_pow2(size_t x) {
    //     size_t p = 1;
    //     while (p < x) {p <<= 1;}
    //     return p;
    // }
};

}
