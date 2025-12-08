#pragma once
#include <vector>

namespace ToLoG
{

/// Small Utility Container which stores pairs of keys/values in a vector.
/// Allows accessing like a map for convenience. Recommended when a map is known to be small.
template<typename K, typename V>
class KeyValueVector
{
public:
    using key_type = K;
    using value_type = V;

    KeyValueVector() {}

    inline void reserve(size_t _size) {
        keyvalues_.reserve(_size);
    }

    inline size_t size() const {
        return keyvalues_.size();
    }

    inline bool empty() const {
        return keyvalues_.empty();
    }

    inline void clear() {
        keyvalues_.clear();
    }

    auto begin() {return keyvalues_.begin();}
    auto end() {return keyvalues_.end();}
    auto begin() const {return keyvalues_.begin();}
    auto end() const {return keyvalues_.end();}

    inline const V& at(const K& _key) const {
        for (const auto& kv : keyvalues_) {
            if (kv.first == _key) {
                return kv.second;
            }
        }
        throw std::runtime_error("Key not found");
    }

    inline const std::pair<K,V>& at_index(const int& _i) const {
        return keyvalues_[_i];
    }

    inline const K& key_at_index(const int& _i) const {
        return keyvalues_[_i].first;
    }

    inline const V& value_at_index(const int& _i) const {
        return keyvalues_[_i].second;
    }

    inline const V& operator[](const K& _key) const {
        return at(_key);
    }

    inline V& operator[](const K& _key) {
        for (auto& kv : keyvalues_) {
            if (kv.first == _key) {
                return kv.second;
            }
        }
        keyvalues_.emplace_back(_key, V());
        return keyvalues_.back().second;
    }

    inline void set(const K& _key, const V& _value) {
        this->operator[](_key) = _value;
    }

private:
    std::vector<std::pair<K,V>> keyvalues_;
};

}
