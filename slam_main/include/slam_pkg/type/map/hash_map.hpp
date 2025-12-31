#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "ankerl/unordered_dense.h"

namespace Slam::Morton {

struct Key {
  int x;
  int y;
  int z;

  bool operator==(const Key& other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

inline uint64_t part1by2(uint32_t n) {
  if ((n >> 21) != 0) {
    throw std::out_of_range("part1by2: input exceeds 21 bits");
  }

  uint64_t x = n & 0x1fffff;
  x = (x | x << 32) & 0x1f00000000ffff;
  x = (x | x << 16) & 0x1f0000ff0000ff;
  x = (x | x << 8) & 0x100f00f00f00f00f;
  x = (x | x << 4) & 0x10c30c30c30c30c3;
  x = (x | x << 2) & 0x1249249249249249;
  return x;
}

inline uint64_t encode(int x, int y, int z) {
  const uint32_t offset = 1U << 20;
  uint32_t ux = x + offset;
  uint32_t uy = y + offset;
  uint32_t uz = z + offset;

  return part1by2(ux) | (part1by2(uy) << 1) | (part1by2(uz) << 2);
}

inline Key decode(uint64_t code) {
  static auto compact1by2 = [](uint64_t n) -> uint32_t {
    n &= 0x1249249249249249;
    n = (n ^ (n >> 2)) & 0x10c30c30c30c30c3;
    n = (n ^ (n >> 4)) & 0x100f00f00f00f00f;
    n = (n ^ (n >> 8)) & 0x1f0000ff0000ff;
    n = (n ^ (n >> 16)) & 0x1f00000000ffff;
    n = (n ^ (n >> 32)) & 0x1fffff;
    return static_cast<uint32_t>(n);
  };

  const uint32_t offset = 1U << 20;
  int x = static_cast<int>(compact1by2(code) - offset);
  int y = static_cast<int>(compact1by2(code >> 1) - offset);
  int z = static_cast<int>(compact1by2(code >> 2) - offset);
  return Key{x, y, z};
}

template <typename T>
class HashMap {
 public:
  using code_type = std::uint64_t;
  using map_type = ankerl::unordered_dense::map<code_type, T>;

  class xyz_iterator {
    typename map_type::iterator it_{};

   public:
    using difference_type = std::ptrdiff_t;
    using iterator_category = std::forward_iterator_tag;

    xyz_iterator() = default;
    explicit xyz_iterator(typename map_type::iterator it) : it_(it) {}

    // accessors (decode on demand)
    Key key() const { return decode(it_->first); }
    T& value() const { return it_->second; }  // iterator itself models mutable access

    // Make range-for natural: *it gives a lightweight handle (this).
    xyz_iterator& operator*() noexcept { return *this; }
    const xyz_iterator& operator*() const noexcept { return *this; }
    xyz_iterator* operator->() noexcept { return this; }
    const xyz_iterator* operator->() const noexcept { return this; }

    xyz_iterator& operator++() {
      ++it_;
      return *this;
    }
    xyz_iterator operator++(int) {
      xyz_iterator tmp(*this);
      ++(*this);
      return tmp;
    }

    friend bool operator==(const xyz_iterator& a, const xyz_iterator& b) noexcept {
      return a.it_ == b.it_;
    }
    friend bool operator!=(const xyz_iterator& a, const xyz_iterator& b) noexcept {
      return !(a == b);
    }

    // internal interop
    typename map_type::iterator base() const noexcept { return it_; }
  };

  class const_xyz_iterator {
    typename map_type::const_iterator it_{};

   public:
    using difference_type = std::ptrdiff_t;
    using iterator_category = std::forward_iterator_tag;

    const_xyz_iterator() = default;
    explicit const_xyz_iterator(typename map_type::const_iterator it) : it_(it) {}
    // allow implicit conversion from mutable iterator
    const_xyz_iterator(xyz_iterator it) : it_(it.base()) {}

    Key key() const { return decode(it_->first); }
    const T& value() const { return it_->second; }

    const const_xyz_iterator& operator*() const noexcept { return *this; }
    const const_xyz_iterator* operator->() const noexcept { return this; }

    const_xyz_iterator& operator++() {
      ++it_;
      return *this;
    }
    const_xyz_iterator operator++(int) {
      const_xyz_iterator tmp(*this);
      ++(*this);
      return tmp;
    }

    friend bool operator==(const const_xyz_iterator& a, const const_xyz_iterator& b) noexcept {
      return a.it_ == b.it_;
    }
    friend bool operator!=(const const_xyz_iterator& a, const const_xyz_iterator& b) noexcept {
      return !(a == b);
    }

    typename map_type::const_iterator base() const noexcept { return it_; }
  };

  using iterator = xyz_iterator;
  using const_iterator = const_xyz_iterator;
  using value_type = typename map_type::value_type;

  HashMap() = default;

  // Capacity
  std::size_t size() const noexcept { return hash_map_.size(); }
  bool empty() const noexcept { return hash_map_.empty(); }
  void clear() noexcept { hash_map_.clear(); }
  void reserve(std::size_t n) { hash_map_.reserve(n); }

  // Iteration (decode happens when you call it.key())
  xyz_iterator begin() noexcept { return xyz_iterator{hash_map_.begin()}; }
  xyz_iterator end() noexcept { return xyz_iterator{hash_map_.end()}; }
  const_xyz_iterator begin() const noexcept { return const_xyz_iterator{hash_map_.begin()}; }
  const_xyz_iterator end() const noexcept { return const_xyz_iterator{hash_map_.end()}; }
  const_xyz_iterator cbegin() const noexcept { return const_xyz_iterator{hash_map_.cbegin()}; }
  const_xyz_iterator cend() const noexcept { return const_xyz_iterator{hash_map_.cend()}; }

  // Key helpers
  static code_type code(int x, int y, int z) { return encode(x, y, z); }
  static code_type code(Key k) { return encode(k.x, k.y, k.z); }

  // Lookup
  bool contains(int x, int y, int z) const { return contains(code(x, y, z)); }
  bool contains(Key k) const { return contains(code(k)); }
  bool contains(code_type c) const { return hash_map_.find(c) != hash_map_.end(); }

  xyz_iterator find(int x, int y, int z) { return xyz_iterator{hash_map_.find(code(x, y, z))}; }
  xyz_iterator find(Key k) { return xyz_iterator{hash_map_.find(code(k))}; }
  xyz_iterator find(code_type c) { return xyz_iterator{hash_map_.find(c)}; }

  const_xyz_iterator find(int x, int y, int z) const {
    return const_xyz_iterator{hash_map_.find(code(x, y, z))};
  }
  const_xyz_iterator find(Key k) const { return const_xyz_iterator{hash_map_.find(code(k))}; }
  const_xyz_iterator find(code_type c) const { return const_xyz_iterator{hash_map_.find(c)}; }

  T& at(int x, int y, int z) { return at(code(x, y, z)); }
  T& at(Key k) { return at(code(k)); }
  T& at(code_type c) {
    auto it = hash_map_.find(c);
    if (it == hash_map_.end()) throw std::out_of_range("Morton::HashMap::at: key not found");
    return it->second;
  }

  const T& at(int x, int y, int z) const { return at(code(x, y, z)); }
  const T& at(Key k) const { return at(code(k)); }
  const T& at(code_type c) const {
    auto it = hash_map_.find(c);
    if (it == hash_map_.end()) throw std::out_of_range("Morton::HashMap::at: key not found");
    return it->second;
  }

  // Compatibility: previous get() returned by value
  T get(int x, int y, int z) const { return at(x, y, z); }
  T get(Key k) const { return at(k); }

  // Insertion / update
  // - operator[]: default-construct if absent
  T& operator[](Key k) { return hash_map_[code(k)]; }
  T& operator[](code_type c) { return hash_map_[c]; }

  // - try_emplace: only constructs value if key absent
  template <class... Args>
  std::pair<iterator, bool> try_emplace(int x, int y, int z, Args&&... args) {
    auto [it, ok] = hash_map_.try_emplace(code(x, y, z), std::forward<Args>(args)...);
    return {iterator{it}, ok};
  }
  template <class... Args>
  std::pair<iterator, bool> try_emplace(Key k, Args&&... args) {
    auto [it, ok] = hash_map_.try_emplace(code(k), std::forward<Args>(args)...);
    return {iterator{it}, ok};
  }
  template <class... Args>
  std::pair<iterator, bool> try_emplace(code_type c, Args&&... args) {
    auto [it, ok] = hash_map_.try_emplace(c, std::forward<Args>(args)...);
    return {iterator{it}, ok};
  }

  // - insert_or_assign
  std::pair<iterator, bool> insert_or_assign(int x, int y, int z, T value) {
    auto [it, ok] = hash_map_.insert_or_assign(code(x, y, z), std::move(value));
    return {iterator{it}, ok};
  }
  std::pair<iterator, bool> insert_or_assign(Key k, T value) {
    auto [it, ok] = hash_map_.insert_or_assign(code(k), std::move(value));
    return {iterator{it}, ok};
  }
  std::pair<iterator, bool> insert_or_assign(code_type c, T value) {
    auto [it, ok] = hash_map_.insert_or_assign(c, std::move(value));
    return {iterator{it}, ok};
  }

  // Backward-compatible insert() (assign semantics like your original)
  void insert(int x, int y, int z, T value) { (void)insert_or_assign(x, y, z, std::move(value)); }
  void insert(Key key, T value) { (void)insert_or_assign(key, std::move(value)); }

  // Erase
  bool erase(int x, int y, int z) { return erase(code(x, y, z)); }
  bool erase(Key k) { return erase(code(k)); }
  bool erase(code_type c) { return hash_map_.erase(c) != 0; }

 private:
  map_type hash_map_;
};

}  // namespace Slam::Morton