#pragma once

#include <cassert>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

namespace Slam {

template <class T, class IndexT = std::uint32_t, class GenerationT = std::uint32_t>
class SlotMap {
 public:
  static_assert(std::is_unsigned_v<IndexT>, "IndexT must be unsigned.");
  static_assert(std::is_unsigned_v<GenerationT>, "GenerationT must be unsigned.");

  struct Key {
    IndexT index{};
    GenerationT generation{};

    // 64비트 변환 시 데이터 손실 방지를 위한 assert
    static_assert(sizeof(IndexT) <= 4 && sizeof(GenerationT) <= 4,
                  "Key packing currently assumes 32-bit (or less) types.");

    friend bool operator==(const Key& a, const Key& b) noexcept {
      return a.index == b.index && a.generation == b.generation;
    }
    friend bool operator!=(const Key& a, const Key& b) noexcept { return !(a == b); }

    std::uint64_t pack() const noexcept {
      return (std::uint64_t(generation) << 32) | std::uint64_t(index);
    }
    static Key unpack(std::uint64_t v) noexcept {
      return Key{IndexT(v & 0xFFFF'FFFFu), GenerationT((v >> 32) & 0xFFFF'FFFFu)};
    }
  };

 private:
  static constexpr IndexT npos() noexcept { return std::numeric_limits<IndexT>::max(); }

  struct Slot {
    GenerationT generation{0};
    IndexT dense_index{npos()};         // npos()면 dead로 취급 (alive bool 제거)
  };

  std::vector<T> m_data;                // dense storage (Iterate 빠름)
  std::vector<Slot> m_slots;            // stable handle
  std::vector<IndexT> m_dense_to_slot;  // dense -> slot 역참조
  std::vector<IndexT> m_free;           // 빈 슬롯 관리

 public:
  SlotMap() = default;

  void reserve(std::size_t n) {
    m_data.reserve(n);
    m_slots.reserve(n);
    m_dense_to_slot.reserve(n);
    m_free.reserve(n);
  }

  std::size_t size() const noexcept { return m_data.size(); }
  bool empty() const noexcept { return m_data.empty(); }

  void clear() {
    m_data.clear();
    m_dense_to_slot.clear();
    m_free.clear();

    for (IndexT i = 0; i < IndexT(m_slots.size()); ++i) {
      Slot& s = m_slots[i];
      if (s.dense_index != npos()) {  // 살아있던 것만 세대 증가
        s.dense_index = npos();
        ++s.generation;
      }
      m_free.push_back(i);
    }
  }

  // Accessors
  T* data() noexcept { return m_data.data(); }
  const T* data() const noexcept { return m_data.data(); }
  auto begin() noexcept { return m_data.begin(); }
  auto end() noexcept { return m_data.end(); }
  auto begin() const noexcept { return m_data.begin(); }
  auto end() const noexcept { return m_data.end(); }
  auto cbegin() const noexcept { return m_data.cbegin(); }
  auto cend() const noexcept { return m_data.cend(); }

  // Insert / Emplace
  Key insert(const T& value) { return emplace(value); }
  Key insert(T&& value) { return emplace(std::move(value)); }

  template <class... Args>
  Key emplace(Args&&... args) {
    IndexT slot_idx;
    if (!m_free.empty()) {
      slot_idx = m_free.back();
      m_free.pop_back();
    } else {
      slot_idx = IndexT(m_slots.size());
      m_slots.push_back(Slot{});  // 초기 generation은 0
    }

    Slot& s = m_slots[slot_idx];

    const IndexT dense_idx = IndexT(m_data.size());
    m_data.emplace_back(std::forward<Args>(args)...);
    m_dense_to_slot.push_back(slot_idx);

    s.dense_index = dense_idx;
    return Key{slot_idx, s.generation};
  }

  // Validation
  bool contains(Key k) const noexcept {
    if (k.index >= m_slots.size()) return false;
    const Slot& s = m_slots[k.index];
    return (s.dense_index != npos()) && (s.generation == k.generation);
  }

  // Lookup
  T* get(Key k) noexcept {
    if (!contains(k)) return nullptr;
    return &m_data[m_slots[k.index].dense_index];
  }
  const T* get(Key k) const noexcept {
    if (!contains(k)) return nullptr;
    return &m_data[m_slots[k.index].dense_index];
  }

  // operator[]는 위험할 수 있으므로 at() 권장
  T& at(Key k) {
    T* p = get(k);
    assert(p && "SlotMap::at: invalid key");
    return *p;
  }

  const T& at(Key k) const {
    const T* p = get(k);
    assert(p && "SlotMap::at: invalid key");
    return *p;
  }

  // Erase (Swap and Pop)
  bool erase(Key k) {
    if (!contains(k)) return false;

    const IndexT slot_idx = k.index;
    const IndexT dense_idx = m_slots[slot_idx].dense_index;
    const IndexT last_dense = IndexT(m_data.size() - 1);

    // 1. Data 배열에서 삭제할 놈을 맨 끝 놈과 교체
    if (dense_idx != last_dense) {
      m_data[dense_idx] = std::move(m_data[last_dense]);

      // 끝에 있던 놈의 슬롯 정보 갱신
      const IndexT moved_slot = m_dense_to_slot[last_dense];
      m_dense_to_slot[dense_idx] = moved_slot;
      m_slots[moved_slot].dense_index = dense_idx;
    }

    // 2. 물리적 삭제
    m_data.pop_back();
    m_dense_to_slot.pop_back();

    // 3. 슬롯 무효화 (여기서 generation 증가!)
    Slot& s = m_slots[slot_idx];
    s.dense_index = npos();
    ++s.generation;  // <--- 핵심: 삭제될 때 세대 교체
    m_free.push_back(slot_idx);

    return true;
  }
};

}  // namespace Slam