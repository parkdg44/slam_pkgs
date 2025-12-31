#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <slam_pkg/type/container/slot_map.hpp>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include "utils/slot_map.h"  // dod::slot_map64 baseline

namespace {

using Slam::SlotMap;

TEST(SlotMap, InsertGetContainsBasic) {
  SlotMap<int> sm;

  auto k1 = sm.insert(10);
  auto k2 = sm.emplace(20);

  EXPECT_EQ(sm.size(), 2u);
  EXPECT_TRUE(sm.contains(k1));
  EXPECT_TRUE(sm.contains(k2));

  ASSERT_NE(sm.get(k1), nullptr);
  ASSERT_NE(sm.get(k2), nullptr);
  EXPECT_EQ(*sm.get(k1), 10);
  EXPECT_EQ(*sm.get(k2), 20);

  EXPECT_EQ(sm.at(k1), 10);
}

TEST(SlotMap, EraseInvalidatesKeyAndGetReturnsNull) {
  SlotMap<std::string> sm;

  auto k = sm.emplace("hello");
  EXPECT_TRUE(sm.contains(k));

  EXPECT_TRUE(sm.erase(k));
  EXPECT_FALSE(sm.contains(k));
  EXPECT_EQ(sm.get(k), nullptr);
  EXPECT_EQ(sm.size(), 0u);

  // double erase should fail
  EXPECT_FALSE(sm.erase(k));
}

TEST(SlotMap, ReuseSlotBumpsGenerationSoOldKeyStaysInvalid) {
  SlotMap<int> sm;

  auto k1 = sm.emplace(1);
  EXPECT_TRUE(sm.erase(k1));
  EXPECT_FALSE(sm.contains(k1));

  // likely reuses the freed slot immediately
  auto k2 = sm.emplace(2);

  // if slot reused, index same but generation should differ
  if (k2.index == k1.index) {
    EXPECT_NE(k2.generation, k1.generation);
  }
  EXPECT_FALSE(sm.contains(k1));
  EXPECT_TRUE(sm.contains(k2));
  EXPECT_EQ(sm.at(k2), 2);
}

TEST(SlotMap, SwapAndPopKeepsDenseConsistent) {
  SlotMap<int> sm;

  auto kA = sm.emplace(111);
  auto kB = sm.emplace(222);
  auto kC = sm.emplace(333);

  // erase middle one -> last element should move into its dense slot
  EXPECT_TRUE(sm.erase(kB));
  EXPECT_EQ(sm.size(), 2u);

  EXPECT_TRUE(sm.contains(kA));
  EXPECT_FALSE(sm.contains(kB));
  EXPECT_TRUE(sm.contains(kC));

  // values still accessible via keys
  EXPECT_EQ(sm.at(kA), 111);
  EXPECT_EQ(sm.at(kC), 333);

  // dense storage should contain exactly {111,333} in some order
  std::vector<int> dense(sm.begin(), sm.end());
  ASSERT_EQ(dense.size(), 2u);
  EXPECT_TRUE((dense[0] == 111 && dense[1] == 333) || (dense[0] == 333 && dense[1] == 111));
}

TEST(SlotMap, ClearInvalidatesAllKeysAndEmptiesDense) {
  SlotMap<int> sm;

  auto k1 = sm.emplace(1);
  auto k2 = sm.emplace(2);

  sm.clear();

  EXPECT_TRUE(sm.empty());
  EXPECT_EQ(sm.size(), 0u);
  EXPECT_FALSE(sm.contains(k1));
  EXPECT_FALSE(sm.contains(k2));
  EXPECT_EQ(sm.get(k1), nullptr);
  EXPECT_EQ(sm.get(k2), nullptr);
}

TEST(SlotMap, KeyPackUnpackRoundTrip) {
  using SM = SlotMap<int, std::uint32_t, std::uint32_t>;
  typename SM::Key k{123u, 456u};

  std::uint64_t packed = k.pack();
  auto u = SM::Key::unpack(packed);

  EXPECT_EQ(u.index, k.index);
  EXPECT_EQ(u.generation, k.generation);
}

TEST(SlotMap, ConstAccessorsWork) {
  SlotMap<int> sm;
  auto k = sm.emplace(42);

  const SlotMap<int>& csm = sm;
  EXPECT_TRUE(csm.contains(k));
  EXPECT_EQ(*csm.get(k), 42);
  EXPECT_EQ(csm.at(k), 42);
}

TEST(SlotMap, EmplaceLookupEraseInt) {
  using Clock = std::chrono::steady_clock;
  using ns = std::chrono::nanoseconds;

  const std::size_t N = 200'000;

  SlotMap<std::uint32_t> sm;
  sm.reserve(N);

  std::vector<SlotMap<std::uint32_t>::Key> keys;
  keys.reserve(N);

  // 1) Emplace
  const auto t0 = Clock::now();
  for (std::size_t i = 0; i < N; ++i) keys.push_back(sm.emplace(static_cast<std::uint32_t>(i)));
  const auto t1 = Clock::now();

  // 2) Lookup (sum)
  std::uint64_t sum = 0;
  const auto t2 = Clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    const auto* p = sm.get(keys[i]);
    if (p) sum += *p;
  }
  const auto t3 = Clock::now();

  // 3) Random erase
  std::mt19937 rng(12345);
  std::shuffle(keys.begin(), keys.end(), rng);

  const auto t4 = Clock::now();
  std::size_t erased = 0;
  for (const auto& k : keys) erased += sm.erase(k) ? 1u : 0u;
  const auto t5 = Clock::now();

  EXPECT_EQ(erased, N);
  EXPECT_TRUE(sm.empty());
  // sum sanity: sum_{i=0..N-1} i = (N-1)N/2
  const std::uint64_t expected =
      (N == 0) ? 0ull : (static_cast<std::uint64_t>(N - 1) * static_cast<std::uint64_t>(N)) / 2ull;
  EXPECT_EQ(sum, expected);

  const auto emplace_ns = std::chrono::duration_cast<ns>(t1 - t0).count();
  const auto lookup_ns = std::chrono::duration_cast<ns>(t3 - t2).count();
  const auto erase_ns = std::chrono::duration_cast<ns>(t5 - t4).count();

  std::cout << "\n[SlotMapPerf] N=" << N << " emplace: " << (double)emplace_ns / (double)N
            << " ns/op"
            << " lookup: " << (double)lookup_ns / (double)N << " ns/op"
            << " erase: " << (double)erase_ns / (double)N << " ns/op"
            << "\n";
}

TEST(SlotMap, Throughput_VsDodSlotMap64_Int) {
  using Clock = std::chrono::steady_clock;
  using ns = std::chrono::nanoseconds;

  const std::size_t N = 200'000;

  // --------------------
  // Slam::SlotMap benchmark
  // --------------------
  SlotMap<std::uint32_t> sm;
  sm.reserve(N);

  std::vector<SlotMap<std::uint32_t>::Key> sm_keys;
  sm_keys.reserve(N);

  const auto sm_t0 = Clock::now();
  for (std::size_t i = 0; i < N; ++i) sm_keys.push_back(sm.emplace(static_cast<std::uint32_t>(i)));
  const auto sm_t1 = Clock::now();

  std::uint64_t sm_sum = 0;
  const auto sm_t2 = Clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    const auto* p = sm.get(sm_keys[i]);
    if (p) sm_sum += *p;
  }
  const auto sm_t3 = Clock::now();

  std::mt19937 rng(12345);
  std::shuffle(sm_keys.begin(), sm_keys.end(), rng);

  const auto sm_t4 = Clock::now();
  std::size_t sm_erased = 0;
  for (const auto& k : sm_keys) sm_erased += sm.erase(k) ? 1u : 0u;
  const auto sm_t5 = Clock::now();

  EXPECT_EQ(sm_erased, N);
  EXPECT_TRUE(sm.empty());

  // --------------------
  // dod::slot_map64 baseline
  // --------------------
  dod::slot_map64<std::uint32_t> dsm;

  std::vector<dod::slot_map64<std::uint32_t>::key> dsm_keys;
  dsm_keys.reserve(N);

  const auto d_t0 = Clock::now();
  for (std::size_t i = 0; i < N; ++i)
    dsm_keys.push_back(dsm.emplace(static_cast<std::uint32_t>(i)));
  const auto d_t1 = Clock::now();

  std::uint64_t d_sum = 0;
  const auto d_t2 = Clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    const auto* p = dsm.get(dsm_keys[i]);
    if (p) d_sum += *p;
  }
  const auto d_t3 = Clock::now();

  std::mt19937 rng2(12345);
  std::shuffle(dsm_keys.begin(), dsm_keys.end(), rng2);

  const auto d_t4 = Clock::now();
  for (const auto& k : dsm_keys) dsm.erase(k);  // void-returning API
  const auto d_t5 = Clock::now();

  EXPECT_TRUE(dsm.empty());

  // --------------------
  // Print (ns/op) + ratios
  // --------------------
  const auto sm_emplace_ns = std::chrono::duration_cast<ns>(sm_t1 - sm_t0).count();
  const auto sm_lookup_ns = std::chrono::duration_cast<ns>(sm_t3 - sm_t2).count();
  const auto sm_erase_ns = std::chrono::duration_cast<ns>(sm_t5 - sm_t4).count();

  const auto d_emplace_ns = std::chrono::duration_cast<ns>(d_t1 - d_t0).count();
  const auto d_lookup_ns = std::chrono::duration_cast<ns>(d_t3 - d_t2).count();
  const auto d_erase_ns = std::chrono::duration_cast<ns>(d_t5 - d_t4).count();

  const auto ns_per = [N](auto total_ns) -> double {
    return N ? (double)total_ns / (double)N : 0.0;
  };
  const auto ratio = [](double a, double b) -> double {
    return b > 0.0 ? (a / b) : 0.0;
  };  // a relative to b

  const std::uint64_t expected =
      (N == 0) ? 0ull : (static_cast<std::uint64_t>(N - 1) * static_cast<std::uint64_t>(N)) / 2ull;
  EXPECT_EQ(sm_sum, expected);
  EXPECT_EQ(d_sum, expected);

  std::cout << "\n[SlotMapPerf vs dod::slot_map64] N=" << N
            << "\n  emplace:           Slam::SlotMap " << ns_per(sm_emplace_ns) << " ns/op, dod "
            << ns_per(d_emplace_ns)
            << " ns/op, Slam/dod=" << ratio(ns_per(sm_emplace_ns), ns_per(d_emplace_ns))
            << "\n  lookup(sum):       Slam::SlotMap " << ns_per(sm_lookup_ns) << " ns/op, dod "
            << ns_per(d_lookup_ns)
            << " ns/op, Slam/dod=" << ratio(ns_per(sm_lookup_ns), ns_per(d_lookup_ns))
            << "\n  erase:             Slam::SlotMap " << ns_per(sm_erase_ns) << " ns/op, dod "
            << ns_per(d_erase_ns)
            << " ns/op, Slam/dod=" << ratio(ns_per(sm_erase_ns), ns_per(d_erase_ns)) << "\n";
}

struct HeavyVecU32 {
  std::vector<std::uint32_t> data;

  HeavyVecU32() = default;
  explicit HeavyVecU32(std::size_t n, std::uint32_t seed) : data(n) {
    for (std::size_t i = 0; i < n; ++i) data[i] = seed + static_cast<std::uint32_t>(i);
  }

  std::uint64_t checksum() const { return std::accumulate(data.begin(), data.end(), 0ull); }
};

TEST(SlotMap, HeavyType_VectorPayload_InsertGetErase) {
  constexpr std::size_t PayloadN = 1024;  // >= 1000
  constexpr std::size_t N = 128;

  SlotMap<HeavyVecU32> sm;
  sm.reserve(N);

  std::vector<SlotMap<HeavyVecU32>::Key> keys;
  keys.reserve(N);

  for (std::size_t i = 0; i < N; ++i) {
    keys.push_back(sm.emplace(PayloadN, static_cast<std::uint32_t>(i * 10u)));
  }
  EXPECT_EQ(sm.size(), N);

  // spot-check payload integrity via get/at
  for (std::size_t i = 0; i < N; i += 17) {
    const auto k = keys[i];
    ASSERT_TRUE(sm.contains(k));
    const auto* p = sm.get(k);
    ASSERT_NE(p, nullptr);
    ASSERT_EQ(p->data.size(), PayloadN);
    EXPECT_EQ(p->data.front(), static_cast<std::uint32_t>(i * 10u));
    EXPECT_EQ(p->data.back(), static_cast<std::uint32_t>(i * 10u + (PayloadN - 1)));
    EXPECT_EQ(sm.at(k).checksum(), p->checksum());
  }

  // erase half (deterministic pattern) and ensure invalidation
  std::size_t erased = 0;
  for (std::size_t i = 0; i < N; i += 2) {
    erased += sm.erase(keys[i]) ? 1u : 0u;
    EXPECT_FALSE(sm.contains(keys[i]));
    EXPECT_EQ(sm.get(keys[i]), nullptr);
  }
  EXPECT_EQ(erased, N / 2);
  EXPECT_EQ(sm.size(), N - erased);

  // remaining keys still valid and payload still large
  for (std::size_t i = 1; i < N; i += 2) {
    const auto k = keys[i];
    ASSERT_TRUE(sm.contains(k));
    const auto* p = sm.get(k);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->data.size(), PayloadN);
  }
}

TEST(SlotMap, Throughput_VsDodSlotMap64_HeavyVecU32) {
  using Clock = std::chrono::steady_clock;
  using ns = std::chrono::nanoseconds;

  constexpr std::size_t PayloadN = 1024;  // >= 1000
  constexpr std::size_t N = 5'000;        // keep runtime/memory reasonable

  // --------------------
  // Slam::SlotMap benchmark
  // --------------------
  SlotMap<HeavyVecU32> sm;
  sm.reserve(N);

  std::vector<SlotMap<HeavyVecU32>::Key> sm_keys;
  sm_keys.reserve(N);

  const auto sm_t0 = Clock::now();
  for (std::size_t i = 0; i < N; ++i)
    sm_keys.push_back(sm.emplace(PayloadN, static_cast<std::uint32_t>(i * 10u)));
  const auto sm_t1 = Clock::now();

  std::uint64_t sm_sum = 0;
  const auto sm_t2 = Clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    const auto* p = sm.get(sm_keys[i]);
    if (p && !p->data.empty())
      sm_sum +=
          static_cast<std::uint64_t>(p->data.front()) + static_cast<std::uint64_t>(p->data.back());
  }
  const auto sm_t3 = Clock::now();

  std::mt19937 rng(12345);
  std::shuffle(sm_keys.begin(), sm_keys.end(), rng);

  const auto sm_t4 = Clock::now();
  std::size_t sm_erased = 0;
  for (const auto& k : sm_keys) sm_erased += sm.erase(k) ? 1u : 0u;
  const auto sm_t5 = Clock::now();

  EXPECT_EQ(sm_erased, N);
  EXPECT_TRUE(sm.empty());

  // --------------------
  // dod::slot_map64 baseline
  // --------------------
  dod::slot_map64<HeavyVecU32> dsm;

  std::vector<dod::slot_map64<HeavyVecU32>::key> dsm_keys;
  dsm_keys.reserve(N);

  const auto d_t0 = Clock::now();
  for (std::size_t i = 0; i < N; ++i)
    dsm_keys.push_back(dsm.emplace(HeavyVecU32(PayloadN, static_cast<std::uint32_t>(i * 10u))));
  const auto d_t1 = Clock::now();

  std::uint64_t d_sum = 0;
  const auto d_t2 = Clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    const auto* p = dsm.get(dsm_keys[i]);
    if (p && !p->data.empty())
      d_sum +=
          static_cast<std::uint64_t>(p->data.front()) + static_cast<std::uint64_t>(p->data.back());
  }
  const auto d_t3 = Clock::now();

  std::mt19937 rng2(12345);
  std::shuffle(dsm_keys.begin(), dsm_keys.end(), rng2);

  const auto d_t4 = Clock::now();
  for (const auto& k : dsm_keys) dsm.erase(k);  // void-returning API
  const auto d_t5 = Clock::now();

  EXPECT_TRUE(dsm.empty());

  // --------------------
  // Sanity + Print (ns/op) + ratios
  // --------------------
  const auto sm_emplace_ns = std::chrono::duration_cast<ns>(sm_t1 - sm_t0).count();
  const auto sm_lookup_ns = std::chrono::duration_cast<ns>(sm_t3 - sm_t2).count();
  const auto sm_erase_ns = std::chrono::duration_cast<ns>(sm_t5 - sm_t4).count();

  const auto d_emplace_ns = std::chrono::duration_cast<ns>(d_t1 - d_t0).count();
  const auto d_lookup_ns = std::chrono::duration_cast<ns>(d_t3 - d_t2).count();
  const auto d_erase_ns = std::chrono::duration_cast<ns>(d_t5 - d_t4).count();

  const auto ns_per = [N](auto total_ns) -> double {
    return N ? (double)total_ns / (double)N : 0.0;
  };
  const auto ratio = [](double a, double b) -> double { return b > 0.0 ? (a / b) : 0.0; };

  const std::uint64_t expected =
      10ull * static_cast<std::uint64_t>(N) * static_cast<std::uint64_t>(N - 1) +
      static_cast<std::uint64_t>(N) * static_cast<std::uint64_t>(PayloadN - 1);

  EXPECT_EQ(sm_sum, expected);
  EXPECT_EQ(d_sum, expected);

  std::cout << "\n[SlotMapPerf vs dod::slot_map64] HeavyVecU32"
            << " N=" << N << " PayloadN=" << PayloadN << "\n  emplace:           Slam::SlotMap "
            << ns_per(sm_emplace_ns) << " ns/op, dod " << ns_per(d_emplace_ns)
            << " ns/op, Slam/dod=" << ratio(ns_per(sm_emplace_ns), ns_per(d_emplace_ns))
            << "\n  lookup(front+back):Slam::SlotMap " << ns_per(sm_lookup_ns) << " ns/op, dod "
            << ns_per(d_lookup_ns)
            << " ns/op, Slam/dod=" << ratio(ns_per(sm_lookup_ns), ns_per(d_lookup_ns))
            << "\n  erase:             Slam::SlotMap " << ns_per(sm_erase_ns) << " ns/op, dod "
            << ns_per(d_erase_ns)
            << " ns/op, Slam/dod=" << ratio(ns_per(sm_erase_ns), ns_per(d_erase_ns)) << "\n";
}

}  // namespace
