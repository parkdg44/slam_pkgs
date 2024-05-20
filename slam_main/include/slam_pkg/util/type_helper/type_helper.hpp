//
// Created by park on 24. 4. 27.
//

#pragma once

namespace Slam::util {

#define HAS_MEMBER(T, EXPR) \
  has_member_impl<T>([](auto&& obj) -> decltype(obj.EXPR) { return obj.EXPR; })

template <typename... T, typename F>
constexpr auto has_member_impl(F&& f) -> decltype(f((std::declval<T>(), ...)), true) {
  return true;
}

template <typename>
constexpr bool has_member_impl(...) {
  return false;
}

}  // namespace Slam::util