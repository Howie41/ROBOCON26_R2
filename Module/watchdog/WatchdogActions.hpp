/**
 * @file WatchdogActions.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 预置看门狗回调 — 设标志位、调成员函数
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include "SoftwareWatchdog.hpp"

#include <cstdint>

/* ========== 设标志位 ==================================================== */

/// 超时时将 bool 标志置为 true（调用方负责轮询该标志并清除）
inline void actionSetFlag(void *ctx) {
  if (ctx) {
    *static_cast<bool *>(ctx) = true;
  }
}

/// 超时时递增计数器,计数单位就是timeout
inline void actionIncrementCounter(void *ctx) {
  if (ctx) {
    (*static_cast<uint32_t *>(ctx))++;
  }
}

/* ========== 调用对象成员函数 ============================================ */

/**
 * @brief 生成一个回调，调用 T::method()
 *
 * @usage
 *   Motor motor;
 *   auto cb = actionCallMember<Motor, &Motor::emergencyStop>(&motor);
 *   SoftwareWatchdog<HALClock> wd{100, cb};
 */
template <typename T, void (T::*Method)()>
inline WatchdogAction actionCallMember(T *obj) {
  return WatchdogAction{[](void *ctx) { (static_cast<T *>(ctx)->*Method)(); },
                        static_cast<void *>(obj)};
}