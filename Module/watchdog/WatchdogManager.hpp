/**
 * @file WatchdogManager.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 看门狗集中管理器 — 侵入式链表，统一轮询，零堆分配
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 * 用法：
 *   WatchdogManager<8> mgr;           // 最多 8 个看门狗
 *   mgr.registerWd(&motor_wd);
 *   mgr.registerWd(&sensor_wd);
 *   // 在主循环或定时器 ISR 中：
 *   mgr.poll();
 */

#pragma once

#include "SoftwareWatchdog.hpp"

#include <cstdint>

/**
 * @brief 看门狗集中管理器
 *
 * @tparam MaxWatchdogs  最大管理的看门狗数量（编译期常量，零堆分配）
 *
 * 所有看门狗通过侵入式单链表串联，poll() 时逐个检查。
 * 可放在 main() while 循环、RTOS 任务、或硬件定时器 ISR 中调用。
 */
template <uint8_t MaxWatchdogs> class WatchdogManager {
public:
  WatchdogManager() = default;
  ~WatchdogManager() = default;

  /**
   * @brief 注册看门狗
   *
   * @param wd
   * @return true 成功，false 已满或已注册
   */
  bool registerWd(IWatchdogNode *wd) {
    if (count_ >= MaxWatchdogs || wd == nullptr) {
      return false;
    }
    // 检查是否已在链表中
    if (find(wd) >= 0) {
      return false;
    }
    wd->next_ = head_;
    head_ = wd;
    count_++;
    return true;
  }

  /// 注销看门狗
  bool unregisterWd(IWatchdogNode *wd) {
    if (head_ == nullptr || wd == nullptr) {
      return false;
    }

    IWatchdogNode **indirect = &head_;
    while (*indirect != nullptr) {
      if (*indirect == wd) {
        *indirect = wd->next_;
        wd->next_ = nullptr;
        count_--;
        return true;
      }
      indirect = &(*indirect)->next_;
    }
    return false;
  }

  /// 轮询所有已注册看门狗
  void poll() {
    IWatchdogNode *wd = head_;
    while (wd != nullptr) {
      wd->poll();
      wd = wd->next_;
    }
  }

  /// 统计：存活 / 总数
  uint8_t aliveCount() const {
    uint8_t n = 0;
    IWatchdogNode *wd = head_;
    while (wd != nullptr) {
      if (wd->isAlive())
        n++;
      wd = wd->next_;
    }
    return n;
  }

  uint8_t totalCount() const { return count_; }
  uint8_t maxCount() const { return MaxWatchdogs; }

  /// 是否全部存活
  bool allAlive() const { return aliveCount() == count_; }

private:
  int8_t find(IWatchdogNode *target) const {
    int8_t idx = 0;
    IWatchdogNode *wd = head_;
    while (wd != nullptr) {
      if (wd == target)
        return idx;
      wd = wd->next_;
      idx++;
    }
    return -1;
  }

  IWatchdogNode *head_{nullptr};
  uint8_t count_{0};
};