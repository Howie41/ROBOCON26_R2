/**
 * @file SoftwareWatchdog.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 轻量级软件看门狗 — 零堆分配，可嵌入任意外设模块
 * @version 0.1
 * @date 2026-06-08
 *
 * @copyright Copyright (c) 2026
 *
 * 设计要点：
 * - TimeSource：模板参数注入时间源，不绑定 HAL。要求 static uint32_t now()
 * - Action：默认为 fn(ctx) 回调，也可传入无捕获 lambda 或仿函数
 * - IWatchdogNode：非模板虚基类，供 WatchdogManager 统一轮询
 * - 状态机：IDLE → ARMED → TRIGGERED，支持 ONE_SHOT / AUTO_REARM / LATCHED
 * 三种模式
 * - 防抖：连续 debounce 次超时才触发，避免偶发丢帧误报
 */

#pragma once

#include <cstdint>
#include <array>

// 状态机
enum class WatchdogState : uint8_t {
  IDLE = 0, // 未启用
  ARMED,    // 已启用，等待喂狗
  TRIGGERED // 已触发超时
};

// 三种状态
enum class WatchdogMode : uint8_t {
  ONE_SHOT,   // 触发后停在 TRIGGERED，等待手动重装
  AUTO_REARM, // 触发后执行回调并立刻重新 ARMED
  LATCHED     // 触发后锁存为 TRIGGERED，feed() 不会恢复，必须 disarm→arm
};

/// 通用回调：函数指针 + 上下文指针（可捕获状态，零堆分配）
struct WatchdogAction {
    using Fn = void (*)(void *);

    // 函数指针
    Fn fn{nullptr};
    // 上下文指针
    void *ctx{nullptr};

    // 调用运算符，允许把对象像函数一样调用来触发回调
    void operator()() const {
        if (fn) {
            fn(ctx);
        }
    }

    explicit operator bool() const { return fn != nullptr; }
};

// 链表节点，基类
class IWatchdogNode {
public:
  virtual ~IWatchdogNode() = default;

  /// 周期调用，评估是否超时
  virtual void poll() = 0;

  /// 返回当前是否存活（未触发）
  virtual bool isAlive() const = 0;

  /// 侵入式链表指针（由 Manager 维护）
  IWatchdogNode *next_{nullptr};
};

/**
 * @brief 软件看门狗
 *
 * @tparam TimeSource  时间源，要求 static uint32_t now() 返回单调 ms 时间戳
 * @tparam Action      回调类型，默认 WatchdogAction(fn+ctx)，需支持
 * operator()()
 *
 * @usage
 *   // 1. 定义时间源
 *   struct HALClock { static uint32_t now() { return HAL_GetTick(); } };
 *
 *   // 2. 创建看门狗（50ms 超时，防抖 2 次，自动重武装）
 *   SoftwareWatchdog<HALClock> wd{50, {onTimeout, this},
 * WatchdogMode::AUTO_REARM, 2};
 *
 *   // 3. 在数据到达路径喂狗
 *   wd.feed();
 *
 *   // 4. 在主循环 / 定时器 ISR / Manager 中轮询
 *   wd.poll();  // 或 manager.poll()
 */
template <typename TimeSource, typename Action = WatchdogAction>
class SoftwareWatchdog : public IWatchdogNode {
public:
    SoftwareWatchdog() = default;
    /**
    * @brief Construct a new Software Watchdog object
    *
    * @param timeout_ms  超时阈值 (ms)
    * @param action      超时回调
    * @param mode        触发模式（默认自动重武装）
    * @param debounce    防抖次数（默认 1，即首次超时就触发）
    */
    SoftwareWatchdog(uint32_t timeout_ms, Action action, WatchdogMode mode = WatchdogMode::AUTO_REARM, uint32_t debounce = 1) :
        timeout_ms_(timeout_ms), actions_({action}), actions_count_(1), mode_(mode), debounce_(debounce > 0 ? debounce : 1) {}

    // 喂狗 — 在数据到达路径中调用
    void feed() {
        last_feed_time_ = TimeSource::now();
        miss_count_ = 0;
        // 如果超时已触发，并且非LATCHED模式，即为AUTO_REARM/ONE_SHOT
        if (state_ == WatchdogState::TRIGGERED && mode_ != WatchdogMode::LATCHED) {
            // 再次feed可以重恢复看门狗状态
            state_ = WatchdogState::ARMED;
        }
    }

    // 启用看门狗
    void arm() {
        last_feed_time_ = TimeSource::now();
        miss_count_ = 0;
        state_ = WatchdogState::ARMED;
    }

    // 停用看门狗
    void disarm() { state_ = WatchdogState::IDLE; }

    // 复位看门狗（disarm + arm）
    void reset() {
        disarm();
        arm();
    }

    // 周期轮询 — 评估是否超时，若超时则执行回调
    void poll() override {
        if (state_ != WatchdogState::ARMED) return;

        const uint32_t now = TimeSource::now();
        if (now - last_feed_time_ < timeout_ms_) return;

        // 超时窗口满足，累计防抖计数
        if (++miss_count_ < debounce_) return;

        // 确认超时
        state_ = WatchdogState::TRIGGERED;
        miss_count_ = debounce_; // 防止溢出

        // 执行看门回调动作
        for (uint8_t i = 0; i < actions_count_; ++i) actions_[i]();

        // 自动复位
        if (mode_ == WatchdogMode::AUTO_REARM) {
            last_feed_time_ = now;
            miss_count_ = 0;
            state_ = WatchdogState::ARMED;
        }
    }

    /* ---------- 状态查询 -------------------------------------------------- */

    bool isAlive() const override { return state_ != WatchdogState::TRIGGERED; }
    bool isArmed() const { return state_ == WatchdogState::ARMED; }
    bool isIdle() const { return state_ == WatchdogState::IDLE; }
    WatchdogState state() const { return state_; }
    WatchdogMode mode() const { return mode_; }

    /// 距上次喂狗已过去多少 ms（仅在 ARMED 状态有意义）
    uint32_t elapsed() const {
        if (state_ == WatchdogState::IDLE) return 0;
        return TimeSource::now() - last_feed_time_;
    }

    uint32_t missCount() const { return miss_count_; }
    uint32_t debounce() const { return debounce_; }

    /* ---------- 参数配置 -------------------------------------------------- */

    void setTimeout(uint32_t ms) { timeout_ms_ = ms; }
    void setDebounce(uint32_t count) { debounce_ = count > 0 ? count : 1; }
    void setMode(WatchdogMode mode) { mode_ = mode; }
    void addAction(Action action) {
        if (actions_count_ < actions_.size()) {
            actions_[actions_count_++] = action;
        }
    }

private:
    uint32_t timeout_ms_{100};
    std::array<Action, 8> actions_{};
    uint8_t actions_count_{0};
    WatchdogMode mode_{WatchdogMode::AUTO_REARM};
    uint32_t debounce_{1};

    uint32_t last_feed_time_{0};
    uint32_t miss_count_{0};
    WatchdogState state_{WatchdogState::IDLE};

};
