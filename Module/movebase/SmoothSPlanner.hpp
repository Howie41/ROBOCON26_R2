/**
 * @file SmoothSPlanner.hpp
 * @author FunFer
 * @brief Smooth S-curve速度规划器
 * @date 2026-6-23
 *
 * @copyright Copyright (c) 2025
 
 * @note 该规划器中x指代电机位置的多圈累加绝对量，且x,v,a,t单位制统一
 * @note 今后将会考虑增加：紧急刹停、中断规划、重规划等接口
 */

#pragma once

#include <cmath>
#include <limits>
#include <optional>
#include <limits>
#include "bsp_dwt.h"


#define EPSILON 1e-4

inline bool isZero(float x) { return fabsf(x) < EPSILON; }

class MotionState {
public:
    // x可正可负，a和v取绝对值
    MotionState(float x = 0.0f, float v = 0.0f, float a = 0.0f) :
        x_(x), v_(v), a_(a) {}
    ~MotionState() {}

    void set_x(float x) { x_ = x; }
    void add_x(float dx) { x_ += dx; }
    const inline float get_x() { return x_; }

    void set_v(float v) { v_ = v; }
    void add_v(float dv) { v_ += dv; }
    const inline float get_v() { return v_; }

    void set_a(float a) { a_ = a; }
    void add_a(float da) { a_ += da; }
    const inline float get_a() { return a_; }

    void set_params(std::optional<float> x = std::nullopt, std::optional<float> v = std::nullopt, std::optional<float> a = std::nullopt) {
        if (x.has_value()) set_x(x.value());
        if (v.has_value()) set_v(v.value());
        if (a.has_value()) set_a(a.value());
    }

private:
    float x_;
    float v_;
    float a_;
};

class SmoothSPlanner {  // 该规划器下，急动度最大值: j = π/2 * a²/v，且j自动平滑变化
public:
    SmoothSPlanner() = default;
    SmoothSPlanner(float x, float v = 0.0f, float a = 0.0f) {
        cur_state_.set_params(x, v, a);
        cur_t_ = DWT_GetTimeline_s();
    }
    ~SmoothSPlanner() {}

    // 缓冲段v-t曲线
    inline float smooth_curve(float t) {
        return tar_state_.get_a() / 2 * t - tar_state_.get_v() / (2 * M_PI) * sin(tar_state_.get_a() / tar_state_.get_v() * M_PI * t);
    }
    // 缓冲段x-t曲线
    inline float integral_smooth_curve(float t) {
        return tar_state_.get_a() / 4 * t * t + tar_state_.get_v() * tar_state_.get_v() / (4 * M_PI * M_PI * tar_state_.get_a()) * cos(tar_state_.get_a() / tar_state_.get_v() * M_PI * t);
    }

    /**
     * @brief 以指定参数开始速度规划。此处未加is_planning_判断！（主要用于始规划，一般不可用于中断规划/重规划）
     * @param t 目标时间（所需时间）
     * @param x 目标位置
     * @param v 最大速度（绝对值）；默认std::nullopt，传入std::nullopt表示不限制
     * @param a 最大加速度（绝对值）；默认std::nullopt，传入std::nullopt表示不限制
     * @return true 规划成功，开始速度规划
     * @return false 指定参数下无解，规划失败
     */
    bool init(float t, float x, std::optional<float> v = std::nullopt, std::optional<float> a = std::nullopt) {
        if (t <= 0.0f || isZero(x) || v.value() <= 0.0f || a.value() <= 0.0f) return false;  // 判断参数合理性: x可正可负，a和v取绝对值
        if (!v.has_value()) v.value() = std::numeric_limits<float>::max();
        if (!a.has_value()) a.value() = std::numeric_limits<float>::max();
        // 此处共有两个约束：x = v(t - 2v/a) 且 2v/a < t/2，进行数学建模
        float p = fabsf(x - cur_state_.get_x());  // 距离差
        if (v.value() >= 2 * p / t) {  // 区间1
            float a_temp = 8 * p / (t * t);
            if (a.value() >= a_temp) a.value() = a_temp;
            else return false;  // 无解
        } else if (v.value() > p / t) {  // 区间2
            float a_temp = 2.0f * v.value() * v.value() / (t * v.value() - p);
            if (a.value() >= a_temp) a.value() = a_temp;
            else return false;  // 无解
        } else return false;  // 区间3，无解

        // 有解，开始规划
        ini_state_.set_x(cur_state_.get_x());
        tar_state_.set_params(x, v.value(), a.value());
        ini_t_ = DWT_GetTimeline_s();
        tar_t_ = ini_t_ + t;

        cur_state_.set_params(std::nullopt, 0.0f, 0.0f);
        cur_t_ = DWT_GetTimeline_s();
        
        T_buf_ = 2 * tar_state_.get_v() / tar_state_.get_a();
        T_vel_ = t - 2 * T_buf_;
        process_ = 0.0f;
        is_towards_positive_ = x > cur_state_.get_x() ? 1 : -1;
        is_planning_ = true;

        return true;
    }

    /**
     * @brief 根据当前是否处于规划，更新当前运动学状态 或 锁角
     * @param tar_x 追踪位置引用（规划器输出）
     * @param tar_v 追踪速度引用（规划器输出）
     */
    void update(float &tar_x, float &tar_v) {
        cur_t_ = DWT_GetTimeline_s();
        if (is_planning_) {  // 进行速度规划
            if (process_ >= 1.0f) {  // 结束
                process_ = 1.0f;
                cur_state_.set_params(tar_state_.get_x(), 0.0f, 0.0f);
                is_planning_ = false;
            } else {  // 未结束
                float t = cur_t_ - ini_t_;
                process_ = t / (tar_t_ - ini_t_);
                // 缓冲段1
                if (t < T_buf_) cur_state_.set_params((is_towards_positive_ > 0 ? integral_smooth_curve(t) : -integral_smooth_curve(t)) + ini_state_.get_x(), smooth_curve(t), 0.0f);
                // 缓冲段2
                else if (t < T_buf_ + T_vel_) cur_state_.set_params((is_towards_positive_ > 0 ? (tar_state_.get_v() * tar_state_.get_v() / tar_state_.get_a() + tar_state_.get_v() * (t - T_buf_)) : -(tar_state_.get_v() * tar_state_.get_v() / tar_state_.get_a() + tar_state_.get_v() * (t - T_buf_))) + ini_state_.get_x(), tar_state_.get_v(), 0.0f);
                // 缓冲段3
                else cur_state_.set_params(tar_state_.get_x() + (is_towards_positive_ > 0 ? -integral_smooth_curve(tar_t_ - cur_t_) : integral_smooth_curve(tar_t_ - cur_t_)), smooth_curve(tar_t_ - cur_t_), 0.0f);
            }
            // 输出
            tar_x = cur_state_.get_x();
            tar_v = is_towards_positive_ > 0 ? cur_state_.get_v() : -cur_state_.get_v();
        } else {  // 锁角
            tar_x = cur_state_.get_x();
            tar_v = 0.0f;
        }
    }

    const inline bool is_finished() { return !is_planning_; }

    // 属性的getter
    const inline bool get_is_planning() { return is_planning_; }
    const inline float get_process() { return process_; }
    const inline MotionState get_cur_state() { return cur_state_; }

private:
    MotionState ini_state_{0.0f, 0.0f, 0.0f};  // 初始运动学状态
    MotionState tar_state_{0.0f, 0.0f, 0.0f};  // 目标运动学状态
    float ini_t_{0.0f}, tar_t_{0.0f};  // 初始时刻，目标时刻

    MotionState cur_state_{0.0f, 0.0f, 0.0f};  // 当前运动学状态
    float cur_t_{0.0f};  // 当前时刻

    float T_buf_{0.0f}, T_vel_{0.0f};  // 缓冲时间段，匀速时间段
    float process_{0.0f};  // 规划进度: 0% -> 100%
    int8_t is_towards_positive_{1};  // 当前移动方向: 1 -> 正向；-1 -> 反向
    bool is_planning_{false};  // 当前是否完成规划进度

};
