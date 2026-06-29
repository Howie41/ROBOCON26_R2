/**
 * @file SmoothSPlanner.hpp
 * @author FunFer
 * @brief Smooth S-curve速度规划器
 * @date 2026-6-23
 *
 * @copyright Copyright (c) 2025
 
 * @note 该规划器中x指代电机位置的多圈累加绝对量，且x,v,t单位制统一
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
    MotionState(float x = 0.0f, float v = 0.0f) : x_(x), v_(v) {}
    ~MotionState() {}

    void add_x(float dx) { x_ += dx; }
    inline float& get_x() { return x_; }

    void add_v(float dv) { v_ += dv; }
    inline float& get_v() { return v_; }

    void set_params(std::optional<float> x = std::nullopt, std::optional<float> v = std::nullopt) {
        if (x.has_value()) x_ = x.value();
        if (v.has_value()) v_ = v.value();
    }

private:
    float x_;
    float v_;
};

class SmoothSPlanner {  // 该规划器下，急动度最大值: j = π/2 * a²/v，且j自动平滑变化
public:
    SmoothSPlanner() = default;
    SmoothSPlanner(float x, float v = 0.0f) {
        cur_state_.set_params(x, v);
        cur_t_ = DWT_GetTimeline_s();
    }
    ~SmoothSPlanner() {}

    // 缓冲段v-t曲线
    inline float smooth_curve(float t) {
        return tar_state_.get_x() / tar_t_ * (1.0f - cos(2 * M_PI * t / tar_t_));
    }
    // 缓冲段x-t曲线
    inline float integral_smooth_curve(float t) {
        float u = M_PI * t / tar_t_;
        return tar_state_.get_x() * 0.5f / M_PI * (u + sin(2.0f * u));
    }

    // 开始规划
    void init(float t, float x) {
        ini_t_ = DWT_GetTimeline_s();
        cur_t_ = ini_t_;
        tar_t_ = t;

        tar_state_.set_params(x, 2.0f * x / t);
        cur_state_.set_params(0.0f, 0.0f);

        process_ = 0.0f;
        is_planning_ = false;
    }
    
    /**
    * @brief 根据当前是否处于规划，更新当前运动学状态
    * @param tar_x 追踪位置引用（规划器输出）
    * @param tar_v 追踪速度引用（规划器输出）
    * @return true 规划完成; false 规划未完成
    */
    bool update(float &tar_x, float &tar_v) {
        cur_t_ = DWT_GetTimeline_s();
        if (is_finished()) {  // 规划完成。锁角
            tar_x = 0.0f;
            tar_v = 0.0f;
            return true;
        } else {  // 规划未完成，继续计算
            float t = cur_t_ - ini_t_;
            tar_x = integral_smooth_curve(t);
            tar_v = smooth_curve(t);
            process_ = t / tar_t_;
            if (process_ >= 1.0f) {
                process_ = 1.0f;
                is_planning_ = false;
            }
            return false;
        }
    }

    const inline bool is_finished() { return !is_planning_; }

    // 属性的getter
    const inline bool get_is_planning() { return is_planning_; }
    const inline float get_process() { return process_; }
    const inline MotionState get_cur_state() { return cur_state_; }

private:
    MotionState tar_state_{0.0f, 0.0f};  // 目标运动学状态
    float ini_t_{0.0f}, tar_t_{0.0f};  // 初始时刻，目标时刻

    MotionState cur_state_{0.0f, 0.0f};  // 当前运动学状态
    float cur_t_{0.0f};  // 当前时刻

    float process_{0.0f};  // 规划进度: 0% -> 100%
    bool is_planning_{false};  // 当前是否完成规划进度

};
