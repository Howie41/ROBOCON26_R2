/**
 * @file motor_task.hpp
 * @author FunFer
 * @brief 电机托管系统（带速度规划器）
 * @version 0.1
 * @date 2026-05-12
 *
 * @copyright Copyright (c) 2026
 *
 * @note : 用的是冯大帅最爱的S规划器，但比七段式更加S
 */
#pragma once

#include <cstdint>
#include <optional>
#include <array>
#include <functional>
#include "Motor.hpp"
#include "pid_controller.h"
#include "filters.hpp"
#include "SmoothSPlanner.hpp"

// MPS中托管电机的最大数量
#define MAX_MOTOR_COUNT 16

class MotorPlanningUnit {
public:
    MotorPlanningUnit() = delete;
    MotorPlanningUnit(const MotorPlanningUnit&) = delete;
    MotorPlanningUnit& operator=(const MotorPlanningUnit&) = delete;
    MotorPlanningUnit(MotorBase &motor, uint8_t pos_pid_control_ratio = 50, uint8_t speed_pid_control_ratio = 10) :
          motor_(motor), SSP_(motor.getCurrentSumPos()), pos_pid_control_ratio_(pos_pid_control_ratio), speed_pid_control_ratio_(speed_pid_control_ratio) {
        pos_pid_ = {  // 位置环 初始化
            .Kp = 10.0f,
            .Ki = 0.5f,
            .Kd = 0.4f,
            .MaxOut = 60.0f,
            .DeadBand = 0.01f
        };
        speed_pid_ = {  // 速度环 初始化
            .Kp = 6.0f,
            .Ki = 2.0f,
            .Kd = 0.1f,
            .MaxOut = 12000.0f,
            .DeadBand = 0.0f
        };
    }
    ~MotorPlanningUnit() {}

    // 按规划控制电机至指定位置（带速度规划的位置环控制）
    bool plan(float t, float x, std::optional<float> max_v = std::nullopt, std::optional<float> max_a = std::nullopt) {
        if (SSP_.is_finished()) return SSP_.init(t, x, max_v, max_a);  // 开始规划
        else return false;  // 不可中断规划
    }

    // 判断位置环/速度环等待周期是否足够（用于拉开各环计算的频段）
    bool is_pos_pid_waiting_enough() { return ++pos_pid_waiting_rec_ > pos_pid_control_ratio_ ? (pos_pid_waiting_rec_ = 0, true) : false; }
    bool is_speed_pid_waiting_enough() { return ++speed_pid_waiting_rec_ > speed_pid_control_ratio_ ? (speed_pid_waiting_rec_ = 0, true) : false; }

    // getter & setter
    inline MotorBase& get_motor() { return motor_; }
    inline SmoothSPlanner& get_SSP() { return SSP_; }
    inline PID_t& get_pos_pid() { return pos_pid_; }
    inline PID_t& get_speed_pid() { return speed_pid_; }
    const inline uint8_t get_pos_pid_control_ratio() { return pos_pid_control_ratio_; }
    void set_pos_pid_control_ratio(uint8_t pos_pid_control_ratio) { pos_pid_control_ratio_ = pos_pid_control_ratio; }
    const inline uint8_t get_speed_pid_control_ratio() { return speed_pid_control_ratio_; }
    void set_speed_pid_control_ratio(uint8_t speed_pid_control_ratio) { speed_pid_control_ratio_ = speed_pid_control_ratio; }

private:
    MotorBase& motor_;  // 电机
    SmoothSPlanner SSP_;  // 规划器
    PID_t pos_pid_;  // 位置环
    PID_t speed_pid_;  // 速度环
    uint8_t pos_pid_control_ratio_{50};  // 位置环控制周期
    uint8_t speed_pid_control_ratio_{10};  // 速度环控制周期

    uint8_t pos_pid_waiting_rec_{0};
    uint8_t speed_pid_waiting_rec_{0};

};


class MotorPlanningSystem {
public:
    MotorPlanningSystem() = default;
    ~MotorPlanningSystem() {}

    bool register_motor(MotorPlanningUnit &MP_Unit) {
        if (motor_count_ >= MAX_MOTOR_COUNT) return false;
        motor_planning_units_[motor_count_] = std::ref(MP_Unit);
        motor_count_++;
        return true;
    }

    void update() {
        for (uint8_t i = 0; i < motor_count_; i++) {
            MotorPlanningUnit& MP_Unit = motor_planning_units_[i]->get();
            float tar_x, tar_v;  // 追踪的目标位置与目标速度
            MP_Unit.get_SSP().update(tar_x, tar_v);
            if (MP_Unit.is_pos_pid_waiting_enough()) PID_Calculate(&MP_Unit.get_pos_pid(), MP_Unit.get_motor().getCurrentSumPos(), tar_x);  // 位置环计算
            if (MP_Unit.is_speed_pid_waiting_enough()) PID_Calculate(&MP_Unit.get_speed_pid(), MP_Unit.get_motor().getCurrentSpeed(), tar_v + MP_Unit.get_pos_pid().Output);  // 速度环计算
            MP_Unit.get_motor().setMotorCmd(MP_Unit.get_speed_pid().Output);  // 设置电机命令
        }
    }
    
private:
    uint8_t motor_count_{0};  // 电机数量
    std::array<std::optional<std::reference_wrapper<MotorPlanningUnit>>, MAX_MOTOR_COUNT> motor_planning_units_{};  // 电机规划单元数组

};


void motorTask(void *argument);

