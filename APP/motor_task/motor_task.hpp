/**
 * @file motor_task.hpp
 * @author FunFer
 * @brief 电机托管系统（带速度规划器）
 * @version 0.1
 * @date 2026-05-18
 *
 * @copyright Copyright (c) 2026
 *
 * @note :
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
#include "bsp_dwt.h"

// MPS中托管电机的最大数量
#define MAX_MOTOR_COUNT 8

class MotorPlanningUnit {
public:
    MotorPlanningUnit(MotorBase &motor, uint8_t pos_pid_control_ratio = 4, uint8_t speed_pid_control_ratio = 2) :
          motor_(motor), SSP_(motor.getCurrentSumPos()), pos_pid_control_ratio_(pos_pid_control_ratio), speed_pid_control_ratio_(speed_pid_control_ratio),
          init_pos_(motor.getCurrentSumPos()), cur_pos_(motor.getCurrentSumPos()), last_pos_(motor.getCurrentSumPos()), stable_pos_(motor.getCurrentSumPos()) {
        pos_pid_ = {
            .Kp = 44.0f,
            .Ki = 0.0f,
            .Kd = 1.0f,
            .MaxOut = 60.0f,
            .DeadBand = 0.1f
        };
        speed_pid_ = {
            .Kp = 3600.0f,
            .Ki = 210000.0f,
            .Kd = 0.0f,
            .MaxOut = 10000.0f,
            .DeadBand = 0.0f,
            .Improve = IMCREATEMENT_OF_OUT
        };
    }
    ~MotorPlanningUnit() {}

    // 按规划控制电机至指定位置（带速度规划的位置环控制）
    void plan(float t, float x) {
        float dx = x - motor_.getCurrentSumPos();
        if (isZero(dx)) return;
        towards_positive_ = dx > 0;  // 记录运动方向
        init_pos_ = motor_.getCurrentSumPos();
        SSP_.init(t, fabsf(dx));  // 开始规划
    }

    // 判断位置环/速度环等待周期是否足够（用于拉开各环计算的频段）
    bool is_pos_pid_waiting_enough() { return ++pos_pid_waiting_rec_ > pos_pid_control_ratio_ ? (pos_pid_waiting_rec_ = 0, true) : false; }
    bool is_speed_pid_waiting_enough() { return ++speed_pid_waiting_rec_ > speed_pid_control_ratio_ ? (speed_pid_waiting_rec_ = 0, true) : false; }
    
    // 获取转动方向
    const inline int8_t get_direction() { return towards_positive_ ? 1 : -1; }

    // 传入实时位置以差分获取速度
    void update_pos(float pos) {
        last_pos_ = cur_pos_;
        cur_pos_ = pos;
        last_time_s_ = cur_time_s_;
        cur_time_s_ = DWT_GetTimeline_s();
        cur_speed_ = (cur_pos_ - last_pos_) / (cur_time_s_ - last_time_s_);
    }

    // getter & setter
    inline MotorBase& get_motor() { return motor_; }
    inline SmoothSPlanner& get_SSP() { return SSP_; }
    inline PID_t& get_pos_pid() { return pos_pid_; }
    inline PID_t& get_speed_pid() { return speed_pid_; }
    const inline uint8_t get_pos_pid_control_ratio() { return pos_pid_control_ratio_; }
    void set_pos_pid_control_ratio(uint8_t pos_pid_control_ratio) { pos_pid_control_ratio_ = pos_pid_control_ratio; }
    const inline uint8_t get_speed_pid_control_ratio() { return speed_pid_control_ratio_; }
    void set_speed_pid_control_ratio(uint8_t speed_pid_control_ratio) { speed_pid_control_ratio_ = speed_pid_control_ratio; }
    const inline float get_init_pos() { return init_pos_; }
    const inline float get_speed() { return cur_speed_; }
    const inline float get_stable_pos() { return stable_pos_; }
    void set_stable_pos(float stable_pos) { stable_pos_ = stable_pos; }

private:
    MotorBase& motor_;  // 电机
    SmoothSPlanner SSP_;  // 规划器
    PID_t pos_pid_;  // 位置环
    PID_t speed_pid_;  // 速度环
    uint8_t pos_pid_control_ratio_{4};  // 位置环控制周期
    uint8_t speed_pid_control_ratio_{2};  // 速度环控制周期

    uint8_t pos_pid_waiting_rec_{0};
    uint8_t speed_pid_waiting_rec_{0};

    bool towards_positive_{true};  // 电机运动方向 (是否为正方向)

    bool init_pos_;
    float cur_pos_;
    float last_pos_;
    float stable_pos_;

    float cur_speed_{0.0f};

    float cur_time_s_{0.0f};
    float last_time_s_{0.0f};
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
            float cur_pos = MP_Unit.get_motor().getCurrentSumPos();
            MP_Unit.update_pos(cur_pos);
            float tar_x, tar_v;  // 追踪的 raw目标位置(相对于初始位置的绝对值) 与 raw目标速度(绝对值)
            if (MP_Unit.get_SSP().is_finished()) {  // 规划已完成
                tar_x = MP_Unit.get_stable_pos();
                tar_v = 0.0f;
            } else {  // 规划未完成
                if (MP_Unit.get_SSP().update(tar_x, tar_v)) {  // 开始锁角
                    MP_Unit.set_stable_pos(cur_pos);
                }
                tar_x = MP_Unit.get_init_pos() + tar_x * MP_Unit.get_direction();
                tar_v = tar_v * MP_Unit.get_direction();
            }
            if (MP_Unit.is_pos_pid_waiting_enough()) PID_Calculate(&MP_Unit.get_pos_pid(), cur_pos, tar_x);  // 位置环计算
            if (MP_Unit.is_speed_pid_waiting_enough()) PID_Calculate(&MP_Unit.get_speed_pid(), MP_Unit.get_speed(), tar_v + MP_Unit.get_pos_pid().Output);  // 速度环计算
            MP_Unit.get_motor().setMotorCmd(MP_Unit.get_speed_pid().Output);  // 设置电机命令
        }
    }
    
private:
    uint8_t motor_count_{0};  // 电机数量
    std::array<std::optional<std::reference_wrapper<MotorPlanningUnit>>, MAX_MOTOR_COUNT> motor_planning_units_{};  // 电机规划单元数组

};


void motorTask(void *argument);

