/**
 * @file arm_task.hpp
 * @author FunFer
 * @brief 取矿机构任务
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note arm动作调用链:
 *     上层执行arm.xxx(...);（对外动作接口），对应arm属性is_xxxing变为true；
 *      arm_task任务大循环里轮询arm的is_xxxing属性，并执行arm.run_xxxing(...)；
 *      arm.run_xxxing(...)中按config中的时间序列执行arm.xxx_proceed(...)；
 *      arm.xxx_proceed(...);中调用arm.set_pose(...)设定到对应姿态。
 */

#pragma once

#include "Motor.hpp"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include <optional>
#include "arm_actions_config.hpp"

enum class LOAD_TYPE: int8_t {
    LOW = -1,
    PLAIN = 0,
    MEDIUM = 1,
    HIGH = 2,
    TOP = 4
};

enum class UNLOAD_TYPE : uint8_t {
    LOW = 1,
    MEDIUM = 2,
    HIGH = 3,
    TOP = 4
};

namespace arm_action {
    bool load_kfs(LOAD_TYPE step);
    bool unload_kfs(std::optional<UNLOAD_TYPE> level);
    void release_kfs();
    void raise_kfs_top();
}


class Arm {
public:
    Arm(DM43xxMotor &arm_lift, MotorBase &arm_rotate, MotorBase &arm_expand, DM43xxMotor &arm_flip, uint8_t kfs_num = 0) : arm_lift_(arm_lift), arm_rotate_(arm_rotate), arm_expand_(arm_expand), arm_flip_(arm_flip), kfs_num_(kfs_num) {}
    ~Arm() {}


    // 对外KFS控制接口
    bool fetch_step(LOAD_TYPE step) {
        if (get_kfs_amount() == 3 || (step == LOAD_TYPE::TOP && !get_is_kfs_raised()) || (step != LOAD_TYPE::TOP && get_is_kfs_raised())) return false;
        reset_timeline();
        switch (step) {
            case LOAD_TYPE::MEDIUM: { set_is_fetching_step_M(true); break; }
            case LOAD_TYPE::HIGH: { set_is_fetching_step_H(true); break; }
            case LOAD_TYPE::LOW: { set_is_fetching_step_L(true); break; }
            case LOAD_TYPE::PLAIN: { set_is_fetching_step_P(true); break; }
            case LOAD_TYPE::TOP: { set_is_fetching_step_T(true); break; }
        }
        return true;
    }
    bool place_kfs(std::optional<UNLOAD_TYPE> kfs_layer = std::nullopt) {
        if (kfs_layer.has_value()) {  // 取特定层KFS
            if ((kfs_layer.value() == UNLOAD_TYPE::TOP && !get_is_kfs_raised()) || (get_is_kfs_raised() && kfs_layer.value() != UNLOAD_TYPE::TOP)) return false;
            reset_timeline();
            switch (kfs_layer.value()) {
                case UNLOAD_TYPE::LOW: { set_is_placing_kfs_L(true); break; }
                case UNLOAD_TYPE::MEDIUM: { set_is_placing_kfs_M(true); break; }
                case UNLOAD_TYPE::HIGH: { set_is_placing_kfs_H(true); break; }
                case UNLOAD_TYPE::TOP: { set_is_placing_kfs_T(true); break; }
            }
        } else {  // 默认值
            if ((get_kfs_amount() == 0 && !get_is_kfs_raised())) return false;
            reset_timeline();
            if (get_is_kfs_raised()) set_is_placing_kfs_T(true);
            else switch (get_kfs_amount()) {
                case 1: { set_is_placing_kfs_L(true); break; }
                case 2: { set_is_placing_kfs_M(true); break; }
                case 3: { set_is_placing_kfs_H(true); break; }
            }
        }
        return true;
    }
    void place_release() { reset_timeline(); set_is_place_releasing(true); }
    void raise_kfs() { reset_timeline(); set_is_raising_kfs(true); }
    void start() { reset_timeline(); set_is_starting(true); }

    
    // 重置计时器
    void reset_timeline() { act_index_ = 0; last_t_ = DWT_GetTimeline_s(); }



    // 电机控制类行为基，为电机角度控制提供相对的基准值
    void setHeight(float pos_deg, float speed_deg) { arm_lift_.posWithSpeedControl(215.0f + pos_deg, speed_deg); }
    void setRotate(float pos, float speed, float ini_buffer_pos, float end_buffer_pos) { arm_rotate_.posWithSpeedControl(pos + 70.0f, speed, ini_buffer_pos, end_buffer_pos, 0.0f, 0.0f); }
    void setExpand(float pos, float speed, float ini_buffer_pos, float end_buffer_pos) { arm_expand_.posWithSpeedControl(-pos, speed, ini_buffer_pos, end_buffer_pos, 0.0f, 0.0f); }
    void setFlip(float pos_deg, float speed_deg) { arm_flip_.posWithSpeedControl(-pos_deg, speed_deg); }
    
    // 核心动作行为，姿态控制类接口，以此将config中的姿态解析并执行。动作链末端需要主动增加kfs_num_，且返回true
    bool set_pose(arm_pose pose) {
        if (!(pose.special_operations & SKIP_MOTOR_CONTROL_)) {
            setHeight(pose.h.pos, pose.h.speed);
            setFlip(pose.f.pos, pose.f.speed);
            setRotate(pose.r.pos, pose.r.speed, pose.r.ip, pose.r.ep);
            setExpand(pose.e.pos, pose.e.speed, pose.e.ip, pose.e.ep);
        }
        if (pose.special_operations & RESET_) reset();
        if (pose.special_operations & FETCH_) fetch();
        if (pose.special_operations & PLACE_RELEASE_START_) place_release_start();
        if (pose.special_operations & PLACE_RELEASE_STOP_) place_release_stop();
        return pose.is_end;
    }

    // 吸取KFS入储存的具体原子动作序列（包含姿态点位，不包含时间序列）
    bool fetch_proceed(LOAD_TYPE step, uint8_t index) {  // 此函数不会增加kfs_num_，需要在外部结束动作链后主动增加kfs_num_
        auto get_pose_by_kfs = [&](auto& kfs_0, auto& kfs_1, auto& kfs_2) -> bool {
            switch (kfs_num_) {
                case 0: return set_pose(kfs_0[index]);
                case 1: return set_pose(kfs_1[index]);
                case 2: return set_pose(kfs_2[index]);
                default: return false;
            }
        };
        using namespace arm_actions_config::fetch_proceed;
        switch (step) {
            case LOAD_TYPE::MEDIUM: return get_pose_by_kfs(step_M::kfs_0, step_M::kfs_1, step_M::kfs_2);
            case LOAD_TYPE::HIGH: return get_pose_by_kfs(step_H::kfs_0, step_H::kfs_1, step_H::kfs_2);
            case LOAD_TYPE::LOW: return get_pose_by_kfs(step_L::kfs_0, step_L::kfs_1, step_L::kfs_2);
            case LOAD_TYPE::PLAIN: return get_pose_by_kfs(step_P::kfs_0, step_P::kfs_1, step_P::kfs_2);
            case LOAD_TYPE::TOP: return get_pose_by_kfs(step_T::kfs_0, step_T::kfs_1, step_T::kfs_2);
            default: return false;
        }
    }
    // 取出KFS出储存的具体原子动作序列（包含姿态点位，不包含时间序列）
    bool place_proceed(uint8_t index) {  // 此函数不会减少kfs_num_，需要在外部结束动作链后主动减少kfs_num_
        if (is_kfs_raised_) {
            return set_pose(arm_actions_config::place_proceed::kfs_4[index]);
        } else {
            if (kfs_num_ == 1) return set_pose(arm_actions_config::place_proceed::kfs_1[index]);
            else if (kfs_num_ == 2) return set_pose(arm_actions_config::place_proceed::kfs_2[index]);
            else if (kfs_num_ == 3) return set_pose(arm_actions_config::place_proceed::kfs_3[index]);
        }
        return false;
    }
    // 释放取出的KFS，并reset
    bool place_release_proceed(uint8_t index) { return set_pose(arm_actions_config::place_release_proceed[index]); }
    // 吸取平地KFS并举高高
    bool raise_kfs_proceed(uint8_t index) { return set_pose(arm_actions_config::raise_kfs_proceed[index]); }
    // 启动时初始化
    bool start_proceed(uint8_t index) { return set_pose(arm_actions_config::start_proceed[index]); }

    // KFS数量控制类接口
    void addKFS() { kfs_num_++; }
    void rmvKFS() { kfs_num_--; }

    // 气泵控制类行为
    void fetch() { set_is_holding_kfs(true); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET); }
    void release() { set_is_holding_kfs(false); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET); }
    void destroy_vaccum_start() { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET); }
    void destroy_vaccum_stop() { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET); }
    // 恢复至默认姿态
    void reset() { set_pose(arm_actions_config::reset); }
    // 气泵控制类接口
    void place_release_start() { release(); destroy_vaccum_start(); }
    void place_release_stop() { destroy_vaccum_stop(); }



    // fetching_step 模块化实现
    void run_fetching_step(LOAD_TYPE step) {
        set_now_t(DWT_GetTimeline_s() - get_last_t());  // now_t记录以last_t为基准的相对时间
        float delta_t;
        void (Arm::*setter)(bool);
        using namespace arm_actions_config::fetch_proceed;
        auto get_dt_by_kfs = [&](auto& kfs_0, auto& kfs_1, auto& kfs_2) -> float {
            switch (get_kfs_amount()) {
                case 0: return kfs_0[act_index_].delta_t;
                case 1: return kfs_1[act_index_].delta_t;
                case 2: return kfs_2[act_index_].delta_t;
                default: return 0.0f;
            }
        };
        switch (step) {
            case LOAD_TYPE::HIGH: { delta_t = get_dt_by_kfs(step_H::kfs_0, step_H::kfs_1, step_H::kfs_2); setter = &Arm::set_is_fetching_step_H; break; }
            case LOAD_TYPE::MEDIUM: { delta_t = get_dt_by_kfs(step_M::kfs_0, step_M::kfs_1, step_M::kfs_2); setter = &Arm::set_is_fetching_step_M; break; }
            case LOAD_TYPE::LOW: { delta_t = get_dt_by_kfs(step_L::kfs_0, step_L::kfs_1, step_L::kfs_2); setter = &Arm::set_is_fetching_step_L; break; }
            case LOAD_TYPE::PLAIN: { delta_t = get_dt_by_kfs(step_P::kfs_0, step_P::kfs_1, step_P::kfs_2); setter = &Arm::set_is_fetching_step_P; break; }
            case LOAD_TYPE::TOP: { delta_t = get_dt_by_kfs(step_T::kfs_0, step_T::kfs_1, step_T::kfs_2); setter = &Arm::set_is_fetching_step_T; break; }
        }
        if (get_now_t() > delta_t) {
            if (fetch_proceed(step, act_index_++)) { (this->*setter)(false); addKFS(); set_is_kfs_raised(false); }
            else set_last_t(DWT_GetTimeline_s());
        }
    }
    // placing_kfs 模块化实现
    void run_placing_kfs(UNLOAD_TYPE layer) {
        set_now_t(DWT_GetTimeline_s() - get_last_t());  // now_t记录以last_t为基准的相对时间
        float delta_t;
        void (Arm::*setter)(bool);
        using namespace arm_actions_config::place_proceed;
        switch (layer) {
            case UNLOAD_TYPE::LOW: { delta_t = kfs_1[act_index_].delta_t; setter = &Arm::set_is_placing_kfs_L; break; }
            case UNLOAD_TYPE::MEDIUM: { delta_t = kfs_2[act_index_].delta_t; setter = &Arm::set_is_placing_kfs_M; break; }
            case UNLOAD_TYPE::HIGH: { delta_t = kfs_3[act_index_].delta_t; setter = &Arm::set_is_placing_kfs_H; break; }
            case UNLOAD_TYPE::TOP: { delta_t = kfs_4[act_index_].delta_t; setter = &Arm::set_is_placing_kfs_T; break; }
        }
        if (get_now_t() > delta_t) {
            if (place_proceed(act_index_++)) {
                if (get_is_placing_kfs_T()) set_is_kfs_raised(false);
                else rmvKFS();
                (this->*setter)(false);
            }
            else set_last_t(DWT_GetTimeline_s());
        }
    }
    // place_releasing 模块化实现
    void run_place_releasing() {
        set_now_t(DWT_GetTimeline_s() - get_last_t());  // now_t记录以last_t为基准的相对时间
        if (get_now_t() > arm_actions_config::place_release_proceed[act_index_].delta_t) {
            if (place_release_proceed(act_index_++)) set_is_place_releasing(false);
            else set_last_t(DWT_GetTimeline_s());
        }
    }
    // raising_kfs 模块化实现
    void run_raising_kfs() {
        set_now_t(DWT_GetTimeline_s() - get_last_t());  // now_t记录以last_t为基准的相对时间
        if (get_now_t() > arm_actions_config::raise_kfs_proceed[act_index_].delta_t) {
            if (raise_kfs_proceed(act_index_++)) { set_is_raising_kfs(false); set_is_kfs_raised(true); }
            else set_last_t(DWT_GetTimeline_s());
        }
    }

    // 启动时初始化
    void run_starting() {
        set_now_t(DWT_GetTimeline_s() - get_last_t());  // now_t记录以last_t为基准的相对时间
        if (get_now_t() > arm_actions_config::start_proceed[act_index_].delta_t) {
            if (start_proceed(act_index_++)) { set_is_starting(false); }
            else set_last_t(DWT_GetTimeline_s());
        }
    }



    // 状态属性的getter与setter
    inline const uint8_t get_kfs_amount() { return kfs_num_; }
    void set_kfs_amount(uint8_t num) { kfs_num_ = num; }
    inline const bool get_is_fetching_step_L() { return is_fetching_step_L_; }
    void set_is_fetching_step_L(bool is_fetching_step_L) { is_fetching_step_L_ = is_fetching_step_L; }
    inline const bool get_is_fetching_step_P() { return is_fetching_step_P_; }
    void set_is_fetching_step_P(bool is_fetching_step_P) { is_fetching_step_P_ = is_fetching_step_P; }
    inline const bool get_is_fetching_step_M() { return is_fetching_step_M_; }
    void set_is_fetching_step_M(bool is_fetching_step_M) { is_fetching_step_M_ = is_fetching_step_M; }
    inline const bool get_is_fetching_step_H() { return is_fetching_step_H_; }
    void set_is_fetching_step_H(bool is_fetching_step_H) { is_fetching_step_H_ = is_fetching_step_H; }
    inline const bool get_is_fetching_step_T() { return is_fetching_step_T_; }
    void set_is_fetching_step_T(bool is_fetching_step_T) { is_fetching_step_T_ = is_fetching_step_T; }
    inline const bool get_is_placing_kfs_L() { return is_placing_kfs_L_; }
    void set_is_placing_kfs_L(bool is_placing_kfs_L) { is_placing_kfs_L_ = is_placing_kfs_L; }
    inline const bool get_is_placing_kfs_M() { return is_placing_kfs_M_; }
    void set_is_placing_kfs_M(bool is_placing_kfs_M) { is_placing_kfs_M_ = is_placing_kfs_M; }
    inline const bool get_is_placing_kfs_H() { return is_placing_kfs_H_; }
    void set_is_placing_kfs_H(bool is_placing_kfs_H) { is_placing_kfs_H_ = is_placing_kfs_H; }
    inline const bool get_is_placing_kfs_T() { return is_placing_kfs_T_; }
    void set_is_placing_kfs_T(bool is_placing_kfs_T) { is_placing_kfs_T_ = is_placing_kfs_T; }
    inline const bool get_is_place_releasing() { return is_place_releasing_; }
    void set_is_place_releasing(bool is_place_releasing) { is_place_releasing_ = is_place_releasing; }
    inline const bool get_is_raising_kfs() { return is_raising_kfs_; }
    void set_is_raising_kfs(bool is_raising_kfs) { is_raising_kfs_ = is_raising_kfs; }
    inline const bool get_is_starting() { return is_starting_; }
    void set_is_starting(bool is_starting) { is_starting_ = is_starting; }
    inline const bool get_is_kfs_raised() { return is_kfs_raised_; }
    void set_is_kfs_raised(bool is_kfs_raised) { is_kfs_raised_ = is_kfs_raised; }
    inline const bool get_is_holding_kfs() { return is_holding_kfs_; }
    void set_is_holding_kfs(bool is_holding_kfs) { is_holding_kfs_ = is_holding_kfs; }
    inline const float get_now_t() { return now_t_; }
    void set_now_t(float now_t) { now_t_ = now_t; }
    inline const float get_last_t() { return last_t_; }
    void set_last_t(float last_t) { last_t_ = last_t; }
    
private:
    DM43xxMotor &arm_lift_;
    MotorBase &arm_rotate_;
    MotorBase &arm_expand_;
    DM43xxMotor &arm_flip_;

    uint8_t kfs_num_{0};

    // 吸取KFS
    bool is_fetching_step_L_{false};
    bool is_fetching_step_P_{false};
    bool is_fetching_step_M_{false};
    bool is_fetching_step_H_{false};
    bool is_fetching_step_T_{false};
    // 取出KFS
    bool is_placing_kfs_L_{false};
    bool is_placing_kfs_M_{false};
    bool is_placing_kfs_H_{false};
    bool is_placing_kfs_T_{false};
    // 释放KFS并复位
    bool is_place_releasing_{false};
    // 举高高KFS
    bool is_raising_kfs_{false};
    // 启动时初始化
    bool is_starting_{false};

    bool is_kfs_raised_{false};
    bool is_holding_kfs_{false};

    uint8_t act_index_ = 0;  // 用于记录动作索引
    float now_t_ = 0.0f;  // 当前动作相对时刻记录
    float last_t_ = 0.0f;  // 上一次动作相对时刻记录

};

void armTask(void *argument);
