/**
 * @file arm_task.hpp
 * @author FunFer
 * @brief 取矿机构任务
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 * @note :
 * @versioninfo :
 */

#pragma once

#include "Motor.hpp"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_gpio.h"
#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include "arm_actions_config.hpp"


#define B 66.0f // 预高度补偿

void fetch_step(int8_t step);
void place_kfs(int8_t kfs_layer);
void place_release();

namespace arm_action {
    void load_kfs(int8_t step);
    void unload_kfs(int8_t level);
    void release_kfs();
}

class Arm {

public:
    // 构造函数
    Arm(DM43xxMotor &arm_lift, MotorBase &arm_rotate, MotorBase &arm_expand, DM43xxMotor &arm_flip, uint8_t kfs_num = 0) : arm_lift_(arm_lift), arm_rotate_(arm_rotate), arm_expand_(arm_expand), arm_flip_(arm_flip), kfs_num_(kfs_num) {}
    ~Arm() {}

    // KFS数量控制类接口
    void addKFS() { kfs_num_++; }
    void rmvKFS() { kfs_num_--; }
    uint8_t get_kfs_amount() { return kfs_num_; }
    void set_kfs_amount(uint8_t num) { kfs_num_ = num; }

    // 气泵控制类行为
    void fetch() { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET); }
    void release() { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET); }
    void destroy_vaccum_start() { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET); }
    void destroy_vaccum_stop() { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET); }
    // 恢复至默认姿态
    void reset() { set_pose(arm_actions_config::reset); }
    // 气泵控制类接口
    void place_release_start() { release(); destroy_vaccum_start(); }
    void place_release_stop() { destroy_vaccum_stop(); }

    // 电机控制类行为基，为电机角度控制提供相对的基准值
    void setHeight(float pos_deg, float speed_deg) { arm_lift_.posWithSpeedControl(B + pos_deg, speed_deg); }
    void setRotate(float pos, float speed, float ini_buffer_pos, float end_buffer_pos) { arm_rotate_.posWithSpeedControl(pos, speed, ini_buffer_pos, end_buffer_pos, 0.0f, 0.0f); }
    void setExpand(float pos, float speed, float ini_buffer_pos, float end_buffer_pos) { arm_expand_.posWithSpeedControl(-pos, speed, ini_buffer_pos, end_buffer_pos, 0.0f, 0.0f); }
    void setFlip(float pos_deg, float speed_deg) { arm_flip_.posWithSpeedControl(-pos_deg, speed_deg); }
    
    // 核心动作行为，姿态控制类接口，以此将config中的姿态解析并执行。动作链末端需要主动增加kfs_num_，且返回true
    bool set_pose(arm_pose pose) {
        if (!(pose.special_operations & 0b00000001)) {
            setHeight(pose.h.pos, pose.h.speed);
            setFlip(pose.f.pos, pose.f.speed);
            setRotate(pose.r.pos, pose.r.speed, pose.r.ip, pose.r.ep);
            setExpand(pose.e.pos, pose.e.speed, pose.e.ip, pose.e.ep);
        }
        if (pose.special_operations & 0b00000010) reset();
        if (pose.special_operations & 0b00000100) fetch();
        if (pose.special_operations & 0b00001000) release();
        if (pose.special_operations & 0b00010000) destroy_vaccum_start();
        if (pose.special_operations & 0b00100000) destroy_vaccum_stop();
        if (pose.special_operations & 0b01000000) place_release_start();
        if (pose.special_operations & 0b10000000) place_release_stop();
        return pose.is_end;
    }

    // 吸取KFS入储存的具体原子动作序列（包含姿态点位，不包含时间序列）
    bool fetch_proceed(int8_t step, uint8_t index) {  // 此函数不会增加kfs_num_，需要在外部结束动作链后主动增加kfs_num_
        if (step == 1) {
            if (kfs_num_ == 0 || kfs_num_ == 1) return set_pose(arm_actions_config::fetch_proceed::step_M::kfs_0_1[index]);
            else if (kfs_num_ == 2) return set_pose(arm_actions_config::fetch_proceed::step_M::kfs_2[index]);
        } else if (step == 2) {
            if (kfs_num_ == 0 || kfs_num_ == 1) return set_pose(arm_actions_config::fetch_proceed::step_H::kfs_0_1[index]);
            else if (kfs_num_ == 2) return set_pose(arm_actions_config::fetch_proceed::step_H::kfs_2[index]);
        } else if (step == -1) {
            if (kfs_num_ == 0 || kfs_num_ == 1) return set_pose(arm_actions_config::fetch_proceed::step_L::kfs_0_1[index]);
            else if (kfs_num_ == 2) return set_pose(arm_actions_config::fetch_proceed::step_L::kfs_2[index]);
        }
        return false;
    }
    // 取出KFS出储存的具体原子动作序列（包含姿态点位，不包含时间序列）
    bool place_proceed(uint8_t index) {  // 此函数不会减少kfs_num_，需要在外部结束动作链后主动减少kfs_num_
        if (kfs_num_ == 1) return set_pose(arm_actions_config::place_proceed::kfs_1[index]);
        else if (kfs_num_ == 2) return set_pose(arm_actions_config::place_proceed::kfs_2[index]);
        else if (kfs_num_ == 3) return set_pose(arm_actions_config::place_proceed::kfs_3[index]);
        return false;
    }
    // 释放取出的KFS，并reset
    bool place_release_proceed(uint8_t index) {
        return set_pose(arm_actions_config::place_release_proceed[index]);
    }

    // 状态属性的getter与setter
    bool get_is_fetching_step_L() { return is_fetching_step_L_; }
    void set_is_fetching_step_L(bool is_fetching_step_L) { is_fetching_step_L_ = is_fetching_step_L; }
    bool get_is_fetching_step_M() { return is_fetching_step_M_; }
    void set_is_fetching_step_M(bool is_fetching_step_M) { is_fetching_step_M_ = is_fetching_step_M; }
    bool get_is_fetching_step_H() { return is_fetching_step_H_; }
    void set_is_fetching_step_H(bool is_fetching_step_H) { is_fetching_step_H_ = is_fetching_step_H; }
    bool get_is_placing_kfs_L() { return is_placing_kfs_L_; }
    void set_is_placing_kfs_L(bool is_placing_kfs_L) { is_placing_kfs_L_ = is_placing_kfs_L; }
    bool get_is_placing_kfs_M() { return is_placing_kfs_M_; }
    void set_is_placing_kfs_M(bool is_placing_kfs_M) { is_placing_kfs_M_ = is_placing_kfs_M; }
    bool get_is_placing_kfs_H() { return is_placing_kfs_H_; }
    void set_is_placing_kfs_H(bool is_placing_kfs_H) { is_placing_kfs_H_ = is_placing_kfs_H; }
    bool get_is_place_releasing() { return is_place_releasing_; }
    void set_is_place_releasing(bool is_place_releasing) { is_place_releasing_ = is_place_releasing; }
    
private:
    DM43xxMotor &arm_lift_;
    MotorBase &arm_rotate_;
    MotorBase &arm_expand_;
    DM43xxMotor &arm_flip_;

    uint8_t kfs_num_{0};

    bool is_fetching_step_L_{false};
    bool is_fetching_step_M_{false};
    bool is_fetching_step_H_{false};

    bool is_placing_kfs_L_{false};
    bool is_placing_kfs_M_{false};
    bool is_placing_kfs_H_{false};

    bool is_place_releasing_{false};

};

void armTask(void *argument);
