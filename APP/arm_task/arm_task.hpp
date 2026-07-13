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
#include "arm_task.hpp"
#include "stm32h7xx_hal_gpio.h"
#include <cmath>
#include <stdint.h>
#include <algorithm>
#include <stdio.h>
#include <optional>
#include "arm_actions_config.hpp"
#include "logger.hpp"


#define DELTA_DELAY 0.08f  // 动作统一延时(s)

extern LoggerQueue logger_queue;

enum class LOAD_TYPE: int8_t {
    LOW = -1,
    PLAIN = 0,
    MEDIUM = 1,
    HIGH = 2
};

enum class UNLOAD_TYPE : uint8_t {
    LOW = 1,
    MEDIUM = 2,
    TOP = 4
};

namespace arm_action {
/**
 * @brief 吸取对应层高的kfs、kfs_amount+1
 * @note 伸出、吸取、抬起
 * @param 参数接收: LOAD_TYPE::MEDIUM, LOAD_TYPE::HIGH, LOAD_TYPE::LOW, LOAD_TYPE::PLAIN的输入，对应+200, +400, -200, 0高度的kfs
 */
bool raise_kfs(LOAD_TYPE step);
/**
 * @brief 取出kfs，默认取最外层
 * @note 抬起、伸入储存区、吸取、抬起、伸出
 * @param std::nullopt:自动识别高度，UNLOAD_TYPE::LOW, UNLOAD_TYPE::MEDIUM, UNLOAD_TYPE::TOP : 指定高度
 */
bool unload_kfs(std::optional<UNLOAD_TYPE> level = std::nullopt, bool is_layer3 = false);
/**
 * @brief 承接unload_kfs，释放kfs并恢复默认动作、kfs_amount-1
 */
bool release_kfs();
/**
 * @brief 将举起的kfs放入储存
 */
bool load_kfs();
/**
 * @brief 丢掉kfs（举着的kfs）
 */
bool drop_kfs();

}


struct ArmAttr {
    // 抓取台阶上的kfs
    bool is_fetching_step_H{false};
    bool is_fetching_step_M{false};
    bool is_fetching_step_L{false};
    bool is_fetching_step_P{false};
    // 放置kfs
    bool is_placing_kfs_L{false};
    bool is_placing_kfs_M{false};
    bool is_placing_kfs_T{false};
    // 是否放置第三层
    bool is_placing_to_layer3{false};
    // 放置后释放
    bool is_place_releasing{false};
    // 存入kfs
    bool is_loading_kfs{false};
    // 丢弃kfs
    bool is_dropping_kfs{false};
    // 启动时
    bool is_starting{false};
    // 其他
    bool is_kfs_raised{false};
    bool is_holding_kfs{false};
    bool is_started{false};
};


class Arm {
public:
    Arm(DM43xxMotor &arm_lift, MotorBase &arm_rotate, MotorBase &arm_expand, DM43xxMotor &arm_flip, uint8_t kfs_num = 0) : arm_lift_(arm_lift), arm_rotate_(arm_rotate), arm_expand_(arm_expand), arm_flip_(arm_flip), kfs_num_(kfs_num) {}
    ~Arm() {}

    

    // 电机控制类行为基，为电机角度控制提供相对的基准值
    void setHeight(float pos_deg, float speed_deg) { arm_lift_.posWithSpeedControl(pos_deg - 30.0f, speed_deg); }
    void setRotate(float pos, float speed) { arm_rotate_.posWithSpeedControl(-pos - 84.3f, 0.0025f * speed); }
    void setExpand(float pos, float speed) { arm_expand_.posWithSpeedControl(std::clamp(pos, 30.0f, 1200.0f) + 31.252f, 4.3f * speed); }
    void setFlip(float pos_deg, float speed_deg) { arm_flip_.posWithSpeedControl(3.0f - pos_deg, speed_deg); }
    
    // 核心动作行为，姿态控制类接口，以此将config中的姿态解析并执行。动作链末端需要主动增加kfs_num_，且返回true
    bool set_pose(arm_pose pose) {
        if (!(pose.special_operations & SKIP_MOTOR_CONTROL_)) {
            setHeight(pose.h.pos, pose.h.speed);
            setFlip(pose.f.pos, pose.f.speed);
            setRotate(pose.r.pos, pose.r.speed);
            setExpand(pose.e.pos, pose.e.speed);
        }
        if (pose.special_operations & RESET_) reset();
        if (pose.special_operations & FETCH_) fetch();
        if (pose.special_operations & RELEASE_) release();
        return pose.is_end;
    }



    // 对外KFS控制接口
    bool fetch_step(LOAD_TYPE step) {
        // if (attr_.is_kfs_raised) return false;
        reset_timeline();
        switch (step) {
            case LOAD_TYPE::MEDIUM: { attr_.is_fetching_step_M = true; break; }
            case LOAD_TYPE::HIGH: { attr_.is_fetching_step_H = true; break; }
            case LOAD_TYPE::LOW: { attr_.is_fetching_step_L = true; break; }
            case LOAD_TYPE::PLAIN: { attr_.is_fetching_step_P = true; break; }
        }
        return true;
    }
    bool place_kfs(std::optional<UNLOAD_TYPE> kfs_layer = std::nullopt, bool is_layer3 = false) {
        if (kfs_layer.has_value()) {  // 取特定层KFS
            // if ((kfs_layer.value() != UNLOAD_TYPE::TOP && attr_.is_kfs_raised) || (kfs_layer.value() == UNLOAD_TYPE::TOP && !attr_.is_kfs_raised)) return false;
            reset_timeline();
            switch (kfs_layer.value()) {
                case UNLOAD_TYPE::LOW: { attr_.is_placing_kfs_L = true; break; }
                case UNLOAD_TYPE::MEDIUM: { attr_.is_placing_kfs_M = true; break; }
                case UNLOAD_TYPE::TOP: { attr_.is_placing_kfs_T = true; break; }
            }
        } else {  // 默认值
            // if (get_kfs_amount() == 0) return false;
            reset_timeline();
            if (attr_.is_kfs_raised) attr_.is_placing_kfs_T = true;
            else switch (get_kfs_amount()) {
                case 1: { attr_.is_placing_kfs_L = true; break; }
                case 2: { attr_.is_placing_kfs_M = true; break; }
                case 3: { attr_.is_placing_kfs_T = true; break; }
            }
        }
        attr_.is_placing_to_layer3 = is_layer3;
        return true;
    }
    bool place_release() {
        reset_timeline();
        attr_.is_place_releasing = true;
        return true;
    }
    bool load_kfs() {
        // if (!attr_.is_kfs_raised) return false;
        reset_timeline();
        attr_.is_loading_kfs = true;
        return true;
    }
    bool drop_kfs() {
        // if (!attr_.is_kfs_raised) return false;
        reset_timeline();
        attr_.is_dropping_kfs = true;
        return true;
    }
    bool start() {
        if (attr_.is_started) return false;
        reset_timeline();
        attr_.is_started = true; 
        attr_.is_starting = true;
        return true;
    }

    
    // 重置计时器
    void reset_timeline() { act_index_ = 0; last_t_ = DWT_GetTimeline_s(); }


    // 吸取KFS入储存的具体原子动作序列（包含姿态点位，不包含时间序列）
    bool fetch_proceed(LOAD_TYPE step, uint8_t index) {  // 此函数不会增加kfs_num_，需要在外部结束动作链后主动增加kfs_num_
        using namespace arm_actions_config::fetch_proceed;
        switch (step) {
            case LOAD_TYPE::MEDIUM: return set_pose(step_M[index]);
            case LOAD_TYPE::HIGH: return set_pose(step_H[index]);
            case LOAD_TYPE::LOW: return set_pose(step_L[index]);
            case LOAD_TYPE::PLAIN: return set_pose(step_P[index]);
            default: return false;
        }
    }
    // 取出KFS出储存的具体原子动作序列（包含姿态点位，不包含时间序列）
    bool place_proceed(uint8_t index) {  // 此函数不会减少kfs_num_，需要在外部结束动作链后主动减少kfs_num_
        if (attr_.is_kfs_raised) {
            return set_pose(arm_actions_config::place_proceed::kfs_3[index]);
        } else {
            if (kfs_num_ == 1) return set_pose(arm_actions_config::place_proceed::kfs_1[index]);
            else if (kfs_num_ == 2) return set_pose(arm_actions_config::place_proceed::kfs_2[index]);
        }
        return false;
    }
    // 释放取出的KFS，并reset
    bool place_release_proceed(uint8_t index) { return set_pose(arm_actions_config::place_release_proceed[index]); }
    // 存入KFS
    bool load_kfs_proceed(uint8_t index) {
        using namespace arm_actions_config::load_kfs_proceed;
        switch (get_kfs_amount()) {
            case 1: return set_pose(kfs_0[index]);
            case 2: return set_pose(kfs_1[index]);
        }
        return false;
    }
    // 丢弃KFS
    bool drop_kfs_proceed(uint8_t index) {
        using namespace arm_actions_config::drop_kfs_proceed;
        switch (get_kfs_amount()) {
            case 1: case 2: return set_pose(kfs_12[index]);
            case 3: return set_pose(kfs_3[index]);
        }
        return false;
    }
    // 启动时初始化
    bool start_proceed(uint8_t index) { return set_pose(arm_actions_config::start_proceed[index]); }

    // KFS数量控制类接口
    void addKFS() {
        if (get_kfs_amount() < 3) {
            kfs_num_++;
            logger_queue.log("ARM\tadd_kfs -> %d\n", kfs_num_);
        }
    }
    void rmvKFS() {
        if (get_kfs_amount() > 0) {
            kfs_num_--;
            logger_queue.log("ARM\tremove_kfs -> %d\n", kfs_num_);
        }
    }
    uint8_t get_kfs_amount() { return kfs_num_; }
    void set_kfs_amount(uint8_t num) { kfs_num_ = num; }

    // 气泵控制类行为
    void fetch() { attr_.is_holding_kfs = true; HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET); }
    void release() { attr_.is_holding_kfs = false; HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET); }
    
    // 恢复至默认姿态
    void reset() { set_pose(arm_actions_config::reset); }



    // fetching_step 模块化实现
    void run_fetching_step(LOAD_TYPE step) {
        now_t_ = DWT_GetTimeline_s() - last_t_;  // now_t记录以last_t为基准的相对时间
        float delta_t;
        bool* setter;
        using namespace arm_actions_config::fetch_proceed;
        switch (step) {
            case LOAD_TYPE::HIGH: { delta_t = step_H[act_index_].delta_t; setter = &attr_.is_fetching_step_H; break; }
            case LOAD_TYPE::MEDIUM: { delta_t = step_M[act_index_].delta_t; setter = &attr_.is_fetching_step_M; break; }
            case LOAD_TYPE::LOW: { delta_t = step_L[act_index_].delta_t; setter = &attr_.is_fetching_step_L; break; }
            case LOAD_TYPE::PLAIN: { delta_t = step_P[act_index_].delta_t; setter = &attr_.is_fetching_step_P; break; }
        }
        delta_t += DELTA_DELAY;
        if (now_t_ > delta_t) {
            if (fetch_proceed(step, act_index_++)) { *setter = false; addKFS(); attr_.is_kfs_raised = true; }
            else last_t_ = DWT_GetTimeline_s();
        }
    }
    // placing_kfs 模块化实现
    void run_placing_kfs(UNLOAD_TYPE layer) {
        now_t_ = DWT_GetTimeline_s() - last_t_;  // now_t记录以last_t为基准的相对时间
        float delta_t;
        bool* setter;
        if (attr_.is_placing_to_layer3) {
            using namespace arm_actions_config::place3_proceed;
            switch (layer) {
                case UNLOAD_TYPE::LOW: { delta_t = kfs_1[act_index_].delta_t; setter = &attr_.is_placing_kfs_L; break; }
                case UNLOAD_TYPE::MEDIUM: { delta_t = kfs_2[act_index_].delta_t; setter = &attr_.is_placing_kfs_M; break; }
                case UNLOAD_TYPE::TOP: { delta_t = kfs_3[act_index_].delta_t; setter = &attr_.is_placing_kfs_T; break; }
            }
        } else {
            using namespace arm_actions_config::place_proceed;
            switch (layer) {
                case UNLOAD_TYPE::LOW: { delta_t = kfs_1[act_index_].delta_t; setter = &attr_.is_placing_kfs_L; break; }
                case UNLOAD_TYPE::MEDIUM: { delta_t = kfs_2[act_index_].delta_t; setter = &attr_.is_placing_kfs_M; break; }
                case UNLOAD_TYPE::TOP: { delta_t = kfs_3[act_index_].delta_t; setter = &attr_.is_placing_kfs_T; break; }
            }
        }
        delta_t += DELTA_DELAY;
        if (now_t_ > delta_t) {
            if (place_proceed(act_index_++)) {
                attr_.is_kfs_raised = false;
                attr_.is_placing_to_layer3 = false;
                rmvKFS();
                *setter = false;
            }
            else last_t_ = DWT_GetTimeline_s();
        }
    }
    // place_releasing 模块化实现
    void run_place_releasing() {
        now_t_ = DWT_GetTimeline_s() - last_t_;  // now_t记录以last_t为基准的相对时间
        float delta_t = arm_actions_config::place_release_proceed[act_index_].delta_t;
        delta_t += DELTA_DELAY;
        if (now_t_ > delta_t) {
            if (place_release_proceed(act_index_++)) attr_.is_place_releasing = false;
            else last_t_ = DWT_GetTimeline_s();
        }
    }
    // loading_kfs 模块化实现
    void run_loading_kfs() {
        now_t_ = DWT_GetTimeline_s() - last_t_;  // now_t记录以last_t为基准的相对时间
        float delta_t;
        using namespace arm_actions_config::load_kfs_proceed;
        switch (get_kfs_amount()) {
            case 1: { delta_t = kfs_0[act_index_].delta_t; break; }
            case 2: { delta_t = kfs_1[act_index_].delta_t; break; }
        }
        delta_t += DELTA_DELAY;
        if (now_t_ > delta_t) {
            if (load_kfs_proceed(act_index_++)) { attr_.is_loading_kfs = false; attr_.is_kfs_raised = false; }
            else last_t_ = DWT_GetTimeline_s();
        }
    }
    // dropping_kfs 模块化实现
    void run_dropping_kfs() {
        now_t_ = DWT_GetTimeline_s() - last_t_;  // now_t记录以last_t为基准的相对时间
        float delta_t;
        using namespace arm_actions_config::drop_kfs_proceed;
        switch (get_kfs_amount()) {
            case 1: case 2: { delta_t = kfs_12[act_index_].delta_t; break; }
            case 3: { delta_t = kfs_3[act_index_].delta_t; break; }
        }
        delta_t += DELTA_DELAY;
        if (now_t_ > delta_t) {
            if (drop_kfs_proceed(act_index_++)) { attr_.is_dropping_kfs = false; attr_.is_kfs_raised = false; }
            else last_t_ = DWT_GetTimeline_s();
        }
    }

    // 启动时初始化
    void run_starting() {
        now_t_ = DWT_GetTimeline_s() - last_t_;  // now_t记录以last_t为基准的相对时间
        float delta_t = arm_actions_config::start_proceed[act_index_].delta_t;
        delta_t += DELTA_DELAY;
        if (now_t_ > delta_t) {
            if (start_proceed(act_index_++)) { attr_.is_starting = false; }
            else last_t_ = DWT_GetTimeline_s();
        }
    }



    // 状态属性的getter与setter
    inline ArmAttr& get_attr() { return attr_; }
    
private:
    DM43xxMotor &arm_lift_;
    MotorBase &arm_rotate_;
    MotorBase &arm_expand_;
    DM43xxMotor &arm_flip_;

    uint8_t kfs_num_{0};

    // Arm状态属性
    ArmAttr attr_;

    uint8_t act_index_ = 0;  // 用于记录动作索引
    float now_t_ = 0.0f;  // 当前动作相对时刻记录
    float last_t_ = 0.0f;  // 上一次动作相对时刻记录

};

void armTask(void *argument);
