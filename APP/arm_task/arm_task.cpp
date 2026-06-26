/**
 * @file arm_task.cpp
 * @author FunFer
 * @brief 取矿任务实现
 * @version 0.1
 * @date 2026-05-26
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "arm_task.hpp"

#include "Motor.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "logger.hpp"
#include "bsp_dwt.h"


osThreadId_t Arm_TaskHandle;


static TypedTopicSubscriber<pub_arm_cmd> arm_cmd_sub("arm_cmd", 8);
static pub_arm_cmd arm_cmd{};

extern Arm arm;  // 取矿机构实例

static uint8_t act_index = 0;  // 用于记录动作索引

static float now_t = 0.0f;  // 当前相对时刻记录
static float last_t = 0.0f;  // 上一次相对时刻记录

static uint8_t flag = 0;  // 当前功能状态值
static uint8_t test_flag = 0;  // 上位机进行速度规划调试用时的flag

namespace arm_action {

/**
 * @brief 吸取并放入对应层高的kfs，自动处理存入kfs的数量
 * @note 伸出、吸取、抬起、伸入储存区、释放、恢复默认、kfs_amount+1
 * @param 参数接收: 1, 2, -1, 0的输入，对应+200, +400, -200, 0高度的kfs
 */
void load_kfs(int8_t step) { fetch_step(step); }

/**
 * @brief 取出kfs，自动识别当前kfs数量，从对应的高度取出(取最外层)
 * @note 抬起、伸入储存区、吸取、抬起、伸出、kfs_amount-1
 * @param -1:自动识别高度，1,2,3:指定高度
 */
void unload_kfs(int8_t level = -1) { place_kfs(level); }

/**
 * @brief 承接unload_kfs，释放kfs并恢复默认动作
 */
void release_kfs() { place_release(); }

}


// 重置计时器
void reset_timeline() {
    act_index = 0;
    last_t = DWT_GetTimeline_s();
}


void fetch_step(int8_t step) { 
    if (arm.get_kfs_amount() == 3) return;
    reset_timeline();
    switch (step) {
        case 1: { arm.set_is_fetching_step_M(true); break; }
        case 2: { arm.set_is_fetching_step_H(true); break; }
        case -1: { arm.set_is_fetching_step_L(true); break; }
        case 0: { arm.set_is_fetching_step_P(true); break; }
    }
}

void place_kfs(int8_t kfs_layer = -1) {
    if (arm.get_kfs_amount() == 0) return;
    reset_timeline();
    switch (kfs_layer) {
        case -1: { place_kfs(arm.get_kfs_amount()); break; }
        case 1: { arm.set_is_placing_kfs_L(true); break; }
        case 2: { arm.set_is_placing_kfs_M(true); break; }
        case 3: { arm.set_is_placing_kfs_H(true); break; }
    }
}

void place_release() {
    reset_timeline();
    arm.set_is_place_releasing(true);
}


void armTask(void *argument) {
    arm.reset();

    // fetching_step 模块化实现
    auto run_fetching_step = [&](int8_t step) -> void {
        now_t = DWT_GetTimeline_s() - last_t;  // now_t记录以last_t为基准的相对时间
        float delta_t;
        void (Arm::*setter)(bool);
        using namespace arm_actions_config::fetch_proceed;
        auto get_dt_by_kfs = [&](auto& kfs_0, auto& kfs_1, auto& kfs_2) -> float {
            switch (arm.get_kfs_amount()) {
                case 0: return kfs_0[act_index].delta_t;
                case 1: return kfs_1[act_index].delta_t;
                case 2: return kfs_2[act_index].delta_t;
                default: return 0.0f;
            }
        };
        switch (step) {
            case 2: { delta_t = get_dt_by_kfs(step_H::kfs_0, step_H::kfs_1, step_H::kfs_2); setter = &Arm::set_is_fetching_step_H; break; }
            case 1: { delta_t = get_dt_by_kfs(step_M::kfs_0, step_M::kfs_1, step_M::kfs_2); setter = &Arm::set_is_fetching_step_M; break; }
            case -1: { delta_t = get_dt_by_kfs(step_L::kfs_0, step_L::kfs_1, step_L::kfs_2); setter = &Arm::set_is_fetching_step_L; break; }
            case 0: { delta_t = get_dt_by_kfs(step_P::kfs_0, step_P::kfs_1, step_P::kfs_2); setter = &Arm::set_is_fetching_step_P; break; }
        }
        if (now_t > delta_t) {
            if (arm.fetch_proceed(step, act_index++)) { (arm.*setter)(false); arm.addKFS(); }
            else last_t = DWT_GetTimeline_s();
        }
    };

    // placing_kfs 模块化实现
    auto run_placing_kfs = [&](uint8_t layer) -> void {
        now_t = DWT_GetTimeline_s() - last_t;  // now_t记录以last_t为基准的相对时间
        float delta_t;
        void (Arm::*setter)(bool);
        using namespace arm_actions_config::place_proceed;
        switch (layer) {
            case 1: { delta_t = kfs_1[act_index].delta_t; setter = &Arm::set_is_placing_kfs_L; break; }
            case 2: { delta_t = kfs_2[act_index].delta_t; setter = &Arm::set_is_placing_kfs_M; break; }
            case 3: { delta_t = kfs_3[act_index].delta_t; setter = &Arm::set_is_placing_kfs_H; break; }
        }
        if (now_t > delta_t) {
            if (arm.place_proceed(act_index++)) { (arm.*setter)(false); arm.rmvKFS(); }
            else last_t = DWT_GetTimeline_s();
        }
    };

    // 任务大循环
    for (;;) {
        // fetching_step 类
        if (arm.get_is_fetching_step_H()) run_fetching_step(2);  // 抓取高台阶
        else if (arm.get_is_fetching_step_M()) run_fetching_step(1);  // 抓取中台阶
        else if (arm.get_is_fetching_step_L()) run_fetching_step(-1);  // 抓取低台阶
        else if (arm.get_is_fetching_step_P()) run_fetching_step(0);  // 抓取平地
        // placing_kfs 类
        else if (arm.get_is_placing_kfs_L()) run_placing_kfs(1);  // 放置最底下的KFS进第二层
        else if (arm.get_is_placing_kfs_M()) run_placing_kfs(2);  // 放置中间层的KFS进第二层
        else if (arm.get_is_placing_kfs_H()) run_placing_kfs(3);  // 放置最上面的KFS进第二层
        // place_releasing 类
        else if (arm.get_is_place_releasing()) {  // 释放KFS，回归默认姿态
            now_t = DWT_GetTimeline_s() - last_t;  // now_t记录以last_t为基准的相对时间
            if (now_t > arm_actions_config::place_release_proceed[act_index].delta_t) {
                if (arm.place_proceed(act_index++)) arm.set_is_place_releasing(false);
                else last_t = DWT_GetTimeline_s();
            }
        }

        // 测试用（Xbox手柄）
        if (arm_cmd_sub.TryGet(&arm_cmd) || test_flag) {
            if (arm_cmd.update) flag++;
            if (arm_cmd.fetch || test_flag) {
                switch (flag) {
                    case 1:
                        fetch_step(1);
                        break;
                    case 2:
                        fetch_step(2);
                        break;
                    case 3:
                        fetch_step(-1);
                        break;
                    case 4:
                        fetch_step(0);
                        break;
                    case 5:
                        place_kfs();
                        break;
                    case 6:
                        place_release();
                        break;
                }
                flag = 0;
                test_flag = 0;
            }
        }

        osDelay(1);
    }
}

