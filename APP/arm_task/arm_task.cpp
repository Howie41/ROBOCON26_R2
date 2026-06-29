/**
 * @file arm_task.cpp
 * @author FunFer
 * @brief 取矿任务实现
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

#include "arm_task.hpp"
#include <optional>
#include "Motor.hpp"
#include "com_config.h"
#include "pid_controller.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "logger.hpp"
#include "bsp_dwt.h"
#include <optional>


osThreadId_t Arm_TaskHandle;


static TypedTopicSubscriber<pub_arm_cmd> arm_cmd_sub("arm_cmd", 8);
static pub_arm_cmd arm_cmd{};

extern Arm arm;  // 取矿机构实例

static uint8_t flag = 0;  // 当前功能状态值
static uint8_t test_flag = 0;  // 上位机进行速度规划调试用时的flag

namespace arm_action {
/**
 * @brief 吸取并放入对应层高的kfs，自动处理存入kfs的数量
 * @note 伸出、吸取、抬起、伸入储存区、释放、恢复默认、kfs_amount+1
 * @param 参数接收: LOAD_TYPE::MEDIUM, LOAD_TYPE::HIGH, LOAD_TYPE::LOW, LOAD_TYPE::PLAIN, LOAD_TYPE::TOP的输入，对应+200, +400, -200, 0高度与举高高的kfs
 */
bool load_kfs(LOAD_TYPE step) { return arm.fetch_step(step); }
/**
 * @brief 取出kfs，自动识别当前kfs数量，从对应的高度取出(取最外层)
 * @note 抬起、伸入储存区、吸取、抬起、伸出、kfs_amount-1
 * @param 默认std::nullopt:自动识别高度，UNLOAD_TYPE::LOW, UNLOAD_TYPE::MEDIUM, UNLOAD_TYPE::HIGH, UNLOAD_TYPE::TOP : 指定高度
 */
bool unload_kfs(std::optional<UNLOAD_TYPE> level = std::nullopt) { return arm.place_kfs(level); }
/**
 * @brief 承接unload_kfs，释放kfs并恢复默认动作
 */
void release_kfs() { arm.place_release(); }
/**
 * @brief 将平地KFS举高高
 */
void raise_kfs_top() { arm.raise_kfs(); }
}



void armTask(void *argument) {
    arm.start();

    // 任务大循环
    for (;;) {
        // fetching_step 类
        if (arm.get_is_fetching_step_H()) arm.run_fetching_step(LOAD_TYPE::HIGH);  // 抓取高台阶
        else if (arm.get_is_fetching_step_M()) arm.run_fetching_step(LOAD_TYPE::MEDIUM);  // 抓取中台阶
        else if (arm.get_is_fetching_step_L()) arm.run_fetching_step(LOAD_TYPE::LOW);  // 抓取低台阶
        else if (arm.get_is_fetching_step_P()) arm.run_fetching_step(LOAD_TYPE::PLAIN);  // 抓取平地
        else if (arm.get_is_fetching_step_T()) arm.run_fetching_step(LOAD_TYPE::TOP);  // 抓取举高高
        // placing_kfs 类
        else if (arm.get_is_placing_kfs_L()) arm.run_placing_kfs(UNLOAD_TYPE::LOW);  // 放置最底层的KFS进第一层下的KFS进第二层
        else if (arm.get_is_placing_kfs_M()) arm.run_placing_kfs(UNLOAD_TYPE::MEDIUM);  // 放置中间层的KFS进第二层
        else if (arm.get_is_placing_kfs_H()) arm.run_placing_kfs(UNLOAD_TYPE::HIGH);  // 放置最上面的KFS进第二层
        else if (arm.get_is_placing_kfs_T()) arm.run_placing_kfs(UNLOAD_TYPE::TOP);  // 放置举高高的KFS进第二层
        // place_releasing 类
        else if (arm.get_is_place_releasing()) arm.run_place_releasing();  // 释放KFS，回归默认姿态
        // raising_kfs 类
        else if (arm.get_is_raising_kfs()) arm.run_raising_kfs();  // 举高高KFS
        // starting 类
        else if (arm.get_is_starting()) arm.run_starting();  // 启动时初始化



        // 测试用（Xbox手柄）
        if (arm_cmd_sub.TryGet(&arm_cmd) || test_flag) {
            if (arm_cmd.update) flag++;
            if (arm_cmd.fetch || test_flag) {
                switch (flag) {
                    case 1:
                        arm.fetch_step(LOAD_TYPE::MEDIUM);
                        break;
                    case 2:
                        arm.fetch_step(LOAD_TYPE::HIGH);
                        break;
                    case 3:
                        arm.fetch_step(LOAD_TYPE::LOW);
                        break;
                    case 4:
                        arm.fetch_step(LOAD_TYPE::PLAIN);
                        break;
                    case 5:
                        arm.raise_kfs();
                        break;
                    case 6:
                        arm.fetch_step(LOAD_TYPE::TOP);
                        break;
                    case 7:
                        arm.place_kfs();
                        break;
                    case 8:
                        arm.place_release();
                        break;
                }
                flag = 0;
                test_flag = 0;
            }
        }

        osDelay(1);
    }
}

