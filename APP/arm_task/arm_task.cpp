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


osThreadId_t Arm_TaskHandle;

extern LoggerQueue logger_queue;

static TypedTopicSubscriber<pub_arm_cmd> arm_cmd_sub("arm_cmd", 8);
static pub_arm_cmd arm_cmd{};

extern Arm arm;  // 取矿机构实例

static uint8_t flag = 0;  // 当前功能状态值
static uint8_t test_flag = 0;  // 上位机进行速度规划调试用时的flag


namespace arm_action {
/**
 * @brief 吸取对应层高的kfs、kfs_amount+1
 * @note 伸出、吸取、抬起
 * @param 参数接收: LOAD_TYPE::MEDIUM, LOAD_TYPE::HIGH, LOAD_TYPE::LOW, LOAD_TYPE::PLAIN的输入，对应+200, +400, -200, 0高度的kfs
 */
bool raise_kfs(LOAD_TYPE step) { 
    auto result = arm.fetch_step(step);
    if (result) {
        switch (step) {
            case LOAD_TYPE::MEDIUM: { osDelay(1500+2000); break; }
            case LOAD_TYPE::HIGH: { osDelay(1500+2000); break; }
            case LOAD_TYPE::LOW: { osDelay(1500+2800); break; }
            case LOAD_TYPE::PLAIN: { osDelay(1500+1700); break; }
        }
    } else {
        logger_queue.log("ARM\traise_kfs failed!\n");
    }
    return result;
}
/**
 * @brief 取出kfs，默认取最外层
 * @note 抬起、伸入储存区、吸取、抬起、伸出
 * @param std::nullopt:自动识别高度，UNLOAD_TYPE::LOW, UNLOAD_TYPE::MEDIUM, UNLOAD_TYPE::TOP : 指定高度
 */
bool unload_kfs(std::optional<UNLOAD_TYPE> level, bool is_layer3) {
    auto result = arm.place_kfs(level, is_layer3);
    if (result) {
        if (level.has_value()) {
            switch (level.value()) {
                case UNLOAD_TYPE::LOW: { osDelay(1500+4200); break; }
                case UNLOAD_TYPE::MEDIUM: { osDelay(1500+3200); break; }
                case UNLOAD_TYPE::TOP: { osDelay(1500+800); break; }
            }
        } else {
            osDelay(1500+4200);
        }
    } else {
        logger_queue.log("ARM\tunload_kfs failed!\n");
    }
    return result;
}
/**
 * @brief 承接unload_kfs，释放kfs并恢复默认动作、kfs_amount-1
 */
bool release_kfs() {
    auto result = arm.place_release();
    if (result) {
        osDelay(1500+600);
    } else {
        logger_queue.log("ARM\trelease_kfs failed!\n");
    }
    return result;
}
/**
 * @brief 将举起的kfs放入储存
 */
bool load_kfs() { 
    auto result = arm.load_kfs();
    if (result) {
        switch (arm.get_kfs_amount()) {
            case 1: { osDelay(1500+1800); break; }
            case 2: { osDelay(1500+1800); break; }
        }
    } else {
        logger_queue.log("ARM\tload_kfs failed!\n");
    }
    return result;
}
/**
 * @brief 丢掉kfs（举着的kfs）
 */
bool drop_kfs() {
    auto result = arm.drop_kfs();
    if (result) {
        switch (arm.get_kfs_amount()) {
            case 1: case 2: { osDelay(1500+1000); break; }  // 第二层没kfs挡着
            case 3: { osDelay(1500+1600); break; }  // 第二层有kfs挡着，动作链会多一个伸出动作
        }
    } else {
        logger_queue.log("ARM\tdrop_kfs failed!\n");
    }
    return result;
}

} // namespace arm_action



void armTask(void *argument) {
    // arm.start();  // 手动在状态机里调用

    // 任务大循环
    for (;;) {
        osDelay(1);
        if (arm.get_attr().is_starting) arm.run_starting();  // 启动时初始化

        // 测试用（Xbox手柄）
        if (arm_cmd_sub.TryGet(&arm_cmd) || test_flag) {
            if (arm_cmd.update) flag++;
            if (arm_cmd.fetch || test_flag) {
                switch (flag) {
                    case 0:
                        arm.start();
                        break;
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
                        arm.load_kfs();
                        break;
                    case 6:
                        arm.place_kfs();
                        break;
                    case 7:
                        arm.place_kfs(std::nullopt, true);
                        break;
                    case 8:
                        arm.place_release();
                        break;
                    case 9:
                        arm.drop_kfs();
                        break;
                    case 10:
                        arm.fetch();
                        break;
                    case 11:
                        arm.release();
                        break;
                }
                flag = 0;
                test_flag = 0;
            }
        }

        
        if (!arm.get_attr().is_started || arm.get_attr().is_starting) continue;

        // fetching_step 类
        if (arm.get_attr().is_fetching_step_H) arm.run_fetching_step(LOAD_TYPE::HIGH);  // 抓取高台阶
        else if (arm.get_attr().is_fetching_step_M) arm.run_fetching_step(LOAD_TYPE::MEDIUM);  // 抓取中台阶
        else if (arm.get_attr().is_fetching_step_L) arm.run_fetching_step(LOAD_TYPE::LOW);  // 抓取低台阶
        else if (arm.get_attr().is_fetching_step_P) arm.run_fetching_step(LOAD_TYPE::PLAIN);  // 抓取平地
        // placing_kfs 类
        else if (arm.get_attr().is_placing_kfs_L) arm.run_placing_kfs(UNLOAD_TYPE::LOW);  // 放置最底层的KFS进第一层下的KFS进第二层
        else if (arm.get_attr().is_placing_kfs_M) arm.run_placing_kfs(UNLOAD_TYPE::MEDIUM);  // 放置中间层的KFS进第二层
        else if (arm.get_attr().is_placing_kfs_T) arm.run_placing_kfs(UNLOAD_TYPE::TOP);  // 放置举起的KFS进第二层
        // place_releasing 类
        else if (arm.get_attr().is_place_releasing) arm.run_place_releasing();  // 释放KFS，回归默认姿态
        // loading_kfs 类
        else if (arm.get_attr().is_loading_kfs) arm.run_loading_kfs();  // 存入KFS
        // dropping_kfs 类
        else if (arm.get_attr().is_dropping_kfs) arm.run_dropping_kfs();  // 丢弃KFS

    }
}