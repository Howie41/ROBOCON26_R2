/**
 * @file state_machine_task.cpp
 * @author zhy (Howie41)
 * @brief 状态机任务
 * @date 2026-05-24
 */

#include "cmsis_os2.h"
#include <atomic>
#include <cstdint>

#include "state_machine_task.h"
#include "NavProtocol.hpp"
#include "infrared_com.hpp"
#include "topic_pool.h"
#include "topics.hpp" 
#include "waypoint_navigator.hpp"
#include "chassis_task.h"
osThreadId_t StateMachineTaskHandle;

static std::atomic<RobotState> current_state{RobotState::begin};

TypedTopicSubscriber<pub_infrared_msg> infrared_sub(InfraredModule::INFRARED_MSG_TOPIC, 1);
TypedTopicSubscriber<pub_qr_code_parsed> qr_code_sub("qr_code_parsed", 1);

/**
 * @brief 等待直到条件满足
 * @param condition 条件函数，传一个匿名函数就行，返回布尔值
 * @param delay_ms 多久检查一次条件
 */
template <typename T>
void wait_until(T &&condition, uint32_t delay_ms = 100U) {
    while (!condition()) {
        osDelay(delay_ms);
    }
}

/**
 * @brief 等待直到条件满足或超时
 * @param condition 条件函数，传一个匿名函数就行，返回布尔值
 * @param timeout_ms 超时时间
 * @param delay_ms 多久检查一次条件
 * @return true 条件满足，false 超时
 */
template <typename T>
bool wait_until_timeout_or(T &&condition, uint32_t timeout_ms, uint32_t delay_ms = 100U) {
    const uint32_t start = osKernelGetTickCount();
    while (!condition()) {
        if ((osKernelGetTickCount() - start) >= timeout_ms) {
            return false;
        }
        osDelay(delay_ms);
    }
    return true;
}

void change_state_to(RobotState new_state) {
    current_state.store(new_state);
}

// 这个函数必须在任务环境里调用
bool move_to_pos(int16_t x, int16_t y, int16_t yaw, uint32_t timeout_ms = 0) {
    taskENTER_CRITICAL();
    nav_control::target_x = x;
    nav_control::target_y = y;
    nav_control::target_yaw = yaw;
    nav_control::auto_enabled = true;
    nav_control::arrived = false;
    nav_control::target_active = true;
    nav_control::arrival_reported = false;
    nav_control::resetAllPIDs();
    taskEXIT_CRITICAL();

    if (timeout_ms == 0) {
      wait_until([&]() { return nav_control::arrived; });
      return true;
    }
    return wait_until_timeout_or([&]() { return nav_control::arrived; }, timeout_ms);
}

/**
 * @brief 清空之前的命令，避免误触发
 * @note 两个Subscriber的长度都是1，各自TryGet一次就能清空之前的命令了
 * @note 不要放入 wait_until，只清理一次就好了
 */
void clean_previous_cmd() {
    pub_infrared_msg temp_im{};
    pub_qr_code_parsed temp_qr{};
    infrared_sub.TryGet(&temp_im);
    qr_code_sub.TryGet(&temp_qr);
}

/**
 * @brief 获取来自R1的命令
 * @return 0x00 表示没有命令，其余值表示实际收到的命令
 * @note 调用前请用 clean_previous_cmd() 清空之前的命令，避免误触发
 * @note 如果二维码和红外都有命令，二维码的命令优先
 */
uint8_t get_cmd_from_r1() {
    uint8_t cmd{0x00};
    pub_infrared_msg infrared_msg{.data = 0x00};
    pub_qr_code_parsed qr_code_msg{.data = 0x00};

    // 先取红外（作为默认），再用二维码覆盖
    if (infrared_sub.TryGet(&infrared_msg)) {
        cmd = infrared_msg.data;
    }
    if (qr_code_sub.TryGet(&qr_code_msg)) {
        cmd = qr_code_msg.data;
    }
    return cmd;
}

void stateMachineTask(void *argument) {
    for (;;) {
        switch (current_state.load()) {
        #ifdef MATCH_CWTY /** ========== 崇武探幽 单项赛 ========== */

            case RobotState::begin: {
                break;
            }

            case RobotState::go_to_SHR: {
                break;
            }

            case RobotState::aim_at_weapon: {
                break;
            }

            case RobotState::catch_weapon: {
                break;
            }

            case RobotState::rotate_weapon_claw: {
                break;
            }

            // R2松开武器头夹爪，等待操作手决策，决定是否拼装新的武器
            case RobotState::wait_for_cmd: {
                clean_previous_cmd();
                wait_until([&]() -> bool {
                    switch (get_cmd_from_r1()) {
                        case 0x1A: // 夹取新的武器头
                            change_state_to(RobotState::go_to_SHR);
                            return true;
                        case 0x1B: // 进入梅林
                            change_state_to(RobotState::go_to_MF);
                            return true;
                        default:
                            return false;
                    }
                });
                break;
            }

        #elif MATCH_JGCB /** ========== 九宫藏宝 单项赛 ========== */


        #endif /** ============================================= */
            
            default: // 不应该到达这里
                break;

        }

        osDelay(1);
    }
}
