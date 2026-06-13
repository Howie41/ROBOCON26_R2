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
#include "com_config.h"
#include "NavProtocol.hpp"
#include "infrared_com.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "waypoint_navigator.hpp"
#include "chassis_task.h"
osThreadId_t StateMachineTaskHandle;

static std::atomic<RobotState> current_state{RobotState::begin};
static std::atomic<PathCmd> current_path_cmd{PathCmd::unknown}; // 初始值对下位机来说没有意义

TypedTopicSubscriber<pub_qr_code_parsed> qr_code_sub("qr_code_parsed", 1);

TypedTopicSubscriber<PathCmd> path_cmd_sub("pc_path_cmd", 1); // 接收路径规划cmd
TypedTopicPublisher<bool> path_cmd_request_pub("pc_path_cmd_request"); // 请求路径规划cmd

static struct Waypoint wp_init{-550, 150, 0};
static struct Waypoint wp_before_MF{2280, 1600, 0};
static struct Waypoint wp_corridor{2050, 4000, 0};
static struct Waypoint wp_before_uphill{8860, 3760, 0};
static struct Waypoint wp_after_uphill{11930, 3760, 0};
static struct Waypoint wp_before_rotate{11830, 1980, 0};
static struct Waypoint wp_after_rotate{11350, 1860, -90};
static struct Waypoint wp_grid{11170, -980, -90};

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
    nav_control::arrived.store(false);
    nav_control::target_active = true;
    nav_control::arrival_reported = false;
    nav_control::resetAllPIDs();
    taskEXIT_CRITICAL();

    if (timeout_ms == 0) {
      wait_until([&]() { return nav_control::arrived.load(); });
      return true;
    }
    return wait_until_timeout_or([&]() { return nav_control::arrived.load(); }, timeout_ms);
}

bool move_to_pos(const Waypoint &wp, uint32_t timeout_ms = 0) {
    return move_to_pos(wp.x, wp.y, wp.yaw, timeout_ms);
}

/**
 * @brief 清空之前的命令，避免误触发
 * @note 两个Subscriber的长度都是1，各自TryGet一次就能清空之前的命令了
 * @note 不要放入 wait_until，只清理一次就好了
 */
void clean_previous_cmd() {
    pub_qr_code_parsed temp_qr{};
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
    pub_qr_code_parsed qr_code_msg{.data = 0x00};

    // 先取红外（作为默认），再用二维码覆盖
    auto ir_result = infrared_group.tryGet();
    if (ir_result.has_value()) {
        cmd = static_cast<uint8_t>(ir_result.value().data);
    }
    if (qr_code_sub.TryGet(&qr_code_msg)) {
        cmd = qr_code_msg.data;
    }
    return cmd;
}

volatile bool triggered{false};

void stateMachineTask(void *argument) {
    for (;;) {
        switch (current_state.load()) {
        #ifdef MATCH_CWTY /** ========== 崇武探幽 单项赛 ========== */

            case RobotState::begin: {
                wait_until([&]() -> bool { return triggered; });
                triggered = false;
                change_state_to(RobotState::go_to_SHR);
                break;
            }

            case RobotState::go_to_SHR: {
                move_to_pos(wp_corridor, 0);
                change_state_to(RobotState::aim_at_weapon);
                break;
            }

            case RobotState::aim_at_weapon: {
                move_to_pos(wp_before_uphill, 0);
                change_state_to(RobotState::catch_weapon);
                break;
            }

            case RobotState::catch_weapon: {
                move_to_pos(wp_after_uphill, 0);
                change_state_to(RobotState::rotate_weapon_claw);
                break;
            }

            case RobotState::rotate_weapon_claw: {
                move_to_pos(wp_before_rotate, 0);
                change_state_to(RobotState::wait_for_cmd);
                break;
            }

            case RobotState::wait_for_cmd: {
                // clean_previous_cmd();
                // wait_until([&]() -> bool {
                //     switch (get_cmd_from_r1()) {
                //         case 0x1A: // 夹取新的武器头
                //             change_state_to(RobotState::go_to_SHR);
                //             return true;
                //         case 0x1B: // 进入梅林
                //             change_state_to(RobotState::go_to_MF_entrance);
                //             return true;
                //         default:
                //             return false;
                //     }
                // });
                move_to_pos(wp_after_rotate, 0);
                change_state_to(RobotState::test1);
                break;
            }

            case RobotState::test1: {
                move_to_pos(wp_grid, 0);
                change_state_to(RobotState::begin);
                break;
            }

            case RobotState::go_to_MF_entrance: {
                break;
            }

            case RobotState::request_for_path_cmd: {
                path_cmd_request_pub.Publish(true); // 发一次 PathCmd::request

                PathCmd cmd;
                wait_until([&]() -> bool {
                    return path_cmd_sub.TryGet(&cmd);
                });

                current_path_cmd.store(cmd); // 给其他后续状态读取
                switch (cmd) {
                    case PathCmd::move_forward:
                    case PathCmd::move_backward:
                    case PathCmd::turn_left_90:
                    case PathCmd::turn_right_90:
                    case PathCmd::move_left:
                    case PathCmd::move_right:
                        change_state_to(RobotState::execute_chassis_action);
                        break;
                    case PathCmd::grab_low_r2kfs:
                    case PathCmd::grab_mid_r2kfs:
                    case PathCmd::grab_high_r2kfs:
                    case PathCmd::drop_and_grab_new_kfs:
                        change_state_to(RobotState::execute_arm_action);
                        break;
                    case PathCmd::no_more_commands:
                        change_state_to(RobotState::go_to_MF_exit);
                        break;
                    default:
                        break;
                }
            }

            case RobotState::execute_chassis_action: {
                break;
            }

            case RobotState::execute_arm_action: {
                break;
            }

            case RobotState::go_to_MF_exit: {
                break;
            }

            case RobotState::stop: {
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
