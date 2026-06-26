/**
 * @file state_machine_task.cpp
 * @author zhy (Howie41)
 * @brief 状态机任务
 * @date 2026-05-24
 */

#include "arm_task.hpp"
#include "cmsis_os2.h"
#include <atomic>
#include <cstdint>

#include "state_machine_task.h"
#include "com_config.h"
#include "NavProtocol.hpp"
#include "memory_map.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "chassis_task.h"

osThreadId_t StateMachineTaskHandle;
extern Arm arm;  // 取矿机构实例

namespace waypoint {
    typedef struct {
        int16_t x;
        int16_t y;
        int16_t yaw;
    } point;

    [[maybe_unused]] point init{-550, 150, 0};
    [[maybe_unused]] point before_mf{2280, 1600, 0};
    [[maybe_unused]] point corridor{2050, 4000, 0};
    [[maybe_unused]] point before_uphill{8860, 3760, 0};
    [[maybe_unused]] point after_uphill{11930, 3760, 0};
    [[maybe_unused]] point before_rotate{11830, 1980, 0};
    [[maybe_unused]] point after_rotate{11350, 1860, -90};
    [[maybe_unused]] point grid{11170, -980, -90};
}

volatile bool begin_signal{false};

class StateMachine {
public:
    StateMachine() = default;
    ~StateMachine() = default;
    // 禁止拷贝和移动
    // StateMachine(const StateMachine&) = delete;
    // StateMachine& operator=(const StateMachine&) = delete;
    // StateMachine(StateMachine&&) = delete;
    // StateMachine& operator=(StateMachine&&) = delete;

    // static StateMachine& instance() {
    //     RAM_D1_ATTR static StateMachine instance;
    //     return instance;
    // }
    void run() {
        switch (current_state_.load()) {
        #ifdef MATCH_CWTY /** ========== 崇武探幽 单项赛 ========== */

            case robot_state::begin: {
                wait_until([&]() -> bool { return begin_signal; });
                begin_signal = false;
                change_state_to(robot_state::request_for_path_cmd);
                break;
            }

            case robot_state::go_to_shr: {
                // TODO: 一区逻辑
                break;
            }

            case robot_state::aim_at_weapon: {
                break;
            }

            case robot_state::catch_weapon: {
                break;
            }

            case robot_state::rotate_weapon_claw: {
                break;
            }

            case robot_state::go_to_mf_entrance: {
                break;
            }

            case robot_state::request_for_path_cmd: {
                path_cmd_request_pub_.Publish(true); // 发一次 path_cmd::code::request

                path_cmd::code cmd;
                wait_until([&]() -> bool {
                    return path_cmd_sub_.TryGet(&cmd);
                });

                current_path_cmd_.store(cmd); // 给其他后续状态读取
                switch (cmd) {
                    case path_cmd::code::move_forward:
                    case path_cmd::code::move_backward:
                    case path_cmd::code::turn_left_90:
                    case path_cmd::code::turn_right_90:
                    case path_cmd::code::move_left:
                    case path_cmd::code::move_right:
                        change_state_to(robot_state::execute_chassis_action);
                        break;
                    case path_cmd::code::grab_low_r2kfs:
                    case path_cmd::code::grab_mid_r2kfs:
                    case path_cmd::code::grab_high_r2kfs:
                    case path_cmd::code::drop_and_grab_new_kfs:
                        change_state_to(robot_state::execute_arm_action);
                        break;
                    case path_cmd::code::no_more_commands:
                        change_state_to(robot_state::go_to_mf_exit);
                        break;
                    default:
                        break;
                }

                break;
            }

            case robot_state::execute_chassis_action: {
                path_cmd::code executing_cmd = current_path_cmd_.load();
                // chassis_action 是耗时函数 内含阻塞等待逻辑
                switch (executing_cmd) {
                    case path_cmd::code::move_forward:
                        chassis_action::start_climb_upstairs();
                        break;
                    case path_cmd::code::move_backward:
                        chassis_action::start_climb_downstairs();
                        break;
                    case path_cmd::code::turn_left_90:
                        chassis_action::turn_left_90_deg();
                        break;
                    case path_cmd::code::turn_right_90:
                        chassis_action::turn_right_90_deg();
                        break;
                    case path_cmd::code::move_left:
                        // TODO: 实现向左平移
                        break;
                    case path_cmd::code::move_right:
                        // TODO: 实现向右平移
                        break;
                    default:
                        break;
                }
                current_path_cmd_.store(path_cmd::code::unknown); // 清空当前命令
                change_state_to(robot_state::request_for_path_cmd);
                break;
            }

            case robot_state::execute_arm_action: {

                path_cmd::code executing_cmd = current_path_cmd_.load();

                switch (executing_cmd) {
                    case path_cmd::code::grab_low_r2kfs:
                        arm_action::load_kfs(-1);
                        break;
                    case path_cmd::code::grab_mid_r2kfs:
                        arm_action::load_kfs(1);
                        break;
                    case path_cmd::code::grab_high_r2kfs:
                        arm_action::load_kfs(2);
                        break;
                    case path_cmd::code::drop_and_grab_new_kfs:
                        break;
                    default:
                        break;
                }
                wait_until([]() -> bool {
                    return !arm.get_is_fetching_step_L() && !arm.get_is_fetching_step_P()
                            && !arm.get_is_fetching_step_M() && !arm.get_is_fetching_step_H();
                });

                current_path_cmd_.store(path_cmd::code::unknown); // 清空当前命令
                change_state_to(robot_state::request_for_path_cmd);
                break;
            }

            case robot_state::go_to_mf_exit: {
                break;
            }

            case robot_state::stop: {
                break;
            }

        #elif MATCH_JGCB /** ========== 九宫藏宝 单项赛 ========== */


        #endif /** ============================================= */
            
            default: // 不应该到达这里
                break;

        }
    }

private:

    enum class robot_state: uint8_t {
    #ifdef MATCH_CWTY /** ========== 崇武探幽 单项赛 ========== */
        // 武馆
        begin = 0,                     // 启动
        go_to_shr,                     // 前往端头架
        aim_at_weapon,                 // 夹爪对准对应武器头
        catch_weapon,                  // 夹爪夹取武器
        rotate_weapon_claw,            // 夹爪反转
        wait_for_cmd,                  // 等待R1指令 决定继续夹取or前往梅林

        // 梅林
        go_to_mf_entrance,             // 前往梅林入口
        request_for_path_cmd,          // 请求路径规划命令
        execute_chassis_action,        // 执行底盘动作
        execute_arm_action,            // 执行取矿机构动作
        go_to_mf_exit,                 // 前往梅林出口
        stop                           // 停止

    #elif MATCH_JGCB /** ========== 九宫藏宝 单项赛 ========== */
        begin = 0,                     // 启动
        go_to_arena,                   // 上坡、前往竞技场
        go_to_load_kfs,                // 前往距斜坡最近的KFS前
        load_kfs,                      // 装载KFS
        wait_for_place_mid_kfs_cmd,    // 等待放置中层KFS的指令

        go_to_tic_tac_toe,             // 前往九宫格前
        request_for_kfs_location,      // 请求KFS放置位置
        go_to_kfs_location,            // 前往KFS放置位置
        place_kfs,                     // 放置KFS
        go_to_combination_area,        // 前往合体点位
        wait_for_combination_cmd,      // 等待合体指令
        begin_combination,             // 合体

        unload_kfs,                    // 取出KFS并手持
        wait_for_place_hi_kfs_cmd,     // 等待放置高层KFS的指令
        release_kfs,                   // 释放KFS 

        stop                           // 停止
    #endif
    };

    std::atomic<robot_state> current_state_{robot_state::begin};
    std::atomic<path_cmd::code> current_path_cmd_{path_cmd::code::unknown}; // 初始值对下位机来说没有意义

    TypedTopicSubscriber<pub_qr_code_parsed> qr_code_sub_{"qr_code_parsed", 1};

    TypedTopicSubscriber<path_cmd::code> path_cmd_sub_{"pc_path_cmd", 1}; // 接收路径规划cmd
    TypedTopicPublisher<bool> path_cmd_request_pub_{"pc_path_cmd_request"}; // 请求路径规划cmd

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

    void change_state_to(robot_state new_state) {
        current_state_.store(new_state);
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
            wait_until([]() { return nav_control::arrived; });
            return true;
        } else {
            return wait_until_timeout_or([]() { return nav_control::arrived; }, timeout_ms);
        }
    }

    bool is_loosely_arrived() {
        constexpr int16_t LOOSE_ARRIVED_THRESHOLD = 50; // 单位mm
        int16_t delta_x = std::abs(nav_control::target_x - nav_control::current_x);
        int16_t delta_y = std::abs(nav_control::target_y - nav_control::current_y);
        return (delta_x < LOOSE_ARRIVED_THRESHOLD && delta_y < LOOSE_ARRIVED_THRESHOLD);
    }

    bool move_to_pos(const waypoint::point &wp, uint32_t timeout_ms = 0) {
        return move_to_pos(wp.x, wp.y, wp.yaw, timeout_ms);
    }

    /**
    * @brief 清空之前的命令，避免误触发
    * @note 两个Subscriber的长度都是1，各自TryGet一次就能清空之前的命令了
    * @note 不要放入 wait_until，只清理一次就好了
    */
    void clean_previous_cmd() {
        pub_qr_code_parsed temp_qr{};
        qr_code_sub_.TryGet(&temp_qr);
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
        if (qr_code_sub_.TryGet(&qr_code_msg)) {
            cmd = qr_code_msg.data;
        }
        return cmd;
    }
};

RAM_D1_ATTR static StateMachine state_machine;
void stateMachineTask(void *argument) {
    for (;;) {
        state_machine.run();
        osDelay(1);
    }
}