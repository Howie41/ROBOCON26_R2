/**
 * @file state_machine_task.cpp
 * @author zhy (Howie41)
 * @brief 状态机任务
 * @date 2026-05-24
 */

#include "arm_task.hpp"
#include "cmsis_os2.h"
#include <cmath>
#include <cstdint>
#include <mutex>
#include <sys/_pthreadtypes.h>

#include "state_machine_task.h"
#include "com_config.h"
#include "NavProtocol.hpp"
#include "infrared_com.hpp"
#include "memory_map.h"
#include "tail_claw_task.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "chassis_task.h"
#include "logger.hpp"

osThreadId_t StateMachineTaskHandle;
extern Arm arm;  // 取矿机构实例
extern LoggerQueue logger_queue;

namespace waypoint {
    struct point{
        int16_t x;
        int16_t y;
        int16_t yaw;
    };

    constexpr point init{-550, 150, 0};
    constexpr point before_mf{2280, 1600, 0};
    constexpr point corridor{2050, 4000, 0};
    constexpr point before_uphill{7967, 3760, 0};
    constexpr point after_uphill{10837, 3840, 0};
    constexpr point before_rotate{11830, 1980, 0};
    constexpr point after_rotate{11350, 1860, -90};
    constexpr point grid{11170, -980, -90};

    constexpr point mf_entrance_mid{2060, 1500, 0};
    constexpr point mf_entrance_left{2060, 1500+1200, 0};
    constexpr point mf_entrance_right{2060, 1500-1200, 0};

    constexpr point mf_entrance_mid_close{2085, mf_entrance_mid.y, 0};
    constexpr point mf_entrance_left_close{2085, mf_entrance_left.y, 0};
    constexpr point mf_entrance_right_close{2085, mf_entrance_right.y, 0};

    constexpr point grid_left{10517, -480, -90};
    constexpr point grid_mid{9979, -480, -90};
    constexpr point grid_right{9443, -480, -90};
}

volatile bool begin_signal{false};

static TypedTopicPublisher<pub_chassis_cmd> chassis_stop_cmd_pub("chassis_cmd");
volatile bool paused{false};

class StateMachine {

private:
    int8_t moved_left_right_{0}; // 左右移动计数
    int flag_{0};                 // 武器夹取计数，0=第一次夹取，非0=后续夹取

public:
    StateMachine() = default;
    ~StateMachine() = default;

    void run_cwty() {
        if constexpr (MATCH_TYPE != match_type::CWTY) {
            return;
        }
        
        switch (current_state_) {

            case robot_state::begin: {
                // change_state_to(robot_state::wait_for_decision_cmd);
                // break;
                logger_queue.log("SM ======== BEGIN ========\n");
                wait_until([]() -> bool {
                    return begin_signal;
                });
                logger_queue.log("SM begin signal\n");
                change_state_to(robot_state::go_to_mf_entrance);
                break;

                tail_claw_set_weapon_claw(true);            // 打开夹爪，夹紧武器头
                move_to_pos(-180, -115, 0, 5000);
                change_state_to(robot_state::go_to_shr);
                break;
            }

            case robot_state::go_to_shr: {
                if (flag_ == 0) {
                    move_to_pos(335, -825, 90, 5000);
                } else {
                    move_to_pos(300, -170, 90, 4000U);
                    move_to_pos(735, -830, 90, 5000);
                }
                tail_claw_set_weapon_claw(true); // 打开夹爪，夹紧武器头
                constexpr float target = -59.5f;
                tail_claw_set_roll_target(target);
                wait_until_timeout_or([this, target]() -> bool {
                    tail_claw_update_status();
                    return tail_claw_status_valid_
                        && fabsf(tail_claw_status_cache_.roll_target_deg - target) < 0.5f
                        && tail_claw_status_cache_.roll_arrived;
                }, 3000U, 20U);
                change_state_to(robot_state::aim_at_weapon);
                break;
            }

            case robot_state::aim_at_weapon: {
                change_state_to(robot_state::catch_weapon);
                break;
            }

            case robot_state::catch_weapon: {
                if (flag_ == 0) {
                    move_to_pos(335, -895, 90, 5000);
                    // move_to_pos(735, -905, 90, 5000);
                } else {
                    move_to_pos(735, -905, 90, 8000);
                }
                tail_claw_set_weapon_claw(false);         // 闭合夹爪，夹紧武器头
                osDelay(500);                             // 等待夹爪动作完成，具体时间待调试
                flag_++;
                change_state_to(robot_state::rotate_weapon_claw);
                break;
            }

            case robot_state::rotate_weapon_claw: {
                tail_claw_set_roll_target(2.0f);
                osDelay(1000);
                wait_until_timeout_or([this]() -> bool {
                    tail_claw_update_status();
                    return tail_claw_status_valid_ && tail_claw_status_cache_.roll_arrived;
                }, 3000U, 10U);
                tail_claw_reset_match();
                change_state_to(robot_state::match_rod);
                break;
            }

            case robot_state::match_rod: {
                move_to_pos(300, -170, -90, 4000U);
                change_state_to(robot_state::wait_for_decision_cmd);
                break;
            }

            case robot_state::wait_for_claw_cmd: {
                change_state_to(robot_state::wait_for_decision_cmd);
                break;
            }

            case robot_state::wait_for_decision_cmd: {
                clean_previous_cmd();
                wait_until([this]() -> bool {
                    switch (get_cmd_from_r1()) {
                        case 0x0A: // 松开夹爪
                            tail_claw_set_weapon_claw(true);
                            return false;
                            break;
                        case 0x1A: // 夹取新的武器头
                            change_state_to(robot_state::go_to_shr);
                            return true;
                            break;
                        case 0x1B: // 进梅林
                            change_state_to(robot_state::go_to_mf_entrance);
                            return true;
                            break;
                        default:
                            return false;
                            break;
                    }
                }, 25);
                // tail_claw_set_weapon_claw(true);
                // osDelay(3000);
                change_state_to(robot_state::go_to_mf_entrance);
                break;
            }

            case robot_state::go_to_mf_entrance: {
                move_to_pos(waypoint::mf_entrance_mid);
                tail_claw_set_roll_target(-118.0f);
                change_state_to(robot_state::request_for_path_cmd);
                break;
            }

            case robot_state::request_for_path_cmd: {
                path_cmd_request_pub_.Publish(current_path_cmd_index_); // 发一次 request
                logger_queue.log("SM path cmd requested No.%d...\n", current_path_cmd_index_);

                path_cmd::code cmd;
                auto if_received = wait_until_timeout_or([&]() -> bool {
                    return path_cmd_sub_.TryGet(&cmd);
                }, 3000); // 3秒超时

                if (!if_received) {
                    logger_queue.log("SM path cmd timeout!\n");
                    break; // 超时未收到指令，重发
                }

                logger_queue.log("SM path cmd received No.%d 0x%02X\n", current_path_cmd_index_, static_cast<uint8_t>(cmd));
                current_path_cmd_index_ += 1;
                current_path_cmd_ = cmd; // 给其他后续状态读取

                switch (cmd) {
                    case path_cmd::code::move_forward:
                    case path_cmd::code::move_backward:
                    case path_cmd::code::turn_left_90:
                    case path_cmd::code::turn_right_90:
                    case path_cmd::code::move_left:
                    case path_cmd::code::move_right:
                    case path_cmd::code::turn_around:
                        change_state_to(robot_state::execute_chassis_action);
                        break;
                    case path_cmd::code::grab_low_r2kfs:
                    case path_cmd::code::grab_mid_r2kfs:
                    case path_cmd::code::grab_high_r2kfs:
                    case path_cmd::code::drop_and_grab_new_kfs:
                        change_state_to(robot_state::execute_arm_action);
                        break;
                    case path_cmd::code::no_more_commands:
                        logger_queue.log("SM path cmd end\n");
                        change_state_to(robot_state::go_to_mf_exit);
                        break;
                    default:
                        break;
                }

                break;
            }

            case robot_state::execute_chassis_action: {
                path_cmd::code executing_cmd = current_path_cmd_;
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
                    case path_cmd::code::turn_around:
                        chassis_action::turn_right_180_deg();
                        // chassis_action::start_return_to_center();
                        break;
                    case path_cmd::code::move_left:
                        move_left();
                        break;
                    case path_cmd::code::move_right:
                        move_right();
                        break;
                    default:
                        break;
                }
                current_path_cmd_ = path_cmd::code::unknown; // 清空当前命令
                change_state_to(robot_state::request_for_path_cmd);
                break;
            }

            case robot_state::execute_arm_action: {
                path_cmd::code executing_cmd = current_path_cmd_;

                chassis_action::start_go_to_edge();
                switch (executing_cmd) {
                    case path_cmd::code::grab_low_r2kfs:
                        arm_action::load_kfs(LOAD_TYPE::LOW);
                        break;
                    case path_cmd::code::grab_mid_r2kfs:
                        arm_action::load_kfs(LOAD_TYPE::MEDIUM);
                        break;
                    case path_cmd::code::grab_high_r2kfs:
                        arm_action::load_kfs(LOAD_TYPE::HIGH);
                        break;
                    case path_cmd::code::drop_and_grab_new_kfs:
                        // 忽略这个指令
                        break;
                    default:
                        break;
                }

                // wait_until_timeout_or([&]() -> bool {
                //     return arm.get_is_holding_kfs();
                // }, 8000);
                osDelay(10*1000);

                chassis_action::start_return_to_center();

                current_path_cmd_ = path_cmd::code::unknown; // 清空当前命令
                change_state_to(robot_state::request_for_path_cmd);
                break;
            }

            case robot_state::go_to_mf_exit: {
                move_to_pos(waypoint::before_uphill, 5000);
                change_state_to(robot_state::stop);
                break;
            }

            case robot_state::stop: {
                logger_queue.log("SM ======== STOP ========\n");
                // chassis_stop();
                break;
            }

            default: // 不应该到达这里
                break;
        }
    }
    void run_jgcb() {
        if constexpr (MATCH_TYPE != match_type::JGCB) {
            return;
        }
        switch (current_state_) {
            case robot_state::begin: {
                logger_queue.log("SM ======== BEGIN ========\n");
                wait_until([this]() -> bool {
                    if (get_cmd_from_r1() == 0x2A) {
                        return true;
                    } else {
                        return false;
                    }
                });
                logger_queue.log("SM begin signal\n");
                change_state_to(robot_state::go_to_arena);
                break;
            }

            case robot_state::stop: {
                logger_queue.log("SM ======== STOP ========\n");
                break;
            }

            default: // 不应该到达这里
                break;
        }
    }

private:

    enum class robot_state: uint8_t {
        begin = 0,                     // 启动
        stop,                          // 停止

        // 崇武探幽
        go_to_shr,                     // 前往端头架
        aim_at_weapon,                 // 夹爪对准对应武器头
        catch_weapon,                  // 夹爪夹取武器
        rotate_weapon_claw,            // 夹爪反转
        match_rod,                     // 端头架对齐武器杆
        wait_for_claw_cmd,             // 等待R1指令 决定继续夹取or前往梅林
        wait_for_decision_cmd,         // 等待操作手决策，决定是否拼装新的武器

        go_to_mf_entrance,             // 前往梅林入口
        request_for_path_cmd,          // 请求路径规划命令
        execute_chassis_action,        // 执行底盘动作
        execute_arm_action,            // 执行取矿机构动作
        go_to_mf_exit,                 // 前往梅林出口

        // 九宫藏宝
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
    };

    robot_state current_state_{robot_state::begin};
    uint16_t current_path_cmd_index_{0};
    path_cmd::code current_path_cmd_{path_cmd::code::unknown}; // 初始值对下位机来说没有意义

    TypedTopicSubscriber<pub_qr_code_parsed> qr_code_sub_{"qr_code_parsed", 1};
    // 尾爪武器事件发布（通知上位机发送距离数据）
    TypedTopicPublisher<tail_claw_msg> tail_claw_weapon_event_pub_{"tail_claw_weapon_event"};
    // 尾爪状态订阅
    TypedTopicSubscriber<pub_tail_claw_status> tail_claw_status_sub_{"tail_claw_status", 4};

    TypedTopicSubscriber<path_cmd::code> path_cmd_sub_{"pc_path_cmd", 1}; // 接收路径规划cmd
    TypedTopicPublisher<uint16_t> path_cmd_request_pub_{"pc_path_cmd_request"}; // 请求路径规划cmd

    pub_tail_claw_status tail_claw_status_cache_{};
    bool tail_claw_status_valid_{};
    bool state_machine_view_last_{};

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
        logger_queue.log("SM %d -> %d\n", static_cast<int>(current_state_), static_cast<int>(new_state));
        current_state_ = new_state;
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
        int16_t delta_x = nav_control::target_x - nav_control::current_x;
        int16_t delta_y = nav_control::target_y - nav_control::current_y;
        return (delta_x < LOOSE_ARRIVED_THRESHOLD && delta_x > -LOOSE_ARRIVED_THRESHOLD
             && delta_y < LOOSE_ARRIVED_THRESHOLD && delta_y > -LOOSE_ARRIVED_THRESHOLD);
    }

    bool move_to_pos(const waypoint::point &wp, uint32_t timeout_ms = 0) {
        return move_to_pos(wp.x, wp.y, wp.yaw, timeout_ms);
    }

    /**
     * @brief 强制停止底盘导航
     *
     * 禁用自动导航并发布零速度指令，立即刹停底盘。
     * 可在任何任务上下文中安全调用。
     */
    void chassis_stop() {
        taskENTER_CRITICAL();
        nav_control::auto_enabled = false;
        nav_control::arrived = false;
        nav_control::target_active = false;
        nav_control::arrival_reported = false;
        taskEXIT_CRITICAL();

        // 发布零速度指令，确保底盘物理停止
        pub_chassis_cmd cmd{};
        cmd.nav_mode_ = true;
        chassis_stop_cmd_pub.Publish(cmd);
    }

    /**
     * @brief 更新尾爪状态缓存
     * @return true 表示有新的状态更新
     */
    bool tail_claw_update_status() {
        pub_tail_claw_status status{};
        bool updated = false;

        while (tail_claw_status_sub_.TryGet(&status)) {
            tail_claw_status_cache_ = status;
            tail_claw_status_valid_ = true;
            updated = true;
        }

        return updated;
    }

    /**
    * @brief 清空之前的命令，避免误触发
    * @note qr_code_sub_ 的长度是1，TryGet一次就能清空之前的命令了
    * @note 不要放入 wait_until，只清理一次就好了
    */
    void clean_previous_cmd() {
        pub_qr_code_parsed temp_qr{};
        qr_code_sub_.TryGet(&temp_qr);
        (void)infrared_group.tryGet();
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
            logger_queue.log("R1 ir cmd: 0x%02X\n", cmd);
        }
        if (qr_code_sub_.TryGet(&qr_code_msg)) {
            cmd = qr_code_msg.data;
            logger_queue.log("R1 qr cmd: 0x%02X\n", cmd);
        }
        return cmd;
    }

    void move_left() {
        if (moved_left_right_ == 0) {
            moved_left_right_ -= 1;
            move_to_pos(waypoint::mf_entrance_left);
            move_to_pos(waypoint::mf_entrance_left_close);
        } else if (moved_left_right_ == 1) {
            moved_left_right_ -= 1;
            move_to_pos(waypoint::mf_entrance_mid);
            move_to_pos(waypoint::mf_entrance_mid_close);
        } else {
            return; // 已经在最左边了，不能再左移
        }
    }

    void move_right() {
        if (moved_left_right_ == 0) {
            moved_left_right_ += 1;
            move_to_pos(waypoint::mf_entrance_right);
            move_to_pos(waypoint::mf_entrance_right_close);
        } else if (moved_left_right_ == -1) {
            moved_left_right_ += 1;
            move_to_pos(waypoint::mf_entrance_mid);
            move_to_pos(waypoint::mf_entrance_mid_close);
        } else {
            return; // 已经在最右边了，不能再右移
        }
    }
};

RAM_D1_ATTR static StateMachine state_machine;
void stateMachineTask(void *argument) {
    for (;;) {
        if constexpr (MATCH_TYPE == match_type::CWTY) {
            state_machine.run_cwty();
        } else if constexpr (MATCH_TYPE == match_type::JGCB) {
            state_machine.run_jgcb(); 
        }
        osDelay(1);
    }
}