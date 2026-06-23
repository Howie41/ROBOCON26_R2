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
#include "tail_claw_task.hpp"
osThreadId_t StateMachineTaskHandle;

static std::atomic<RobotState> current_state{RobotState::begin};

TypedTopicSubscriber<pub_qr_code_parsed> qr_code_sub("qr_code_parsed", 1);
//下位机发信息给上位机，告诉它当前的状态，或者说事件发生了，或者说需要它做什么
static TypedTopicPublisher<tail_claw_msg>tail_claw_weapon_event_pub("tail_claw_weapon_event");
//尾爪状态
static TypedTopicSubscriber<pub_tail_claw_status>
    tail_claw_status_sub("tail_claw_status", 4);

static pub_tail_claw_status tail_claw_status_cache{};
static bool tail_claw_status_valid = false;
//更新尾爪状态
static bool tail_claw_update_status()
{
    pub_tail_claw_status status{};
    bool updated = false;

    while (tail_claw_status_sub.TryGet(&status)) {
        tail_claw_status_cache = status;
        tail_claw_status_valid = true;
        updated = true;
    }

    return updated;
}

static bool state_machine_view_last = false;
static bool consume_state_machine_view(bool current_state) {
    const bool rising_edge = current_state && !state_machine_view_last;
    state_machine_view_last = current_state;
    return rising_edge;
}

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
bool  move_to_pos(int16_t x, int16_t y, int16_t yaw,uint32_t timeout_ms=8000) {
    taskENTER_CRITICAL();
    nav_control::target_x = x;
    nav_control::target_y = y;
    nav_control::target_yaw = yaw;

    nav_control::auto_enabled = true;
    nav_control::arrived = false;
    nav_control::target_active = true;
    nav_control::arrival_reported = false;
    taskEXIT_CRITICAL();

    nav_control::resetAllPIDs();
    const bool arrived = wait_until_timeout_or([]() -> bool {
        return nav_control::arrived;
    }, timeout_ms, 20U);

    return arrived;
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

int start=0;
void stateMachineTask(void *argument) {
     TypedTopicSubscriber<pub_Xbox_Data> control_xbox_sub("xbox", 8);
    pub_Xbox_Data control_xbox_cmd{};
    for (;;) {
        switch (current_state.load()) {
        #ifdef MATCH_CWTY /** ========== 崇武探幽 单项赛 ========== */

            case RobotState::begin: {
                if(start==2)
                {
                    clean_previous_cmd();
                    wait_until([&]() -> bool {
                    switch (get_cmd_from_r1()) {
                        /*case 0x1A: // 夹取新的武器头
                            change_state_to(RobotState::go_to_SHR);
                            return true;
                        case 0x1B: // 进入梅林
                            change_state_to(RobotState::go_to_MF);
                            return true;*/
                            case 0x0A:
                                tail_claw_set_weapon_claw(true);
                                tail_claw_set_roll_target(-56.5);
                                return true;
                        default:
                            return false;
                    }
                });
                }
                if(start==1) 
                {

                    const bool view_rising_edge =
                    consume_state_machine_view(control_xbox_cmd.btnView);
                    control_xbox_cmd.btnView = view_rising_edge;
                    if (view_rising_edge) {
                        tail_claw_set_mode(TailClawMode::Hold);//锁定当前的位置
                    }
                    if (start==1) {//按下View键进入下一状态
                        //开始进行自动
                        tail_claw_set_roll_target(-56.5);
                        //tail_claw_setAirPump(true);      //气泵的开关
                        tail_claw_set_weapon_claw(true);     //闭合夹爪，夹紧武器头，
                        move_to_pos(-500, 15, 0,5000);
                        tail_claw_set_roll_target(-59.5);
                        change_state_to(RobotState::go_to_SHR);
                    }
                }
                break;
            }

            case RobotState::go_to_SHR: {
                //发个信号，唤醒通知tail_claw_task去对准武器头
                //move_to_pos(-286, -840, 90,5000);
                move_to_pos(-266, -860, 90,5000);
                //tail_claw_setAirPump(false);
                tail_claw_set_mode(TailClawMode::AutoAlign);//进入自动对齐模式
                //发消息给上位机，他要发消息给我了
                tail_claw_msg msg{0}; 
                msg.distance = 1;
                tail_claw_weapon_event_pub.Publish(msg);
                // tail_claw_task会根据距离数据调整位置
                change_state_to(RobotState::aim_at_weapon);
                break;
            }

            case RobotState::aim_at_weapon: {
                wait_until([&]() -> bool {
                    // TODO: 判断夹爪是否对准武器头
                    tail_claw_update_status();
                    return tail_claw_status_valid && tail_claw_status_cache.weapon_matched;     //对准
                });
                //关闭武器对准，告诉上位机不用发了
                tail_claw_msg msg{};
                msg.distance = 2;
                tail_claw_weapon_event_pub.Publish(msg);

                tail_claw_set_mode(TailClawMode::Hold);//对齐后锁定位置
                change_state_to(RobotState::catch_weapon);
                break;
            }

            case RobotState::catch_weapon: {
                 /* if (!move_to_pos(-290, -944, 90, 10000U)) {
                        change_state_to(RobotState::stop);
                        break;
                }*/// 对准后向前移动一段距离，具体数值待调试
                move_to_pos(-266, -960, 90, 10000U);
                tail_claw_set_weapon_claw(false);//闭合夹爪，夹紧武器头
                osDelay(100); // 等待夹爪动作完成，具体时间待调试
                change_state_to(RobotState::rotate_weapon_claw);
                break;
            }

            case RobotState::rotate_weapon_claw: {  
                    tail_claw_set_roll_target(2.0f);

                    /*wait_until([]() -> bool {
                        constexpr float roll_reduction_ratio = 2.5f;
                        constexpr float target_roll_pos = 83.0f * roll_reduction_ratio;
                        constexpr float pos_tolerance = 5.0f;
                        constexpr float speed_tolerance = 5.0f;

                        return fabsf(tail_claw_roll_motor.getCurrentSumPos() - target_roll_pos) < pos_tolerance &&
                            fabsf(tail_claw_roll_motor.getCurrentSpeed()) < speed_tolerance;
                    });*/
                     /*wait_until_timeout_or([]() -> bool {
                    constexpr float roll_reduction_ratio = 2.5f;
                    constexpr float target_roll_pos = 2.0f * roll_reduction_ratio;
                     constexpr float pos_tolerance = 4.0f;
                    constexpr float speed_tolerance = 5.0f;

                    return fabsf(tail_claw_roll_motor.getCurrentSumPos() - target_roll_pos) < pos_tolerance &&
                    fabsf(tail_claw_roll_motor.getCurrentSpeed()) < speed_tolerance;
                    }, 3000U, 10U);   // 最多等 3000ms，每 10ms 检查一次*/
                    wait_until_timeout_or([]() -> bool {
                        tail_claw_update_status();
                        return tail_claw_status_valid && tail_claw_status_cache.roll_arrived;
                    }, 3000U, 10U);
                    //change_state_to(RobotState::stop);

                    change_state_to(RobotState::match_rod);
                    break;
            }

            case RobotState::match_rod: {
                 /* if (!move_to_pos(20, -100, -90, 4000U)) {
                         change_state_to(RobotState::stop);
                            break;
                     } */// TODO: 填入武器rod位置
                move_to_pos(20, -100, -90, 4000U);
                tail_claw_set_mode(TailClawMode::AutoAlign);//进入自动对齐模式

                /*static TypedTopicPublisher<tail_claw_msg>
                tail_claw_rod_event_pub("tail_claw_rod_event");
                tail_claw_msg msg{};
                msg.distance = 1;
                tail_claw_rod_event_pub.Publish(msg);*/
                tail_claw_msg msg{};
                msg.distance = 3;
                tail_claw_weapon_event_pub.Publish(msg);
                tail_claw_reset_match();
                /*wait_until([&]() -> bool {
                    // TODO: 判断夹爪是否对准武器杆
                    return (weapon_match_state_ & ismatch)!= 0;      //对准
                });*/
                wait_until([&]() -> bool {
                    // TODO: 判断夹爪是否对准武器杆
                    tail_claw_update_status();
                    return tail_claw_status_valid && tail_claw_status_cache.weapon_matched;
                });

                //关闭武器对准，告诉上位机不用发了
                /*msg.distance = 2;
                tail_claw_rod_event_pub.Publish(msg);
                tail_claw_set_mode(TailClawMode::Hold);*/
                msg.distance = 4;
                tail_claw_weapon_event_pub.Publish(msg);

                change_state_to(RobotState::wait_for_cmd);
                break;
            }
            // R2松开武器头夹爪，等待操作手决策，决定是否拼装新的武器
            case RobotState::wait_for_cmd: {
                clean_previous_cmd();
                wait_until([&]() -> bool {
                    switch (get_cmd_from_r1()) {
                        /*case 0x1A: // 夹取新的武器头
                            change_state_to(RobotState::go_to_SHR);
                            return true;
                        case 0x1B: // 进入梅林
                            change_state_to(RobotState::go_to_MF);
                            return true;*/
                            case 0x0A:
                                tail_claw_set_weapon_claw(true);
                                return true;
                        default:
                            return false;
                    }
                });
                break;
            }

            case RobotState::stop:{
                nav_control::auto_enabled = false;
                nav_control::target_active = false;
                nav_control::arrived = false;
                nav_control::arrival_reported = false;

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

void  state_machine()
{
    for (;;) {
        switch (current_state.load()) {
            case RobotState::begin:{
                //start==2是测试
                if(start==2)
                {
                    clean_previous_cmd();
                    wait_until([&]() -> bool {
                    switch (get_cmd_from_r1()) {
                        /*case 0x1A: // 夹取新的武器头
                            change_state_to(RobotState::go_to_SHR);
                            return true;
                        case 0x1B: // 进入梅林
                            change_state_to(RobotState::go_to_MF);
                            return true;*/
                            case 0x0A:
                                tail_claw_set_weapon_claw(true);
                                tail_claw_set_roll_target(-56.5);
                                return true;
                        default:
                            return false;
                    }
                });
                }
                if(start==1) 
                {
                    //按下View键进入下一状态
                    //开始进行自动
                    tail_claw_set_roll_target(-56.5);
                    //tail_claw_setAirPump(true);      //气泵的开关
                    tail_claw_set_weapon_claw(true);     //闭合夹爪，夹紧武器头，
                    move_to_pos(-500, 15, 0,5000);
                    tail_claw_set_roll_target(-59.5);
                    change_state_to(RobotState::go_to_SHR);
                    
                }
                break;
            }

            case RobotState::go_to_SHR: {
                //发个信号，唤醒通知tail_claw_task去对准武器头
                //move_to_pos(-286, -840, 90,5000);
                move_to_pos(-266, -860, 90,5000);
                //tail_claw_setAirPump(false);
                tail_claw_set_mode(TailClawMode::AutoAlign);//进入自动对齐模式
                //发消息给上位机，他要发消息给我了
                tail_claw_msg msg{0}; 
                msg.distance = 1;
                tail_claw_weapon_event_pub.Publish(msg);
                // tail_claw_task会根据距离数据调整位置
                change_state_to(RobotState::aim_at_weapon);
                break;
            }

            case RobotState::aim_at_weapon: {
                wait_until([&]() -> bool {
                    // TODO: 判断夹爪是否对准武器头
                    tail_claw_update_status();
                    return tail_claw_status_valid && tail_claw_status_cache.weapon_matched;     //对准
                });
                //关闭武器对准，告诉上位机不用发了
                tail_claw_msg msg{};
                msg.distance = 2;
                tail_claw_weapon_event_pub.Publish(msg);

                tail_claw_set_mode(TailClawMode::Hold);//对齐后锁定位置
                change_state_to(RobotState::catch_weapon);
                break;
            }

            case RobotState::catch_weapon: {
                 /* if (!move_to_pos(-290, -944, 90, 10000U)) {
                        change_state_to(RobotState::stop);
                        break;
                }*/// 对准后向前移动一段距离，具体数值待调试
                move_to_pos(-266, -960, 90, 10000U);
                tail_claw_set_weapon_claw(false);//闭合夹爪，夹紧武器头
                osDelay(100); // 等待夹爪动作完成，具体时间待调试
                change_state_to(RobotState::rotate_weapon_claw);
                break;
            }

            case RobotState::rotate_weapon_claw: {  
                    tail_claw_set_roll_target(2.0f);

                    /*wait_until([]() -> bool {
                        constexpr float roll_reduction_ratio = 2.5f;
                        constexpr float target_roll_pos = 83.0f * roll_reduction_ratio;
                        constexpr float pos_tolerance = 5.0f;
                        constexpr float speed_tolerance = 5.0f;

                        return fabsf(tail_claw_roll_motor.getCurrentSumPos() - target_roll_pos) < pos_tolerance &&
                            fabsf(tail_claw_roll_motor.getCurrentSpeed()) < speed_tolerance;
                    });*/
                     /*wait_until_timeout_or([]() -> bool {
                    constexpr float roll_reduction_ratio = 2.5f;
                    constexpr float target_roll_pos = 2.0f * roll_reduction_ratio;
                     constexpr float pos_tolerance = 4.0f;
                    constexpr float speed_tolerance = 5.0f;

                    return fabsf(tail_claw_roll_motor.getCurrentSumPos() - target_roll_pos) < pos_tolerance &&
                    fabsf(tail_claw_roll_motor.getCurrentSpeed()) < speed_tolerance;
                    }, 3000U, 10U);   // 最多等 3000ms，每 10ms 检查一次*/
                    wait_until_timeout_or([]() -> bool {
                        tail_claw_update_status();
                        return tail_claw_status_valid && tail_claw_status_cache.roll_arrived;
                    }, 3000U, 10U);
                    //change_state_to(RobotState::stop);

                    change_state_to(RobotState::match_rod);
                    break;
            }

            case RobotState::match_rod: {
                 /* if (!move_to_pos(20, -100, -90, 4000U)) {
                         change_state_to(RobotState::stop);
                            break;
                     } */// TODO: 填入武器rod位置
                move_to_pos(20, -100, -90, 4000U);
                tail_claw_set_mode(TailClawMode::AutoAlign);//进入自动对齐模式

                /*static TypedTopicPublisher<tail_claw_msg>
                tail_claw_rod_event_pub("tail_claw_rod_event");
                tail_claw_msg msg{};
                msg.distance = 1;
                tail_claw_rod_event_pub.Publish(msg);*/
                tail_claw_msg msg{};
                msg.distance = 3;
                tail_claw_weapon_event_pub.Publish(msg);
                tail_claw_reset_match();
                /*wait_until([&]() -> bool {
                    // TODO: 判断夹爪是否对准武器杆
                    return (weapon_match_state_ & ismatch)!= 0;      //对准
                });*/
                wait_until([&]() -> bool {
                    // TODO: 判断夹爪是否对准武器杆
                    tail_claw_update_status();
                    return tail_claw_status_valid && tail_claw_status_cache.weapon_matched;
                });

                //关闭武器对准，告诉上位机不用发了
                /*msg.distance = 2;
                tail_claw_rod_event_pub.Publish(msg);
                tail_claw_set_mode(TailClawMode::Hold);*/
                msg.distance = 4;
                tail_claw_weapon_event_pub.Publish(msg);

                change_state_to(RobotState::wait_for_cmd);
                break;
            }
            // R2松开武器头夹爪，等待操作手决策，决定是否拼装新的武器
            case RobotState::wait_for_cmd: {
                clean_previous_cmd();
                wait_until([&]() -> bool {
                    switch (get_cmd_from_r1()) {
                        /*case 0x1A: // 夹取新的武器头
                            change_state_to(RobotState::go_to_SHR);
                            return true;
                        case 0x1B: // 进入梅林
                            change_state_to(RobotState::go_to_MF);
                            return true;*/
                            case 0x0A:
                                tail_claw_set_weapon_claw(true);
                                return true;
                        default:
                            return false;
                    }
                });
                break;
            }

            case RobotState::stop:{
                nav_control::auto_enabled = false;
                nav_control::target_active = false;
                nav_control::arrived = false;
                nav_control::arrival_reported = false;

                break;
            }
        }
    }    
}

