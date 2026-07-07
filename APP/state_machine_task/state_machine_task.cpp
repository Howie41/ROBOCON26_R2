/**
 * @file state_machine_task.cpp
 * @author zhy (Howie41)
 * @brief 状态机任务
 * @date 2026-05-24
 */

#include "arm_task.hpp"
#include "cmsis_os2.h"
#include <atomic>
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
class location{
public:
    int16_t x;
    int16_t y;
    int16_t yaw;
};

constexpr location init{-550, 150, 0};

constexpr location mf_entrance_mid{2060, 1500, 0};
constexpr location mf_entrance_left{2060, 1500+1200, 0};
constexpr location mf_entrance_right{2060, 1500-1200, 0};

constexpr location mf_entrance_mid_close{2085, mf_entrance_mid.y, 0};
constexpr location mf_entrance_left_close{2085, mf_entrance_left.y, 0};
constexpr location mf_entrance_right_close{2085, mf_entrance_right.y, 0};

// ======== 三区 ========
constexpr location before_uphill{8000, 3650, 0};
constexpr location after_uphill{10725, 3650, 0};
/** @brief 赛中装填 KFS 点位 */
constexpr location load_kfs{10210,970,0};

constexpr int16_t grid_y = -140;
constexpr location grid_mid{10000, grid_y, -90};
constexpr location grid_left{grid_mid.x + 540, grid_y, -90};
constexpr location grid_right{grid_mid.x - 540, grid_y, -90};

constexpr int16_t grid_close_y = -500;
constexpr location grid_mid_close{grid_mid.x, grid_close_y, -90};
constexpr location grid_left_close{grid_left.x, grid_close_y, -90};
constexpr location grid_right_close{grid_right.x, grid_close_y, -90};

/** @brief 贴左侧围栏、近九宫格点位 */
constexpr location left_fence_front{grid_left.x + 300, grid_y, -90};
/** @brief 贴左侧围栏、后侧点位 */
constexpr location left_fence_back{grid_left.x + 300, 680, -90};
/** @brief R1 R2 合体预备点 */
constexpr location combination_area{grid_right.x, 680, -90};
} // namespace waypoint

volatile bool debug_pause{false};
std::atomic<bool> g_config_valid{false};
std::atomic<int16_t> g_config_origin_x{0};
std::atomic<int16_t> g_config_origin_y{0};
std::atomic<area_type> g_config_area_type{area_type::blue};

class StateMachine {
public:
    class state {
    private:
        state(const state &) = delete;
        state &operator=(const state &) = delete;
    protected:
        state() = default;
        ~state() = default;
    public:
        virtual void on_tick(StateMachine& sm) = 0;
        virtual char* get_name() = 0;
    };

#define STATE(name) \
    class name : public state { \
    public: \
        static name& instance() { static name i; return i; } \
        char* get_name() override { return (char*)#name; } \
        void on_tick(StateMachine& sm) override

#define STATE_END };

/**
 * STATE(state_name) {
 *     // 状态逻辑 ...
 *     // sm 表示状态机实例
 * } STATE_END
 */

    // 上电后就绪等待开始
    STATE(ready) {
        if constexpr (MATCH_TYPE == match_type::CWTY) {
            logger_queue.log("SM ======== CWTY-READY ========\n");
        } else {
            logger_queue.log("SM ======== JGCB-READY ========\n");
        }
        sm.wait_for_startup_config();
        if constexpr (MATCH_TYPE == match_type::CWTY) {
            sm.change_state_to(begin_cwty::instance());
        } else {
            sm.change_state_to(begin_jgcb::instance());
        }
    } STATE_END

    // 停止
    STATE(stop) {
    } STATE_END

    // ===== 崇武探幽 =====

    // 启动（崇武探幽）
    STATE(begin_cwty) {
        logger_queue.log("SM ======== CWTY-BEGIN ========\n");

        tail_claw_set_weapon_claw(true);            // 打开夹爪，夹紧武器头
        sm.move_to_pos(-180, -115, 0, 5000);
        sm.change_state_to(go_to_shr::instance());
    } STATE_END

    // 前往端头架
    STATE(go_to_shr) {
        if (sm.flag_ == 0) {
            sm.move_to_pos(335, -825, 90, 5000);
        } else {
            sm.move_to_pos(300, -170, 90, 4000U);
            sm.move_to_pos(735, -830, 90, 5000);
        }
        tail_claw_set_weapon_claw(true); // 打开夹爪，夹紧武器头
        constexpr float target = -59.5f;
        tail_claw_set_roll_target(target);
        sm.wait_until_timeout_or([&sm, target]() -> bool {
            sm.tail_claw_update_status();
            return sm.tail_claw_status_valid_
                && fabsf(sm.tail_claw_status_cache_.roll_target_deg - target) < 0.5f
                && sm.tail_claw_status_cache_.roll_arrived;
        }, 3000U, 20U);
        sm.change_state_to(catch_weapon::instance());
    } STATE_END

    // 夹爪夹取武器
    STATE(catch_weapon) {
        if (sm.flag_ == 0) {
            sm.move_to_pos(335, -895, 90, 5000);
            // sm.move_to_pos(735, -905, 90, 5000);
        } else {
            sm.move_to_pos(735, -905, 90, 8000);
        }
        tail_claw_set_weapon_claw(false);         // 闭合夹爪，夹紧武器头
        osDelay(500);                             // 等待夹爪动作完成，具体时间待调试
        sm.flag_++;
        sm.change_state_to(rotate_weapon_claw::instance());
    } STATE_END

    // 夹爪反转
    STATE(rotate_weapon_claw) {
        tail_claw_set_roll_target(2.0f);
        osDelay(1000);
        sm.wait_until_timeout_or([&sm]() -> bool {
            sm.tail_claw_update_status();
            return sm.tail_claw_status_valid_ && sm.tail_claw_status_cache_.roll_arrived;
        }, 3000U, 10U);
        tail_claw_reset_match();
        sm.change_state_to(match_rod::instance());
    } STATE_END

    // 端头架对齐武器杆
    STATE(match_rod) {
        sm.move_to_pos(300, -170, -90, 4000U);
        sm.change_state_to(wait_for_decision_cmd::instance());
    } STATE_END

    // 等待操作手决策，决定是否拼装新的武器
    STATE(wait_for_decision_cmd) {
        sm.clean_previous_cmd();
        sm.wait_until([&sm]() -> bool {
            switch (sm.get_cmd_from_r1()) {
                case 0x0A: // 松开夹爪
                    tail_claw_set_weapon_claw(true);
                    return false;
                case 0x1A: // 夹取新的武器头
                    sm.change_state_to(go_to_shr::instance());
                    return true;
                case 0x1B: // 进梅林
                    sm.change_state_to(go_to_mf_entrance::instance());
                    return true;
                default:
                    return false;
            }
        }, 25);
    } STATE_END

    // 前往梅林入口
    STATE(go_to_mf_entrance) {
        sm.move_to_pos(waypoint::mf_entrance_mid);
        tail_claw_set_roll_target(-118.0f);
        sm.change_state_to(request_for_path_cmd::instance());
    } STATE_END

    // 请求路径规划命令
    STATE(request_for_path_cmd) {
        sm.path_cmd_request_pub_.Publish(sm.current_path_cmd_index_); // 发一次 request
        logger_queue.log("SM path cmd requested No.%d...\n", sm.current_path_cmd_index_);

        path_cmd::code cmd;
        auto if_received = sm.wait_until_timeout_or([&]() -> bool {
            return sm.path_cmd_sub_.TryGet(&cmd);
        }, 3000); // 3秒超时

        if (!if_received) {
            logger_queue.log("SM path cmd timeout!\n");
            return; // 超时未收到指令，保持当前状态重试
        }

        logger_queue.log("SM path cmd received No.%d 0x%02X\n", sm.current_path_cmd_index_, static_cast<uint8_t>(cmd));
        sm.current_path_cmd_index_ += 1;
        sm.current_path_cmd_ = cmd; // 给其他后续状态读取

        switch (cmd) {
            case path_cmd::code::move_forward:
            case path_cmd::code::move_backward:
            case path_cmd::code::turn_left_90:
            case path_cmd::code::turn_right_90:
            case path_cmd::code::move_left:
            case path_cmd::code::move_right:
            case path_cmd::code::turn_around:
                sm.change_state_to(execute_chassis_action::instance());
                break;
            case path_cmd::code::grab_low_r2kfs:
            case path_cmd::code::grab_mid_r2kfs:
            case path_cmd::code::grab_high_r2kfs:
            case path_cmd::code::drop_and_grab_new_kfs:
                sm.change_state_to(execute_arm_action::instance());
                break;
            case path_cmd::code::no_more_commands:
                logger_queue.log("SM path cmd end\n");
                sm.change_state_to(go_to_mf_exit::instance());
                break;
            default:
                break;
        }
    } STATE_END

    // 执行底盘动作
    STATE(execute_chassis_action) {
        path_cmd::code executing_cmd = sm.current_path_cmd_;
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
                sm.move_left();
                break;
            case path_cmd::code::move_right:
                sm.move_right();
                break;
            default:
                break;
        }
        sm.current_path_cmd_ = path_cmd::code::unknown; // 清空当前命令
        sm.change_state_to(request_for_path_cmd::instance());
    } STATE_END

    // 执行取矿机构动作
    STATE(execute_arm_action) {
        path_cmd::code executing_cmd = sm.current_path_cmd_;

        chassis_action::start_go_to_edge();
        
        switch (executing_cmd) {
            case path_cmd::code::grab_low_r2kfs:
                arm_action::raise_kfs(LOAD_TYPE::LOW);
                break;
            case path_cmd::code::grab_mid_r2kfs:
                arm_action::raise_kfs(LOAD_TYPE::MEDIUM);
                break;
            case path_cmd::code::grab_high_r2kfs:
                arm_action::raise_kfs(LOAD_TYPE::HIGH);
                break;
            case path_cmd::code::drop_and_grab_new_kfs:
                // 忽略这个指令
                break;
            default:
                break;
        }

        sm.wait_until_timeout_or([&]() -> bool {
            return arm.get_attr().is_kfs_raised;
        }, 3000);

        arm_action::load_kfs();

        chassis_action::start_return_to_center();

        sm.current_path_cmd_ = path_cmd::code::unknown; // 清空当前命令
        sm.change_state_to(request_for_path_cmd::instance());
    } STATE_END

    // 前往梅林出口
    STATE(go_to_mf_exit) {
        sm.move_to_pos(waypoint::before_uphill, 5000);
        sm.change_state_to(stop::instance());
    } STATE_END

    // ===== 九宫藏宝 =====

    // 启动（九宫藏宝）
    STATE(begin_jgcb) {
        logger_queue.log("SM ======== JGCB-BEGIN ========\n");
        sm.clean_previous_cmd();
        sm.wait_until([&sm]() -> bool {
            return (sm.get_cmd_from_r1() == 0x2A);
        });
        sm.change_state_to(go_to_arena::instance());
    } STATE_END

    // 上坡、前往竞技场
    STATE(go_to_arena) {
        sm.move_to_pos(waypoint::after_uphill);
        sm.change_state_to(go_to_load_kfs::instance());
    } STATE_END

    // 前往距斜坡最近的KFS前
    STATE(go_to_load_kfs) {
        sm.move_to_pos(waypoint::after_uphill.x, waypoint::after_uphill.y - 1500, waypoint::after_uphill.yaw);
        // 防止卡到斜坡上
        sm.move_to_pos(waypoint::load_kfs);
        sm.change_state_to(load_kfs::instance());
    } STATE_END

    // 装载KFS
    STATE(load_kfs) {
        arm_action::raise_kfs(LOAD_TYPE::PLAIN);
        sm.wait_until_timeout_or([&]() -> bool {
            return arm.get_attr().is_kfs_raised;
        }, 2500);
        arm_action::unload_kfs(UNLOAD_TYPE::TOP);

        sm.change_state_to(wait_and_place_kfs::instance());
    } STATE_END

    // 等待指令，然后放置中层KFS
    STATE(wait_and_place_kfs) {
        sm.move_to_pos(waypoint::grid_left);
        sm.clean_previous_cmd();
        sm.wait_until([&sm]() -> bool {
            switch (sm.get_cmd_from_r1()) {
                case 0x3A:
                    sm.move_to_pos(waypoint::grid_left_close);
                    return true;
                case 0x3B:
                    sm.move_to_pos(waypoint::grid_mid);
                    sm.move_to_pos(waypoint::grid_mid_close);
                    return true;
                case 0x3C:
                    sm.move_to_pos(waypoint::grid_right);
                    sm.move_to_pos(waypoint::grid_right_close);
                    return true;
                default:
                    return false;
            }
        });
        arm_action::release_kfs();
        sm.wait_until_timeout_or([&]() -> bool {
            return !arm.get_attr().is_place_releasing;
        }, 1000);
        sm.change_state_to(go_to_combination_area::instance());
    } STATE_END

    // 前往合体点位
    STATE(go_to_combination_area) {
        // 慢慢退后到 R1 后面，准备合体
        sm.move_to_pos(waypoint::grid_left);
        sm.move_to_pos(waypoint::left_fence_front);
        sm.move_to_pos(waypoint::left_fence_back);
        sm.move_to_pos(waypoint::combination_area);
        sm.change_state_to(wait_for_combination_cmd::instance());
    } STATE_END

    // 等待合体指令
    STATE(wait_for_combination_cmd) {
        sm.clean_previous_cmd();
        sm.wait_until([&sm]() -> bool {
            return (sm.get_cmd_from_r1() == 0x4A);
        });
        sm.change_state_to(begin_combination::instance());
    } STATE_END

    // 合体
    STATE(begin_combination) {
        chassis_action::start_climb_R1();
        sm.change_state_to(unload_kfs::instance());
    } STATE_END

    // 取出KFS并手持
    STATE(unload_kfs) {
        arm_action::unload_kfs(UNLOAD_TYPE::LOW);
        sm.wait_until_timeout_or([&]() -> bool {
            return !arm.get_attr().is_placing_kfs_L;
        }, 8000);
        sm.change_state_to(wait_for_place_hi_kfs_cmd::instance());
    } STATE_END

    // 等待放置高层KFS的指令
    STATE(wait_for_place_hi_kfs_cmd) {
        sm.clean_previous_cmd();
        sm.wait_until([&sm]() -> bool {
            switch (sm.get_cmd_from_r1()) {
                case 0x5B: // 释放 KFS
                    sm.change_state_to(release_kfs::instance());
                    return true;
                case 0x5A: // 重试取出KFS
                    sm.change_state_to(unload_kfs::instance());
                    return true;
                default:
                    return false;
            }
        });
    } STATE_END

    // 释放KFS
    STATE(release_kfs) {
        arm_action::release_kfs();
        sm.wait_until_timeout_or([&]() -> bool {
            return !arm.get_attr().is_place_releasing;
        }, 1000);
        sm.change_state_to(stop::instance());
    } STATE_END

#undef STATE
#undef STATE_END

    StateMachine() = default;
    ~StateMachine() = default;

    void run() {
        current_state_->on_tick(*this);
    }

private:
    int8_t moved_left_right_{0}; // 左右移动计数
    int flag_{0};                 // 武器夹取计数，0=第一次夹取，非0=后续夹取

    startup_config current_startup_config_{};
    std::optional<waypoint::location> current_origin_location_{};
    state* current_state_{&ready::instance()};
    uint16_t current_path_cmd_index_{0};
    path_cmd::code current_path_cmd_{path_cmd::code::unknown}; // 初始值对下位机来说没有意义

    TypedTopicSubscriber<pub_qr_code_parsed> qr_code_sub_{"qr_code_parsed", 1};
    // 尾爪武器事件发布（通知上位机发送距离数据）
    TypedTopicPublisher<tail_claw_msg> tail_claw_weapon_event_pub_{"tail_claw_weapon_event"};
    // 尾爪状态订阅
    TypedTopicSubscriber<pub_tail_claw_status> tail_claw_status_sub_{"tail_claw_status", 4};

    TypedTopicSubscriber<path_cmd::code> path_cmd_sub_{"pc_path_cmd", 1}; // 接收路径规划cmd
    TypedTopicPublisher<uint16_t> path_cmd_request_pub_{"pc_path_cmd_request"}; // 请求路径规划cmd

    TypedTopicSubscriber<startup_config> startup_config_sub_{"pc_startup_config", 1}; // 启动配置接收（也是开始信号）
    TypedTopicPublisher<bool> startup_config_ack_pub_{"pc_startup_config_ack"}; // 启动配置接收回应

    pub_tail_claw_status tail_claw_status_cache_{};
    bool tail_claw_status_valid_{};
    bool state_machine_view_last_{};

    /**
     * @brief 等待上位机发送启动配置
     */
    void wait_for_startup_config() {
        logger_queue.log("SM startup config waiting...\n");
        startup_config config{};
        wait_until([&]() -> bool {
            return startup_config_sub_.TryGet(&config);
        });
        logger_queue.log("SM startup config received:\n");
        logger_queue.log("  area=%d begin=%d O(%d,%d)\n",
            static_cast<int>(config.area_type_value),
            static_cast<int>(config.begin_type_value),
            config.origin_x,
            config.origin_y
        );
        current_startup_config_ = config;
        current_origin_location_.emplace(waypoint::location{config.origin_x, config.origin_y, 0});
        g_config_origin_x.store(config.origin_x);
        g_config_origin_y.store(config.origin_y);
        g_config_area_type.store(config.area_type_value);
        g_config_valid.store(true);
        startup_config_ack_pub_.Publish(true); // 发送应答给上位机
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

    void change_state_to(state& new_state) {
        do_debug_pause("change_state");
        logger_queue.log("SM -> %s\n", new_state.get_name());
        current_state_ = &new_state;
    }

    // 这个函数必须在任务环境里调用
    bool move_to_pos(int16_t x, int16_t y, int16_t yaw, uint32_t timeout_ms = 0) {
        if (current_origin_location_.has_value()) {
            x += current_origin_location_->x;
            y += current_origin_location_->y;
        }

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

    bool move_to_pos(const waypoint::location &loc, uint32_t timeout_ms = 0) {
        return move_to_pos(loc.x, loc.y, loc.yaw, timeout_ms);
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

    /**
     * @brief 调试用的暂停函数，只有在 ENABLE_DEBUG_PAUSE 为 true 时才会生效
     * @param msg 日志信息，用于标识暂停的原因
     */
    void do_debug_pause(const char *msg) {
        if constexpr (ENABLE_DEBUG_PAUSE) {
            debug_pause = true;
            logger_queue.log("DEBUG pause: %s\n", msg);
            wait_until([]() -> bool { return !debug_pause; });
            logger_queue.log("DEBUG resume: %s\n", msg);
        }
    }
};

RAM_D1_ATTR static StateMachine state_machine;
void stateMachineTask(void *argument) {
    for (;;) {
        state_machine.run();
        osDelay(1);
    }
}
