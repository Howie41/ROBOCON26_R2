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
#include <optional>

#include "state_machine_task.h"
#include "com_config.h"
#include "NavProtocol.hpp"
#include "infrared_com.hpp"
#include "memory_map.h"
#include "merlin_map.h"
#include "tail_claw_controller.hpp"
#include "tail_claw_task.hpp"
#include "topic_pool.h"
#include "topics.hpp"
#include "chassis_task.h"
#include "logger.hpp"

osThreadId_t StateMachineTaskHandle;
extern Arm arm;  // 取矿机构实例
extern LoggerQueue logger_queue;

constexpr size_t SH_COUNT = 6;
volatile bool debug_pause{false};
std::atomic<bool> g_config_valid{false};
std::atomic<int16_t> g_config_origin_x{0};
std::atomic<int16_t> g_config_origin_y{0};
std::atomic<area_type> g_config_area_type{area_type::blue};

namespace waypoint {
class location{
public:
    int16_t x;
    int16_t y;
    int16_t yaw;
    const char* name = nullptr;
    bool enable_red_area_mirror = true;
};

constexpr location before_shr{500, 0, 0, "before_shr"};

constexpr int16_t sh_aim_y = -692;
//起点：-95，210
//起点2：-83，212
//-88,195
constexpr std::array<location, SH_COUNT> sh_aim{
    /*
    location{-275+50, sh_aim_y-45, 90, "sh_aim"},
    location{-75+50, sh_aim_y-45, 90, "sh_aim"},
    location{125+50, sh_aim_y-45, 90, "sh_aim"},
    location{725+50, sh_aim_y-45, 90, "sh_aim"},
    location{525+50, sh_aim_y-45, 90, "sh_aim"},
    location{325+50, sh_aim_y-45, 90, "sh_aim"},
    */
    //662，-465
    //-520

    //670，-500
    //-540

    //659,-585
    //-640
    location{753+15-1000, -780, 90, "sh_aim"},
    location{753+15-800, -780, 90, "sh_aim"},
    location{753+15-600, -780, 90, "sh_aim"},
    location{753+15, -780, 90, "sh_aim"},
    location{753+15-200, -780, 90, "sh_aim"},
    location{753+15-400, -780, 90, "sh_aim"},
};
constexpr int16_t sh_close_y = -772;

constexpr std::array<location, SH_COUNT> sh_close{
    /*
    location{sh_aim[0].x, sh_close_y-18, 90, "sh_close"},
    location{sh_aim[1].x, sh_close_y-18, 90, "sh_close"},
    location{sh_aim[2].x, sh_close_y-18, 90, "sh_close"},
    location{sh_aim[3].x, sh_close_y-18, 90, "sh_close"},
    location{sh_aim[4].x, sh_close_y-18, 90, "sh_close"},
    location{sh_aim[5].x, sh_close_y-18, 90, "sh_close"},
    */
    location{sh_aim[0].x, -835+10, 90, "sh_close"},
    location{sh_aim[1].x, -835+10, 90, "sh_close"},
    location{sh_aim[2].x, -835+10, 90, "sh_close"},
    location{sh_aim[3].x, -835+10, 90, "sh_close"},
    location{sh_aim[4].x, -835+10, 90, "sh_close"},
    location{sh_aim[5].x, -835+10, 90, "sh_close"},
};

constexpr location match_rod_blue{-95, -822, -90, "match_rod_blue"};

// 这个相对坐标已经基于红区坐标，不需要镜像
constexpr location match_rod_red{match_rod_blue.x, -1000, -90, "match_rod_red",
                                  false};

inline location mf_col(uint8_t num) {
    static const char* names[] = {"mf_col1", "mf_col2", "mf_col3"};
    merlin_map::MerlinPose pose;
    if (g_config_area_type.load() == area_type::blue) {
        pose = merlin_map::g_blue_layout.entry_pose[num - 1];
    } else {
        pose = merlin_map::g_red_layout.entry_pose[num - 1];
    }
    return location{pose.x, pose.y, pose.yaw, names[num - 1], false};
}

// ======== 三区 ========
constexpr location before_uphill{1000, 200, 0, "before_uphill"};
constexpr location after_uphill{3700, 200, 0, "after_uphill"};
constexpr location beside_after_uphill{3500, -1480, -90, "beside_after_uphill"};

constexpr location retry_zone_red{4060, -380, 0, "retry_zone_red"};
constexpr location retry_zone_blue{4060, -retry_zone_red.y, 0, "retry_zone_blue"};
/** @brief 赛中装填 KFS 点位 */




constexpr location load_kfs{3400,-2025-100,0, "load_kfs"};
constexpr location load_kfs_2{3400,load_kfs.y - 700,0, "load_kfs_2"};

constexpr int16_t grid_close_y = -4165 - 20;
constexpr int16_t grid_y = grid_close_y + 350;

constexpr location grid_mid{3050+50, grid_y, -90, "grid_mid"};
constexpr location grid_left{grid_mid.x + 540, grid_y, -90, "grid_left"};
constexpr location grid_right{grid_mid.x - 540, grid_y, -90, "grid_right"};

constexpr location grid_mid_close{grid_mid.x, grid_close_y, -90, "grid_mid_close"};
constexpr location grid_left_close{grid_left.x, grid_close_y, -90, "grid_left_close"};
constexpr location grid_right_close{grid_right.x, grid_close_y, -90, "grid_right_close"};

/** @brief 贴左侧围栏、近九宫格点位 */
constexpr location left_fence_front{grid_left.x + 100, grid_y, -90, "left_fence_front"};
/** @brief 贴左侧围栏、后侧点位 */
constexpr location left_fence_back{left_fence_front.x, left_fence_front.y + 1500, -90, "left_fence_back"};
/** @brief R1 R2 合体预备点 */
constexpr location combination_area{grid_mid.x, left_fence_back.y, -90, "combination_area"};
} // namespace waypoint

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
        virtual const char* get_name() = 0;
    };

#define STATE(name) \
    class name : public state { \
    public: \
        static name& instance() { static name i; return i; } \
        const char* get_name() override { return #name; } \
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
        logger_queue.log("\n");
        logger_queue.log("SM\t======== READY ========\n");

        // 清理之前可能收到过的配置
        startup_config dummy_config;
        sm.startup_config_sub_.TryGet(&dummy_config);

        sm.wait_for_startup_config();

        arm.set_kfs_amount(sm.current_startup_config_.kfs_amount);

        switch (sm.current_startup_config_.begin_type_value) {
            case begin_type::mc:
                sm.change_state_to(begin_cwty::instance());
                break;
            case begin_type::mf:
                sm.change_state_to(go_to_mf_entrance::instance());
                break;
            case begin_type::arena_before_uphill:
                sm.change_state_to(begin_jgcb::instance());
                break;
            case begin_type::arena_retry_zone:
                sm.change_state_to(retry_after_uphill::instance());
                break;
            default:
                break;
        }
    } STATE_END
    
    STATE(ir_debug) {
        logger_queue.log("IR\tdebug_on\n");
        sm.clean_previous_cmd();
        sm.wait_until([&]() -> bool {
            auto cmd = sm.get_cmd_from_r1();
            if (cmd != 0x00) {
                logger_queue.log("IR\treceived\n");
                return true;
            }
            return false;
        });
    } STATE_END

    // 停止
    STATE(stop) {
    } STATE_END

    // ===== 崇武探幽 =====

    // 启动（崇武探幽）
    STATE(begin_cwty) {
        logger_queue.log("SM\tBEGIN CWTY ========\n");

        if (g_config_area_type.load() == area_type::blue) {
            sm.sh_index_ = 3;
        }

        sm.move_to_pos(waypoint::before_shr, 5000);
        sm.move_to_pos(500,0,90,5000);
        osDelay(500);
        //sm.do_debug_pause("before_shr_stop");
        sm.change_state_to(go_to_shr::instance());
    } STATE_END

    // 前往端头架
    STATE(go_to_shr) {
        sm.move_to_pos(waypoint::sh_aim[sm.sh_index_]);
        osDelay(500);
        //sm.do_debug_pause("sh_aim_stop");
        TailClawController::Instance().weapon_claw_open_ = true;
        constexpr float target = 37.0f;
        TailClawController::Instance().roll_target_deg_ = target;
        sm.wait_until_timeout_or([&sm, target]() -> bool {
            sm.tail_claw_update_status();
            return sm.tail_claw_status_valid_
                && fabsf(sm.tail_claw_status_cache_.roll_target_deg - target) < 0.5f
                && sm.tail_claw_status_cache_.roll_arrived;
        },3000, 20U);
        osDelay(1500);
        sm.change_state_to(catch_weapon::instance());
    } STATE_END

    // 夹爪夹取武器
    STATE(catch_weapon) {
        sm.move_to_pos(waypoint::sh_close[sm.sh_index_],5000);
        //sm.do_debug_pause("sh_close_stop");

        TailClawController::Instance().weapon_claw_open_ = false;
        osDelay(500);
        //sm.do_debug_pause("claw");
        sm.change_state_to(rotate_weapon_claw::instance());
    } STATE_END

    // 夹爪反转
    STATE(rotate_weapon_claw) {
        TailClawController::Instance().roll_target_deg_ = 1.0f;
        osDelay(1000);
        sm.wait_until_timeout_or([&sm]() -> bool {
            sm.tail_claw_update_status();
            return sm.tail_claw_status_valid_ && sm.tail_claw_status_cache_.roll_arrived;
        },5000, 20);
        sm.change_state_to(match_rod::instance());
    } STATE_END

    // 端头架对齐武器杆
    STATE(match_rod) {
        sm.move_to_pos(waypoint::sh_aim[sm.sh_index_].x, waypoint::sh_aim[sm.sh_index_].y + 300, waypoint::sh_aim[sm.sh_index_].yaw, 5000);
        osDelay(500);
        sm.move_to_pos(waypoint::sh_aim[sm.sh_index_].x, waypoint::sh_aim[sm.sh_index_].y + 300, -90, 5000);
        osDelay(500);
        if (g_config_area_type.load() == area_type::blue) {
            sm.move_to_pos(waypoint::match_rod_blue, 5000);
        } else {
            sm.move_to_pos(waypoint::match_rod_red, 5000);
        }
        sm.change_state_to(wait_for_decision_cmd::instance());
    } STATE_END

    // 等待操作手决策，决定是否拼装新的武器
    STATE(wait_for_decision_cmd) {
        sm.clean_previous_cmd();
        uint8_t cmd = 0x00;
        sm.wait_until([&]() -> bool {
            cmd = sm.get_cmd_from_r1();
            return (cmd == cmd_open_weapon_claw || cmd == cmd_catch_new_sh || cmd == cmd_go_to_mf);
        });
        switch (cmd) {
            case cmd_open_weapon_claw: // 松开夹爪
                TailClawController::Instance().weapon_claw_open_ = true;
                break;
            case cmd_catch_new_sh: // 夹取新的武器头
                if (sm.sh_index_ < SH_COUNT - 1) {
                    sm.sh_index_ += 1;
                    logger_queue.log("CLAW\tsh_index is now %d\n", sm.sh_index_);
                } else {
                    logger_queue.log("CLAW\tsh_index is at max!\n");   
                }
                sm.move_to_pos(500,0,-90,5000);
                osDelay(500);
                sm.move_to_pos(500,0,90,5000);
                osDelay(500);
                sm.change_state_to(go_to_shr::instance());
                return;
            case cmd_go_to_mf: // 进梅林
                sm.change_state_to(go_to_mf_entrance::instance());
                return;
        }
    } STATE_END

    // 前往梅林入口
    STATE(go_to_mf_entrance) {
        sm.move_to_pos(waypoint::mf_col(2));
        sm.change_state_to(request_for_path_cmd::instance());
    } STATE_END

    // 请求路径规划命令
    STATE(request_for_path_cmd) {
        sm.path_cmd_request_pub_.Publish(sm.current_path_cmd_index_); // 发一次 request

        path_cmd::code cmd;
        auto if_received = sm.wait_until_timeout_or([&]() -> bool {
            return sm.path_cmd_sub_.TryGet(&cmd);
        }, 3000); // 3秒超时

        if (!if_received) {
            logger_queue.log("PATH\ttimeout at path step No.%d! Check connection between PC and chip!\n", sm.current_path_cmd_index_);
            return; // 超时未收到指令，保持当前状态重试
        }

        logger_queue.log("PATH\treceived path step No.%d: 0x%02X\n", sm.current_path_cmd_index_, static_cast<uint8_t>(cmd));
        sm.current_path_cmd_index_ += 1;
        sm.current_path_cmd_ = cmd; // 给其他后续状态读取

        if (cmd == path_cmd::code::move_forward || cmd == path_cmd::code::move_backward) {
            sm.has_entered_mf = true;
        }

        switch (cmd) {
            case path_cmd::code::move_forward:
            case path_cmd::code::move_backward:
            case path_cmd::code::turn_left_90:
            case path_cmd::code::turn_right_90:
            case path_cmd::code::turn_around:
            case path_cmd::code::move_to_col1:
            case path_cmd::code::move_to_col2:
            case path_cmd::code::move_to_col3:
                sm.change_state_to(execute_chassis_action::instance());
                break;
            case path_cmd::code::grab_low_r2kfs:
            case path_cmd::code::grab_mid_r2kfs:
            case path_cmd::code::grab_high_r2kfs:
            case path_cmd::code::drop_and_grab_new_kfs:
                sm.change_state_to(execute_arm_action::instance());
                break;
            case path_cmd::code::no_more_commands:
                logger_queue.log("PATH\tPC sent no more commands! Path planning complete.\n");
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
                break;
            case path_cmd::code::move_to_col1:
                sm.current_mf_col = 1;
                sm.move_to_pos(waypoint::mf_col(1));
                break;
            case path_cmd::code::move_to_col2:
                sm.current_mf_col = 2;
                sm.move_to_pos(waypoint::mf_col(2));
                break;
            case path_cmd::code::move_to_col3:
                sm.current_mf_col = 3;
                sm.move_to_pos(waypoint::mf_col(3));
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

        arm_action::load_kfs();

        chassis_action::start_return_to_center();
        if (!sm.has_entered_mf) { // 梅林前的动作，夹取完往后退
            sm.move_to_pos(waypoint::mf_col(sm.current_mf_col).x - 300, waypoint::mf_col(sm.current_mf_col).y, 0, 5000, false);
        }

        sm.current_path_cmd_ = path_cmd::code::unknown; // 清空当前命令
        sm.change_state_to(request_for_path_cmd::instance());
    } STATE_END

    // 前往梅林出口
    STATE(go_to_mf_exit) {
        sm.change_state_to(stop::instance());
    } STATE_END

    // ===== 九宫藏宝 =====

    // 启动（九宫藏宝）
    STATE(begin_jgcb) {
        logger_queue.log("SM\tBEGIN JGCB ========\n");
        sm.clean_previous_cmd();
        sm.wait_until([&]() -> bool {
            return (sm.get_cmd_from_r1() == cmd_go_uphill);
        });
        // TODO: 改回 20秒！
        sm.countdown(10, "go_uphill", [&]() -> bool {
            return false;
        });
        sm.move_to_pos(waypoint::before_uphill);
        sm.change_state_to(go_to_arena::instance());
    } STATE_END

    // 上坡、前往竞技场
    STATE(go_to_arena) {
        sm.move_to_pos(waypoint::after_uphill);
        sm.move_to_pos(waypoint::after_uphill.x, waypoint::after_uphill.y, -90);
        sm.change_state_to(wait_for_arena_action::instance());
    } STATE_END

    STATE(wait_for_arena_action) {
        sm.clean_previous_cmd();
        sm.countdown(sm.current_startup_config_.arena_delay_seconds, "wait_for_arena_action", [&]() -> bool {
            return sm.get_cmd_from_r1() == cmd_go_uphill;
        });
        sm.change_state_to(go_to_grid::instance());
    } STATE_END

    STATE(retry_after_uphill) {
        sm.move_to_pos(waypoint::after_uphill);
        sm.change_state_to(go_to_grid::instance());
    } STATE_END

    // 导航至九宫格（首次进入或装载后返回）
    STATE(go_to_grid) {
        if (!sm.has_loaded_in_arena_) {
            // 首次进入：从坡道 / 重试区导航至九宫格
            sm.move_to_pos(waypoint::beside_after_uphill);
            if (sm.current_startup_config_.kfs_amount == 0) {
                // 无初始 KFS：跳过放置，直接进入装载流程
                sm.change_state_to(load_kfs::instance());
                return;
            }
        }
        sm.move_to_pos(waypoint::grid_mid);
        sm.change_state_to(wait_for_r1_cmd::instance());
    } STATE_END

    // 统一的放置等待状态：等待 R1 指令（放置KFS 或 合体）
    STATE(wait_for_r1_cmd) {
        logger_queue.log("SM\twaiting for R1 cmd...\n");
        logger_queue.log("SM\twait_for_r1_cmd: kfs_amount = %d\n",
            arm.get_kfs_amount()
        );
        logger_queue.log("SM\twait_for_r1_cmd: has_loaded_in_arena = %c\n",
            sm.has_loaded_in_arena_ ? 'T' : 'F'
        );
        logger_queue.log("SM\twait_for_r1_cmd: arena_load_kfs_amount = %d\n",
            sm.current_startup_config_.arena_load_kfs_amount
        );
        // 红方场地 Y 轴镜像，R1 视角的左右与地图坐标相反
        const bool is_red = (g_config_area_type.load() == area_type::red);
        const auto& L  = is_red ? waypoint::grid_right : waypoint::grid_left;
        const auto& LC = is_red ? waypoint::grid_right_close : waypoint::grid_left_close;
        const auto& R  = is_red ? waypoint::grid_left : waypoint::grid_right;
        const auto& RC = is_red ? waypoint::grid_left_close : waypoint::grid_right_close;

        sm.clean_previous_cmd();
        sm.wait_until([&sm, &L, &LC, &R, &RC]() -> bool {
            uint8_t cmd = sm.get_cmd_from_r1();

            // 合体指令：直接退出到合体流程
            if (cmd == cmd_combination) {
                sm.change_state_to(go_to_combination_area::instance());
                return true;
            }

            // 放置指令
            switch (cmd) {
                case cmd_place_kfs_on_left:
                    sm.do_place_kfs(L, LC);
                    break;
                case cmd_place_kfs_on_mid:
                    sm.do_place_kfs(waypoint::grid_mid, waypoint::grid_mid_close);
                    break;
                case cmd_place_kfs_on_right:
                    sm.do_place_kfs(R, RC);
                    break;
                default:
                    return false;
            }

            // 放置完毕后，判断是否需要去装载更多 KFS
            if (arm.get_kfs_amount() == 0
                && !sm.has_loaded_in_arena_
                && sm.current_startup_config_.arena_load_kfs_amount > 0) {
                logger_queue.log("SM\twait_for_r1_cmd: KFS placed. Going to load more KFS...");
                sm.change_state_to(load_kfs::instance());
                return true;
            }
            return false;  // 还有 KFS 或已装载过，留在等待循环
        });
    } STATE_END

    // 装填 KFS（从赛场装填点）
    STATE(load_kfs) {
        logger_queue.log("SM\tload_kfs: has_loaded_in_arena = %c\n",
            sm.has_loaded_in_arena_ ? 'T' : 'F'
        );
        logger_queue.log("SM\tload_kfs: arena_load_kfs_amount = %d\n",
            sm.current_startup_config_.arena_load_kfs_amount
        );
        logger_queue.log("SM\tload_kfs: kfs_amount = %d\n",
            arm.get_kfs_amount()
        );
        sm.has_loaded_in_arena_ = true;
        switch (sm.current_startup_config_.arena_load_kfs_amount) {
            case 0:
                logger_queue.log("SM\tload_kfs: loading 0 KFS due to startup config\n");
                break;
            case 1:
                logger_queue.log("SM\tload_kfs: loading 1/1 KFS\n");
                sm.move_to_pos(waypoint::load_kfs);
                arm_action::raise_kfs(LOAD_TYPE::PLAIN);
                break;
            case 2:
                logger_queue.log("SM\tload_kfs: loading 1/2 KFS\n");
                sm.move_to_pos(waypoint::load_kfs);
                arm_action::raise_kfs(LOAD_TYPE::PLAIN);
                arm_action::load_kfs();
                logger_queue.log("SM\tload_kfs: loading 2/2 KFS\n");
                sm.move_to_pos(waypoint::load_kfs_2);
                arm_action::raise_kfs(LOAD_TYPE::PLAIN);
                break;
            default:
                logger_queue.log("SM\tload_kfs: ERROR! Check startup config!\n",
                    sm.current_startup_config_.arena_load_kfs_amount);
                break;
        }
        logger_queue.log("SM\tload_kfs: Complete! KFS amount is now %d!\n", arm.get_kfs_amount());
        sm.change_state_to(go_to_grid::instance());
    } STATE_END

    // 前往合体点位
    STATE(go_to_combination_area) {
        // 慢慢退后到 R1 后面，准备合体
        sm.move_to_pos(waypoint::left_fence_front);
        sm.move_to_pos(waypoint::left_fence_back);
        sm.move_to_pos(waypoint::combination_area);
        sm.change_state_to(wait_for_combination_cmd::instance());
    } STATE_END

    // 等待合体指令
    STATE(wait_for_combination_cmd) {
        sm.clean_previous_cmd();
        sm.wait_until([&sm]() -> bool {
            return (sm.get_cmd_from_r1() == cmd_combination);
        });
        sm.change_state_to(begin_combination::instance());
    } STATE_END

    // 合体
    STATE(begin_combination) {
        logger_queue.log("CLIMB\tclimb R1 start\n");
        chassis_action::start_climb_R1();
        logger_queue.log("CLIMB\tclimb R1 end\n");
        sm.change_state_to(unload_kfs::instance());
    } STATE_END

    // 取出KFS并手持
    STATE(unload_kfs) {
        arm_action::unload_kfs(std::nullopt, true);
        sm.change_state_to(wait_for_release_kfs_cmd::instance());
    } STATE_END

    // 等待放置高层KFS的指令
    STATE(wait_for_release_kfs_cmd) {
        sm.clean_previous_cmd();
        sm.wait_until([&sm]() -> bool {
            return (sm.get_cmd_from_r1() == cmd_release_kfs);
        });
        sm.change_state_to(release_kfs::instance());
    } STATE_END

    // 释放KFS
    STATE(release_kfs) {
        arm_action::release_kfs();
        if (arm.get_kfs_amount() > 0) {
            logger_queue.log("ARM\t%d kfs remaining...", arm.get_kfs_amount());
            sm.change_state_to(unload_kfs::instance());
        } else {
            logger_queue.log("ARM\tNo kfs left!");
            sm.change_state_to(stop::instance());
        }
    } STATE_END

    // 等待取出KFS的指令
    // STATE(wait_for_unload_kfs_cmd) {
    //     sm.clean_previous_cmd();
    //     sm.wait_until([&sm]() -> bool {
    //         return (sm.get_cmd_from_r1() == cmd_unload_kfs);
    //     });
    //     sm.change_state_to(unload_kfs::instance());
    // } STATE_END

#undef STATE
#undef STATE_END

    StateMachine() = default;
    ~StateMachine() = default;

    void run() {
        current_state_->on_tick(*this);
    }

private:
    enum r1_cmd: uint8_t {
        cmd_open_weapon_claw = 0x0A,   // 松开武器头夹爪
        cmd_catch_new_sh = 0x1A,       // 夹取新的武器头
        cmd_go_to_mf = 0x1B,           // 进入梅林
        cmd_go_uphill = 0x2A,
        cmd_place_kfs_on_left = 0x3A,
        cmd_place_kfs_on_mid = 0x3B,
        cmd_place_kfs_on_right = 0x3C,
        cmd_combination = 0x4A,
        cmd_unload_kfs = 0x5A,
        cmd_release_kfs = 0x5B,
    };

    bool has_entered_mf{false}; // 是否进入了梅林（用于梅林前底盘动作）
    uint8_t current_mf_col{1};


    uint8_t sh_index_{0};        // 武器夹取计数
    bool has_loaded_in_arena_{false}; // 是否已进入过arena装载流程

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


    /**
     * @brief 等待上位机发送启动配置
     */
    void wait_for_startup_config() {
        logger_queue.log("SM\tstartup config waiting...\n");
        startup_config config{};
        wait_until([&]() -> bool {
            return startup_config_sub_.TryGet(&config);
        });

        auto clamp = [](int16_t val, int16_t min, int16_t max) -> int16_t {
            if (val < min) return min;
            if (val > max) return max;
            return val;
        };
        config.kfs_amount = clamp(config.kfs_amount, 0, 3);
        config.arena_load_kfs_amount = clamp(config.arena_load_kfs_amount, 0, 2);
        config.arena_delay_seconds = clamp(config.arena_delay_seconds, 10, 60);

        logger_queue.log("SM\tstartup config received:\n");
        logger_queue.log("SM\tarea_type_value = %s\n",
            config.area_type_value == area_type::blue ? "BLUE" : "RED"
        );
        logger_queue.log("SM\tbegin_type_value = %d\n",
            static_cast<int>(config.begin_type_value)
        );
        logger_queue.log("SM\tkfs_amount = %d\n",
            config.kfs_amount
        );
        logger_queue.log("SM\torigin_x = %d\n",
            config.origin_x
        );
        logger_queue.log("SM\torigin_y = %d\n",
            config.origin_y
        );
        logger_queue.log("SM\tarena_load_kfs_amount = %d\n",
            config.arena_load_kfs_amount
        );
        logger_queue.log("SM\tarena_delay_seconds = %d\n",
            config.arena_delay_seconds
        );
        // 在三区重试区启动时，将上位机发来的原点减去重试区相对启动区的坐标
        if (config.begin_type_value == begin_type::arena_retry_zone) {
            const auto& retry_zone = (config.area_type_value == area_type::blue) ? waypoint::retry_zone_blue : waypoint::retry_zone_red;
            config.origin_x -= retry_zone.x; // 假设重试区相对启动区的x坐标
            config.origin_y -= retry_zone.y; // 假设重试区相对启动区的y坐标
            logger_queue.log("SM\tRETRY MODE! origin_x = %d\n", config.origin_x);
            logger_queue.log("SM\tRETRY MODE! origin_y = %d\n", config.origin_y);
        }
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

    /**
     * @brief 倒计时，轮询间隔 100ms，可通过 break_when 提前跳出
     * @param seconds 倒计时秒数
     * @param name 名称，用于日志输出
     * @param log_interval 日志输出间隔（毫秒），默认 5000ms
     * @param break_when 提前跳出条件，传入 lambda 返回 true 时跳出
     */
    template <typename T>
    void countdown(uint32_t seconds, const char* name, T &&break_when, uint32_t log_interval = 5000) {
        const uint32_t start = osKernelGetTickCount();
        const uint32_t total_ms = seconds * 1000;
        uint32_t last_log = 0;

        logger_queue.log("CD\t%s countdown start: %lu s\n", name, (unsigned long)seconds);

        while (true) {
            const uint32_t elapsed = osKernelGetTickCount() - start;

            if (elapsed >= total_ms) {
                logger_queue.log("CD\t%s countdown done\n", name);
                return;
            }

            if (break_when()) {
                logger_queue.log("CD\t%s countdown broken at %lu ms\n", name, (unsigned long)elapsed);
                return;
            }

            if (elapsed - last_log >= log_interval) {
                const uint32_t remaining = (total_ms - elapsed) / 1000;
                logger_queue.log("CD\t%s countdown: %lu s remaining\n", name, (unsigned long)remaining);
                last_log = elapsed;
            }

            osDelay(100);
        }
    }

    void change_state_to(state& new_state) {
        logger_queue.log("SM\t-> %s\n", new_state.get_name());
        // do_debug_pause("change_state");
        current_state_ = &new_state;
    }

    /**
     * @brief 移动到指定位置
     * @note 务必在任务上下文里调用
     * @param x 目标位置x坐标
     * @param y 目标位置y坐标
     * @param yaw 目标位置朝向角度
     * @param enable_area_red_mirror 是否启用红区镜像变换
     * @param timeout_ms 超时时间，0表示不超时
     * @param name 目标位置名称，用于日志输出
     */
    bool move_to_pos(int16_t x, int16_t y, int16_t yaw, uint32_t timeout_ms = 0, bool enable_area_red_mirror = true, const char* name = nullptr) {
        int16_t prev_x, prev_y, prev_yaw;
        prev_x = x;
        prev_y = y;
        prev_yaw = yaw;
        // 红区镜像变换
        if (enable_area_red_mirror && g_config_area_type.load() == area_type::red) {
            y = -y;
            yaw = -yaw;
        }
        if (current_origin_location_.has_value()) {
            x += current_origin_location_->x;
            y += current_origin_location_->y;
        }

        if (name) {
            logger_queue.log("POS\t(%d, %d, %d) (%d, %d, %d) %s\n", prev_x, prev_y, prev_yaw, x, y, yaw, name);
        } else {
            logger_queue.log("POS\t(%d, %d, %d) (%d, %d, %d)\n", prev_x, prev_y, prev_yaw, x, y, yaw, name);
        }
        do_debug_pause("move_to_pos");

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

    /**
     * @brief 移动到指定位置
     * @param loc 目标位置
     * @param timeout_ms 超时时间，0表示不超时
     * @return true 表示成功到达目标位置
     */
    bool move_to_pos(const waypoint::location &loc, uint32_t timeout_ms = 0) {
        return move_to_pos(loc.x, loc.y, loc.yaw, timeout_ms, loc.enable_red_area_mirror, loc.name);
    }    /**
     * @brief 放置KFS到指定格子的公共逻辑
     * @param grid 目标格子位置
     * @param grid_close 贴合格子位置
     */
    void do_place_kfs(const waypoint::location& grid, const waypoint::location& grid_close) {
        auto res = arm_action::unload_kfs(std::nullopt);
        move_to_pos(grid, 5000);
        move_to_pos(grid_close);
        if (res) arm_action::release_kfs();
        move_to_pos(grid, 5000);
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
            logger_queue.log("R1-CMD\tir cmd: 0x%02X, uid=%d\n", cmd, ir_result.value().uid);
        }
        if (qr_code_sub_.TryGet(&qr_code_msg)) {
            cmd = qr_code_msg.data;
            logger_queue.log("R1-CMD\tqr cmd: 0x%02X\n", cmd);
        }
        
        // if (cmd != 0x00) {
        //     TailClawController::Instance().weapon_claw_open_ = true;
        //     osDelay(200);
        //     TailClawController::Instance().weapon_claw_open_ = false;
        //     osDelay(200);
        //     TailClawController::Instance().weapon_claw_open_ = true;
        // }
        return cmd;
    }

    /**
     * @brief 调试用的暂停函数，只有在 ENABLE_DEBUG_PAUSE 为 true 时才会生效
     * @param msg 日志信息，用于标识暂停的原因
     */
    void do_debug_pause(const char *msg) {
        if constexpr (ENABLE_DEBUG_PAUSE) {
            debug_pause = true;
            logger_queue.log("DEBUG\tpause at %s\n", msg);
            logger_queue.log("DEBUG\tDEBUG IS ON. Turn it off at state_machine_task.h!", msg);
            wait_until([]() -> bool { return !debug_pause; });
            logger_queue.log("DEBUG\t>> \n", msg);
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
