/**
 * @file debug_task.cpp
 * @brief 调试任务，VOFA+ FireWater 输出
 */
#include "debug_task.h"

#include "task.h"
#include "cmsis_os2.h"

#include "logger.hpp"
#include "com_config.h"
#include "merlin_map/merlin_map.h"
#include "NavProtocol.hpp"
#include "stair_assist.h"
#include "state_machine_task.h"
#include "waypoint_navigator.hpp"

extern Logger logger;

osThreadId_t Debug_TaskHandle;

volatile int32_t g_ozone_laser2_mm = 0;
volatile uint32_t g_ozone_laser2_frame_count = 0;
volatile uint8_t g_ozone_laser2_fresh = 0;
volatile int32_t g_ozone_laser3_mm = 0;
volatile uint32_t g_ozone_laser3_frame_count = 0;
volatile uint8_t g_ozone_laser3_fresh = 0;

volatile uint8_t g_ozone_stair_assist_enabled = 0;
volatile uint8_t g_ozone_stair_assist_mode = 0;
volatile uint8_t g_ozone_suggest_climb_up = 0;
volatile uint8_t g_ozone_suggest_descend_high = 0;
volatile uint8_t g_ozone_should_lower_after_climb = 0;
volatile uint8_t g_ozone_should_lower_after_descend = 0;
volatile uint8_t g_ozone_stair_step = 0;
volatile uint8_t g_ozone_stair_level = 0;
volatile uint8_t g_ozone_stair_armed = 0;
volatile uint8_t g_ozone_robot_state = 0;
volatile uint8_t g_ozone_high_mode_active = 0;

volatile int32_t g_ozone_target_x = 0;
volatile int32_t g_ozone_current_x = 0;
volatile int32_t g_ozone_target_y = 0;
volatile int32_t g_ozone_current_y = 0;
volatile int32_t g_ozone_target_yaw = 0;
volatile int32_t g_ozone_current_yaw = 0;

volatile uint8_t g_ozone_merlin_cell_valid = 0;
volatile uint8_t g_ozone_merlin_row = 0;
volatile uint8_t g_ozone_merlin_col = 0;
volatile int32_t g_ozone_merlin_height_mm = 0;
volatile uint8_t g_ozone_merlin_heading = 0;
volatile int32_t g_ozone_merlin_query_x = 0;
volatile int32_t g_ozone_merlin_query_y = 0;
volatile int32_t g_ozone_merlin_matched_center_x = 0;
volatile int32_t g_ozone_merlin_matched_center_y = 0;
volatile int32_t g_ozone_merlin_nearest_dist_sq = 0;

volatile float g_ozone_vofa_nav_dist_mm = 0.0f;
volatile float g_ozone_vofa_nav_yaw_err_deg = 0.0f;
volatile float g_ozone_vofa_nav_blend = 0.0f;
volatile float g_ozone_vofa_nav_plan_speed_mps = 0.0f;
volatile float g_ozone_vofa_nav_v_ref_mps = 0.0f;
volatile float g_ozone_vofa_nav_pid_vx_mps = 0.0f;
volatile float g_ozone_vofa_nav_pid_vy_mps = 0.0f;
volatile float g_ozone_vofa_nav_pid_omega_radps = 0.0f;
volatile float g_ozone_vofa_nav_vx_cmd_mps = 0.0f;
volatile float g_ozone_vofa_nav_vy_cmd_mps = 0.0f;
volatile float g_ozone_vofa_nav_omega_cmd_radps = 0.0f;
volatile float g_ozone_vofa_nav_cmd_speed_mps = 0.0f;
volatile float g_ozone_vofa_nav_brake_dist_mm = 0.0f;
volatile uint8_t g_ozone_vofa_nav_arrive_hold_count = 0U;
volatile uint8_t g_ozone_vofa_nav_arrived_flag = 0U;

void debugTask(void *argument) {
  (void)argument;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, 20);
    const auto &map_debug = merlin_map::debug();
    const auto &assist_debug = stairAssistDebug();

    g_ozone_laser2_mm = assist_debug.laser2_mm;
    g_ozone_laser2_frame_count = assist_debug.laser2_frame_count;
    g_ozone_laser2_fresh = assist_debug.laser2_fresh ? 1U : 0U;
    g_ozone_laser3_mm = assist_debug.laser3_mm;
    g_ozone_laser3_frame_count = assist_debug.laser3_frame_count;
    g_ozone_laser3_fresh = assist_debug.laser3_fresh ? 1U : 0U;

    g_ozone_stair_assist_enabled = assist_debug.enabled ? 1U : 0U;
    g_ozone_stair_assist_mode = assist_debug.assist_mode;
    g_ozone_suggest_climb_up = assist_debug.suggest_climb_up ? 1U : 0U;
    g_ozone_suggest_descend_high =
        assist_debug.suggest_descend_high ? 1U : 0U;
    g_ozone_should_lower_after_climb =
        assist_debug.should_lower_after_climb ? 1U : 0U;
    g_ozone_should_lower_after_descend =
        assist_debug.should_lower_after_descend ? 1U : 0U;
    g_ozone_stair_step = stairWaypointStep();
    g_ozone_stair_level = stairWaypointLevel();
    g_ozone_stair_armed = stairWaypointArmed() ? 1U : 0U;
    // g_ozone_robot_state = static_cast<uint8_t>(get_current_state());
    g_ozone_high_mode_active = nav_control::high_mode_active ? 1U : 0U;

    g_ozone_target_x = nav_control::target_x;
    g_ozone_current_x = nav_control::current_x;
    g_ozone_target_y = nav_control::target_y;
    g_ozone_current_y = nav_control::current_y;
    g_ozone_target_yaw = nav_control::target_yaw;
    g_ozone_current_yaw = nav_control::current_yaw;

    g_ozone_merlin_cell_valid = map_debug.cell_valid ? 1U : 0U;
    g_ozone_merlin_row = map_debug.row;
    g_ozone_merlin_col = map_debug.col;
    g_ozone_merlin_height_mm = map_debug.height_mm;
    g_ozone_merlin_heading = map_debug.heading;
    g_ozone_merlin_query_x = map_debug.query_x;
    g_ozone_merlin_query_y = map_debug.query_y;
    g_ozone_merlin_matched_center_x = map_debug.matched_center_x;
    g_ozone_merlin_matched_center_y = map_debug.matched_center_y;
    g_ozone_merlin_nearest_dist_sq = map_debug.nearest_dist_sq;

    g_ozone_vofa_nav_dist_mm = g_ozone_nav_dist_mm;
    g_ozone_vofa_nav_yaw_err_deg = g_ozone_nav_yaw_err_deg;
    g_ozone_vofa_nav_blend = g_ozone_nav_blend;
    g_ozone_vofa_nav_plan_speed_mps = g_ozone_nav_plan_speed_mps;
    g_ozone_vofa_nav_v_ref_mps = g_ozone_nav_v_ref_mps;
    g_ozone_vofa_nav_pid_vx_mps = g_ozone_nav_pid_vx_mps;
    g_ozone_vofa_nav_pid_vy_mps = g_ozone_nav_pid_vy_mps;
    g_ozone_vofa_nav_pid_omega_radps = g_ozone_nav_pid_omega_radps;
    g_ozone_vofa_nav_vx_cmd_mps = g_ozone_nav_vx_cmd_mps;
    g_ozone_vofa_nav_vy_cmd_mps = g_ozone_nav_vy_cmd_mps;
    g_ozone_vofa_nav_omega_cmd_radps = g_ozone_nav_omega_cmd_radps;
    g_ozone_vofa_nav_cmd_speed_mps = g_ozone_nav_cmd_speed_mps;
    g_ozone_vofa_nav_brake_dist_mm = g_ozone_nav_brake_dist_mm;
    g_ozone_vofa_nav_arrive_hold_count = g_ozone_nav_arrive_hold_count;
    g_ozone_vofa_nav_arrived_flag = nav_control::arrived ? 1U : 0U;

    logger.log(
        "%ld,%ld,%ld,%ld,"
        "%ld,%ld,%.1f,%.2f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,"
        "%u,%u\n",
        static_cast<long>(nav_control::target_x),
        static_cast<long>(nav_control::current_x),
        static_cast<long>(nav_control::target_y),
        static_cast<long>(nav_control::current_y),
        static_cast<long>(nav_control::target_yaw),
        static_cast<long>(nav_control::current_yaw),
        g_ozone_nav_dist_mm,
        g_ozone_nav_yaw_err_deg,
        g_ozone_nav_blend,
        g_ozone_nav_plan_speed_mps,
        g_ozone_nav_v_ref_mps,
        g_ozone_nav_pid_vx_mps,
        g_ozone_nav_pid_vy_mps,
        g_ozone_nav_pid_omega_radps,
        g_ozone_nav_vx_cmd_mps,
        g_ozone_nav_cmd_speed_mps,
        g_ozone_nav_brake_dist_mm,
        g_ozone_nav_omega_cmd_radps,
        static_cast<unsigned>(g_ozone_nav_arrive_hold_count),
        static_cast<unsigned>(nav_control::arrived ? 1U : 0U));
  }
}
