/**
 * @file debug_task.h
 * @author 大帅将军
 * @brief 调试任务 — VOFA+ Firewater 底盘电机监视
 * @version 0.2
 * @date 2026-06-11
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------include-----------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdint.h>

/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void debugTask(void *argument);
/*------------------------------------test------------------------------------*/

extern volatile int32_t g_ozone_laser2_mm;
extern volatile uint32_t g_ozone_laser2_frame_count;
extern volatile uint8_t g_ozone_laser2_fresh;
extern volatile int32_t g_ozone_laser3_mm;
extern volatile uint32_t g_ozone_laser3_frame_count;
extern volatile uint8_t g_ozone_laser3_fresh;

extern volatile uint8_t g_ozone_stair_assist_enabled;
extern volatile uint8_t g_ozone_stair_assist_mode;
extern volatile uint8_t g_ozone_suggest_climb_up;
extern volatile uint8_t g_ozone_suggest_descend_high;
extern volatile uint8_t g_ozone_should_lower_after_climb;
extern volatile uint8_t g_ozone_should_lower_after_descend;
extern volatile uint8_t g_ozone_stair_step;
extern volatile uint8_t g_ozone_stair_level;
extern volatile uint8_t g_ozone_stair_armed;
extern volatile uint8_t g_ozone_robot_state;
extern volatile uint8_t g_ozone_high_mode_active;

extern volatile int32_t g_ozone_target_x;
extern volatile int32_t g_ozone_current_x;
extern volatile int32_t g_ozone_target_y;
extern volatile int32_t g_ozone_current_y;
extern volatile int32_t g_ozone_target_yaw;
extern volatile int32_t g_ozone_current_yaw;

extern volatile uint8_t g_ozone_merlin_cell_valid;
extern volatile uint8_t g_ozone_merlin_row;
extern volatile uint8_t g_ozone_merlin_col;
extern volatile int32_t g_ozone_merlin_height_mm;
extern volatile uint8_t g_ozone_merlin_heading;
extern volatile int32_t g_ozone_merlin_query_x;
extern volatile int32_t g_ozone_merlin_query_y;
extern volatile int32_t g_ozone_merlin_matched_center_x;
extern volatile int32_t g_ozone_merlin_matched_center_y;
extern volatile int32_t g_ozone_merlin_nearest_dist_sq;

extern volatile float g_ozone_vofa_nav_dist_mm;
extern volatile float g_ozone_vofa_nav_yaw_err_deg;
extern volatile float g_ozone_vofa_nav_blend;
extern volatile float g_ozone_vofa_nav_plan_speed_mps;
extern volatile float g_ozone_vofa_nav_v_ref_mps;
extern volatile float g_ozone_vofa_nav_pid_vx_mps;
extern volatile float g_ozone_vofa_nav_pid_vy_mps;
extern volatile float g_ozone_vofa_nav_pid_omega_radps;
extern volatile float g_ozone_vofa_nav_vx_cmd_mps;
extern volatile float g_ozone_vofa_nav_vy_cmd_mps;
extern volatile float g_ozone_vofa_nav_omega_cmd_radps;
extern volatile float g_ozone_vofa_nav_cmd_speed_mps;
extern volatile float g_ozone_vofa_nav_brake_dist_mm;
extern volatile uint8_t g_ozone_vofa_nav_arrive_hold_count;
extern volatile uint8_t g_ozone_vofa_nav_arrived_flag;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#endif
