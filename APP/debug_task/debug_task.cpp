/**
 * @file debug_task.cpp
 * @brief Export runtime debug globals for Ozone inspection.
 */
#include "debug_task.h"

#include "cmsis_os2.h"
#include "task.h"

#include "NavProtocol.hpp"
#include "merlin_map/merlin_map.h"
#include "stair_assist.h"
#include "waypoint_navigator.hpp"

osThreadId_t Debug_TaskHandle;

volatile int32_t g_ozone_laser2_mm = 0;
volatile uint8_t g_ozone_laser2_fresh = 0;
volatile int32_t g_ozone_laser3_mm = 0;
volatile uint8_t g_ozone_laser3_fresh = 0;
volatile uint8_t g_ozone_laser3_profile = 0;
volatile int32_t g_ozone_laser3_near_min_used_mm = 0;
volatile int32_t g_ozone_laser3_near_max_used_mm = 0;

volatile uint8_t g_ozone_stair_assist_enabled = 0;
volatile uint8_t g_ozone_stair_assist_mode = 0;
volatile uint8_t g_ozone_stair_step = 0;
volatile uint8_t g_ozone_stair_level = 0;
volatile uint8_t g_ozone_stair_armed = 0;
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

void debugTask(void *argument) {
  (void)argument;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, 20);

    const auto &assist_debug = stairAssistDebug();
    const auto &map_debug = merlin_map::debug();

    g_ozone_laser2_mm = assist_debug.laser2_mm;
    g_ozone_laser2_fresh = assist_debug.laser2_fresh ? 1U : 0U;
    g_ozone_laser3_mm = assist_debug.laser3_mm;
    g_ozone_laser3_fresh = assist_debug.laser3_fresh ? 1U : 0U;
    g_ozone_laser3_profile = assist_debug.laser3_profile;
    g_ozone_laser3_near_min_used_mm = assist_debug.laser3_near_min_used_mm;
    g_ozone_laser3_near_max_used_mm = assist_debug.laser3_near_max_used_mm;

    g_ozone_stair_assist_enabled = assist_debug.enabled ? 1U : 0U;
    g_ozone_stair_assist_mode = assist_debug.assist_mode;
    g_ozone_stair_step = stairWaypointStep();
    g_ozone_stair_level = stairWaypointLevel();
    g_ozone_stair_armed = stairWaypointArmed() ? 1U : 0U;
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
  }
}
