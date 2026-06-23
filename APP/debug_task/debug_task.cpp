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
#include "Motor.hpp"
#include "NavProtocol.hpp"
#include "lift_task.h"
#include "stair_assist.h"
#include "state_machine_task.h"
#include "waypoint_navigator.hpp"

extern C620Motor chassis_motor1, chassis_motor2, chassis_motor3, chassis_motor4;
extern C610Motor lift_2006_motor1, lift_2006_motor2;
extern Logger logger;
extern float lift_2006_motor1_pid_out;
extern float lift_2006_motor2_pid_out;

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

    logger.log(
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.0f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,%.1f,"
        "%d,%d,%d,%d,%d,%d,%d,"
        "%ld,%d,%d,%d,%d,%d,%ld\n",
        chassis_motor1.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor1.getRawCurrentTorque(),
        chassis_motor1.getCurrentTemperature(),
        chassis_motor1.cmd_,

        chassis_motor2.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor2.getRawCurrentTorque(),
        chassis_motor2.getCurrentTemperature(),
        chassis_motor2.cmd_,

        chassis_motor3.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor3.getRawCurrentTorque(),
        chassis_motor3.getCurrentTemperature(),
        chassis_motor3.cmd_,

        chassis_motor4.getCurrentSpeed() / RPM_2_RAD_PER_SEC,
        chassis_motor4.getRawCurrentTorque(),
        chassis_motor4.getCurrentTemperature(),
        chassis_motor4.cmd_,

        lift_2006_motor1.getRawCurrentSpeed(),
        lift_2006_motor2.getRawCurrentSpeed(),
        lift_2006_motor1_pid_out,
        lift_2006_motor2_pid_out,

        nav_control::target_x,
        nav_control::current_x,
        nav_control::target_y,
        nav_control::current_y,
        static_cast<int>(stairWaypointStep()),
        static_cast<int>(stairWaypointLevel()),
        stairWaypointArmed() ? 1 : 0,
        assist_debug.laser2_mm,
        assist_debug.laser2_fresh ? 1 : 0,
        assist_debug.should_lower_after_descend ? 1 : 0,
        map_debug.cell_valid ? 1 : 0,
        static_cast<int>(map_debug.row),
        static_cast<int>(map_debug.col),
        static_cast<int>(map_debug.height_mm),
        static_cast<int>(map_debug.heading),
        static_cast<long>(map_debug.nearest_dist_sq));
  }
}
